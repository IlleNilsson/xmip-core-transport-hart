#![forbid(unsafe_code)]

//! Streams that arrive from HART field devices. One command's data is one
//! Stream: a burst-mode device's variable as it comes, or a Stream the
//! device holds, read back a chunk at a time.
//!
//! HART is the 4-20 mA loop's second channel: a 1200-baud FSK signal on
//! the same two wires, a master asking and a transmitter answering, on
//! every process plant built since the eighties. What is here is the frame
//! — preambles, a delimiter, a short or long address, a command, the data
//! and an XOR — commands 0 and 1, and two device-specific commands that
//! carry a Stream in chunks, which is how a device takes anything larger
//! than a frame. A Send Location writes a Stream to a device; a Receive
//! Location takes what the line carries, a burst-mode device's variable
//! included, or reads the Stream a device holds.
//!
//! The line is a trait: [`LoopbackLine`] is a field device on an in-process
//! line, which every test and every box without a HART modem drives, the
//! way can-bus drives its loopback bus. A deployment's line is a HART modem
//! on a serial port — the `xmip-core-transport-serial` technology, once it
//! exposes its port — or a HART-IP gateway. `WirelessHART` carries the same
//! commands over the air and rides on this crate's [`device`] for them.
//!
//! The origin URI carries what the frame knew:
//! `hart://loopback/26-e5-0a1b2c?command=1&burst=true`.

pub mod device;
pub mod frame;
pub mod loopback;

use std::sync::Arc;
use std::time::Duration;

pub use device::{Device, Identity};
pub use frame::{Address, Frame, Kind};
pub use loopback::LoopbackLine;
use transport::error::{Result, protocol_error};
use transport::{Arrived, Directions, Transport};

/// Where frames go and come from.
pub trait Line: Send + Sync {
    /// The line's name, for the origin URI.
    fn name(&self) -> &str;
    /// Put a frame on the line.
    ///
    /// # Errors
    /// Where the line refused it.
    fn transmit(&self, frame: &[u8]) -> Result<()>;
    /// The next frame, or `None` when nothing arrived within `timeout`.
    ///
    /// # Errors
    /// Where the line could not be read.
    fn receive(&self, timeout: Duration) -> Result<Option<Vec<u8>>>;
}

/// The master's side of a line.
#[derive(Clone)]
pub struct HartTransport {
    line: Arc<dyn Line>,
    address: Address,
    timeout: Duration,
}

impl HartTransport {
    /// A master on `line`, talking to the device at `address`.
    #[must_use]
    pub fn new(line: Arc<dyn Line>, address: Address) -> Self {
        Self {
            line,
            address,
            timeout: Duration::from_secs(1),
        }
    }

    /// Give up on a device that does not answer within `timeout`.
    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// `hart://<line>/<address>`.
    #[must_use]
    pub fn origin(&self, address: &Address) -> String {
        format!("hart://{}/{address}", self.line.name())
    }

    /// Ask the device at `address` `command` with `data`; its answer past
    /// the status bytes.
    ///
    /// # Errors
    /// No answer in time, an answer to something else, or a response code
    /// that is not OK.
    pub fn request(&self, address: &Address, command: u8, data: &[u8]) -> Result<Vec<u8>> {
        let request = Frame::new(Kind::Stx, address.clone(), command, data)?;
        self.line.transmit(&request.encode())?;
        loop {
            let bytes = self
                .line
                .receive(self.timeout)?
                .ok_or_else(|| transport::TransportError::retryable("the device did not answer"))?;
            let answer = Frame::decode(&bytes)?;
            if answer.kind == Kind::Ack && answer.command == command {
                return device::answered(&answer.data).map(<[u8]>::to_vec);
            }
        }
    }

    /// Command 0: who the device is.
    ///
    /// # Errors
    /// As [`Self::request`].
    pub fn identify(&self) -> Result<Identity> {
        Identity::from_identify(&self.request(&self.address, 0, &[])?)
    }

    /// Write `bytes` to the device at `address`, a chunk per request.
    ///
    /// # Errors
    /// As [`Self::request`].
    pub fn write_stream(&self, address: &Address, bytes: &[u8]) -> Result<()> {
        let command = u8::try_from(device::WRITE_STREAM).unwrap_or(u8::MAX);
        for request in device::write_requests(bytes, device::MAX_CHUNK) {
            self.request(address, command, &request)?;
        }
        Ok(())
    }

    /// Read the Stream the device holds, a chunk per request.
    ///
    /// # Errors
    /// As [`Self::request`], or a device that never says "last".
    pub fn read_stream(&self) -> Result<Arrived> {
        let command = u8::try_from(device::READ_STREAM).unwrap_or(u8::MAX);
        let mut bytes = Vec::new();
        for index in 0..u32::MAX {
            let answer = self.request(&self.address, command, &device::read_request(index))?;
            let (last, chunk) = device::read_answer(&answer)?;
            bytes.extend_from_slice(chunk);
            if last {
                let origin = format!("{}?command={command}", self.origin(&self.address));
                return Ok(Arrived::new(origin, bytes));
            }
        }
        Err(protocol_error("a Stream that never ends"))
    }

    /// The next frame on the line as a Stream — a burst, or an answer
    /// nobody was waiting for — or `None` when the line is quiet.
    ///
    /// # Errors
    /// Where the line could not be read or carried something that is not a
    /// frame.
    pub fn receive_one(&self) -> Result<Option<Arrived>> {
        let Some(bytes) = self.line.receive(self.timeout)? else {
            return Ok(None);
        };
        let frame = Frame::decode(&bytes)?;
        let origin = format!(
            "{}?command={}&burst={}",
            self.origin(&frame.address),
            frame.command,
            frame.kind == Kind::Back
        );
        Ok(Some(Arrived::new(
            origin,
            device::answered(&frame.data)?.to_vec(),
        )))
    }
}

impl Transport for HartTransport {
    fn name(&self) -> &'static str {
        "hart"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    /// Nothing on the line is not an error: an empty vector.
    fn receive(&self) -> Result<Vec<Arrived>> {
        Ok(self.receive_one()?.into_iter().collect())
    }

    /// `target` may name an address, `hart://line/7`, overriding the
    /// transport's.
    fn send(&self, target: &str, bytes: &[u8]) -> Result<()> {
        let address = match transport::socket::target("hart", target) {
            Some((_, address)) if !address.is_empty() => Address::parse(address)?,
            _ => self.address.clone(),
        };
        self.write_stream(&address, bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use transport::loopback::Loopback;

    fn identity() -> Identity {
        Identity {
            manufacturer: 0x26,
            device_type: 0xe5,
            device_id: 0x0a_1b2c,
        }
    }

    fn master(line: &Arc<LoopbackLine>) -> HartTransport {
        let line: Arc<dyn Line> = Arc::clone(line) as Arc<dyn Line>;
        HartTransport::new(line, Address::Short(0)).timing_out_after(Duration::from_millis(10))
    }

    #[test]
    fn a_stream_goes_out_to_the_device_and_comes_back_whole() {
        let master = HartTransport::loopback();
        let long: Vec<u8> = (0..3000u32)
            .map(|n| u8::try_from(n % 251).unwrap_or(0))
            .collect();
        let back = master.round(&long).expect("round");
        assert_eq!(back.bytes, long);
        assert_eq!(back.origin_uri, "hart://loopback/0?command=131");
        master
            .send("hart://loopback/26-e5-0a1b2c", b"")
            .expect("empty");
        assert!(master.read_stream().expect("reading").bytes.is_empty());
        assert!(master.ceiling().is_none());
        assert!(master.refuses(&long).is_none());
    }

    #[test]
    fn the_loopback_returns_the_edges_whole() {
        let master = HartTransport::loopback();
        let edges: [(&str, Vec<u8>); 6] = [
            ("empty", Vec::new()),
            ("one byte", vec![0x2a]),
            ("every byte", (0..=255).collect()),
            ("nul run", vec![0; 512]),
            ("high bytes", vec![0xff; 512]),
            ("crlf storm", b"\r\n".repeat(400)),
        ];
        for (name, payload) in edges {
            assert_eq!(master.round(&payload).expect(name).bytes, payload, "{name}");
        }
    }

    #[test]
    fn the_device_identifies_itself_and_bursts_its_variable() {
        let line = Arc::new(LoopbackLine::new(
            Device::new(identity()).measuring(12, 4.5),
        ));
        let master = master(&line);
        assert_eq!(master.identify().expect("identify"), identity());
        assert!(
            master.receive().expect("quiet").is_empty(),
            "nothing is not an error"
        );
        line.burst().expect("burst");
        let arrived = master.receive().expect("burst");
        assert_eq!(arrived.len(), 1);
        assert_eq!(arrived[0].bytes, [12, 0x40, 0x90, 0, 0]);
        assert_eq!(
            arrived[0].origin_uri,
            "hart://loopback/26-e5-0a1b2c?command=1&burst=true"
        );
    }

    #[test]
    fn a_device_that_is_not_addressed_does_not_answer() {
        let line = Arc::new(LoopbackLine::new(
            Device::new(identity()).at_polling_address(3),
        ));
        let master = master(&line);
        let error = master.identify().expect_err("silence");
        assert!(error.retryable, "a device may be slow to answer");
        assert!(
            master.send("hart://loopback/64", b"x").is_err(),
            "not an address"
        );
        let found = HartTransport::new(line, Address::Short(3));
        assert_eq!(found.identify().expect("identify"), identity());
    }

    #[test]
    fn a_line_has_no_artefact_to_claim() {
        let line = Arc::new(LoopbackLine::new(Device::new(identity())));
        let master = master(&line);
        assert!(master.claims().is_none());
        assert_eq!(master.name(), "hart");
        assert!(master.directions().receives() && master.directions().sends());
    }
}
