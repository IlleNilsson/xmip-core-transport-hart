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
//! The carrier is a [`Line`]: a HART modem on a serial port, framed by
//! [`Frame::measure`], or the serial technology's multi-drop bus in process,
//! with field devices on it at their own polling addresses —
//! [`loopback::OnTheBus`] — which every test and every box without a modem
//! drives (open problem 24). A HART-IP gateway is a line too. `WirelessHART`
//! carries the same commands over the air and rides on this crate's
//! [`device`] for them.
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
use transport::error::{Result, protocol_error};
use transport::line::Line;
use transport::{Arrived, Directions, Transport};

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
    use crate::loopback::OnTheBus;
    use sdk::serial::Bus;
    use transport::loopback::Loopback;
    use transport::payload::edge_payloads;

    fn identity() -> Identity {
        Identity {
            manufacturer: 0x26,
            device_type: 0xe5,
            device_id: 0x0a_1b2c,
        }
    }

    /// `device` alone on a fresh bus, and the bus.
    fn on_a_bus(device: Device) -> (Arc<Bus>, Arc<OnTheBus>) {
        let bus = Arc::new(Bus::new("loopback"));
        let device = Arc::new(OnTheBus(Arc::new(device)));
        bus.attach(Arc::clone(&device) as Arc<dyn sdk::serial::Device>);
        (bus, device)
    }

    fn master(bus: &Arc<Bus>, address: Address) -> HartTransport {
        HartTransport::new(Arc::clone(bus) as Arc<dyn Line>, address)
            .timing_out_after(Duration::from_millis(10))
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
        let edges = edge_payloads();
        for (name, payload) in edges {
            assert_eq!(master.round(&payload).expect(name).bytes, payload, "{name}");
        }
    }

    #[test]
    fn the_device_identifies_itself_and_bursts_its_variable() {
        let (bus, device) = on_a_bus(Device::new(identity()).measuring(12, 4.5));
        let master = master(&bus, Address::Short(0));
        assert_eq!(master.identify().expect("identify"), identity());
        assert!(
            master.receive().expect("quiet").is_empty(),
            "nothing is not an error"
        );
        bus.speak(device.burst().expect("burst"));
        let arrived = master.receive().expect("burst");
        assert_eq!(arrived.len(), 1);
        assert_eq!(arrived[0].bytes, [12, 0x40, 0x90, 0, 0]);
        assert_eq!(
            arrived[0].origin_uri,
            "hart://loopback/26-e5-0a1b2c?command=1&burst=true"
        );
    }

    #[test]
    fn on_one_bus_only_the_polled_device_answers() {
        let (bus, _) = on_a_bus(Device::new(identity()).at_polling_address(3));
        let other = Identity {
            manufacturer: 0x26,
            device_type: 0xe6,
            device_id: 0x00_0001,
        };
        bus.attach(Arc::new(OnTheBus(Arc::new(
            Device::new(other.clone()).at_polling_address(5),
        ))));
        let error = master(&bus, Address::Short(0))
            .identify()
            .expect_err("silence");
        assert!(error.retryable, "a device may be slow to answer");
        assert!(
            master(&bus, Address::Short(0))
                .send("hart://loopback/64", b"x")
                .is_err(),
            "not an address"
        );
        assert_eq!(
            master(&bus, Address::Short(3)).identify().expect("three"),
            identity()
        );
        assert_eq!(
            master(&bus, Address::Short(5)).identify().expect("five"),
            other
        );
    }

    #[test]
    fn a_frame_is_measured_by_its_preambles_address_and_count() {
        let short = Frame::new(Kind::Stx, Address::Short(2), 0, &[1, 2]).expect("short");
        let long = Frame::new(Kind::Ack, identity().address(), 1, &[9]).expect("long");
        for frame in [short, long] {
            let bytes = frame.encode();
            let whole = (1..=bytes.len())
                .find_map(|read| Frame::measure(&bytes[..read]).expect("measures"))
                .expect("a length");
            assert_eq!(whole, bytes.len());
        }
        assert!(Frame::measure(&[0xff, 0x02]).is_err(), "one preamble");
        assert!(Frame::measure(&[0xff; 21]).is_err(), "no end of preambles");
    }

    #[test]
    fn a_line_has_no_artefact_to_claim() {
        let (bus, _) = on_a_bus(Device::new(identity()));
        let master = master(&bus, Address::Short(0));
        assert!(master.claims().is_none());
        assert_eq!(master.name(), "hart");
        assert!(master.directions().receives() && master.directions().sends());
    }
}
