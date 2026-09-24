//! A field device on the serial technology's multi-drop bus, and both ends
//! of one HART exchange on it (ADR-0051).
//!
//! [`OnTheBus`] is a field device as a device on the SDK's [`Bus`]: it
//! hears every frame the master puts on the bus and answers the ones
//! addressed to it, as a transmitter on a real loop does; a device at another
//! polling address keeps silent. A device in burst mode speaks unasked, which
//! [`OnTheBus::burst`] puts on the bus. The loopback pair is a master and one
//! device on a fresh bus; the far end is the device holding what the master
//! wrote, read back a chunk at a time.
//!
//! Until 2026-09-24 the device sat on a line of its own that no other device
//! could share (open problem 24).

use std::sync::Arc;

use sdk::serial::{self, Bus};
use transport::error::Result;
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};
use transport::{Arrived, Transport};

use crate::HartTransport;
use crate::device::{self, Device, Identity};
use crate::frame::{Address, Frame, Kind};

/// A field device as a device on the bus.
pub struct OnTheBus(pub Arc<Device>);

impl OnTheBus {
    /// The frame a device in burst mode puts on the bus unasked: its primary
    /// variable, to be spoken with [`Bus::speak`].
    ///
    /// # Errors
    /// A variable no frame carries.
    pub fn burst(&self) -> Result<Vec<u8>> {
        let address = self.0.identity().address();
        let data = self.0.answer(device::READ_PRIMARY_VARIABLE, &[]);
        Ok(Frame::new(Kind::Back, address, 1, &data)?.encode())
    }
}

impl serial::Device for OnTheBus {
    fn hear(&self, bytes: &[u8]) -> Result<Option<Vec<u8>>> {
        let frame = Frame::decode(bytes)?;
        if frame.kind != Kind::Stx || !self.0.answers_to(&frame.address) {
            return Ok(None);
        }
        let data = self.0.answer(u16::from(frame.command), &frame.data);
        Ok(Some(
            Frame::new(Kind::Ack, frame.address, frame.command, &data)?.encode(),
        ))
    }
}

impl HartTransport {
    /// Both ends on one bus: a master at polling address 0 and the device
    /// that answers it, on a fresh [`Bus`], the loopback timeout on the
    /// master.
    #[must_use]
    pub fn loopback() -> Self {
        let device = Device::new(Identity {
            manufacturer: 0x26,
            device_type: 0xe5,
            device_id: 0x0a_1b2c,
        });
        let bus = Bus::new("loopback");
        bus.attach(Arc::new(OnTheBus(Arc::new(device))));
        Self::new(Arc::new(bus), Address::Short(0)).timing_out_after(LOOPBACK_TIMEOUT)
    }
}

/// The device on the line, holding what the master wrote until it is read
/// back.
struct Holding {
    master: HartTransport,
    address: String,
}

impl FarEnd for Holding {
    fn address(&self) -> &str {
        &self.address
    }

    fn take_one(self: Box<Self>) -> Result<Arrived> {
        self.master.read_stream()
    }
}

impl Loopback for HartTransport {
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        Ok(Box::new(Holding {
            master: self.clone(),
            address: self.origin(&self.address),
        }))
    }

    /// A fresh master on the same bus writes to the device at `address`.
    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        Self::new(Arc::clone(&self.line), self.address.clone())
            .timing_out_after(self.timeout)
            .send(address, payload)
    }

    fn unblock(&self, _address: &str) {
        // The bus is in-process; nothing listens on a socket.
    }

    /// In order on one thread: a serial line has one master, so the write
    /// goes first and the read-back finds what it left.
    fn round(&self, payload: &[u8]) -> Result<Arrived> {
        self.round_in_order(payload)
    }
}
