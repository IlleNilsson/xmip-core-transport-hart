//! A field device on an in-process line, and both ends of one HART
//! exchange on it (ADR-0051).
//!
//! [`LoopbackLine`] is the line every test and every box without a HART
//! modem drives: what the master transmits, the device answers, and the
//! answer is what the master receives next. The loopback pair is a master
//! on that line and the device it writes a Stream to; the far end is the
//! device holding it, read back a chunk at a time. One line, one thread:
//! the round goes in order.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use transport::error::{Result, protocol_error};
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT, Loopback};
use transport::{Arrived, Transport};

use crate::device::{self, Device, Identity};
use crate::frame::{Address, Frame, Kind};
use crate::{HartTransport, Line};

/// A field device on an in-process line: what the master transmits, the
/// device answers, and the answer is what the master receives next.
pub struct LoopbackLine {
    device: Arc<Device>,
    to_master: Mutex<VecDeque<Vec<u8>>>,
}

impl LoopbackLine {
    #[must_use]
    pub fn new(device: Device) -> Self {
        Self {
            device: Arc::new(device),
            to_master: Mutex::new(VecDeque::new()),
        }
    }

    #[must_use]
    pub fn device(&self) -> &Device {
        &self.device
    }

    /// The device bursts its primary variable, asked by nobody.
    ///
    /// # Errors
    /// Never on this line; the signature is the trait's.
    pub fn burst(&self) -> Result<()> {
        let address = self.device.identity().address();
        let data = self.device.answer(device::READ_PRIMARY_VARIABLE, &[]);
        let frame = Frame::new(Kind::Back, address, 1, &data)?;
        self.queue(frame.encode());
        Ok(())
    }

    fn queue(&self, bytes: Vec<u8>) {
        self.to_master
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .push_back(bytes);
    }
}

impl Line for LoopbackLine {
    fn name(&self) -> &'static str {
        "loopback"
    }

    fn transmit(&self, bytes: &[u8]) -> Result<()> {
        let frame = Frame::decode(bytes)?;
        if frame.kind != Kind::Stx || !self.device.answers_to(&frame.address) {
            return Ok(());
        }
        let data = self.device.answer(u16::from(frame.command), &frame.data);
        let answer = Frame::new(Kind::Ack, frame.address, frame.command, &data)?;
        self.queue(answer.encode());
        Ok(())
    }

    fn receive(&self, _timeout: Duration) -> Result<Option<Vec<u8>>> {
        Ok(self
            .to_master
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .pop_front())
    }
}

impl HartTransport {
    /// Both ends on one line: a master at polling address 0 and the device
    /// that answers it, on a fresh [`LoopbackLine`], the loopback timeout
    /// on the master.
    #[must_use]
    pub fn loopback() -> Self {
        let device = Device::new(Identity {
            manufacturer: 0x26,
            device_type: 0xe5,
            device_id: 0x0a_1b2c,
        });
        Self::new(Arc::new(LoopbackLine::new(device)), Address::Short(0))
            .timing_out_after(LOOPBACK_TIMEOUT)
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

    /// A fresh master on the same line writes to the device at `address`.
    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        Self::new(Arc::clone(&self.line), self.address.clone())
            .timing_out_after(self.timeout)
            .send(address, payload)
    }

    fn unblock(&self, _address: &str) {
        // The line is in-process; nothing listens on a socket.
    }

    /// In order on one thread: a serial line has one master, so the write
    /// goes first and the read-back finds what it left.
    fn round(&self, payload: &[u8]) -> Result<Arrived> {
        let far = self.far_end()?;
        self.send_to(far.address(), payload)?;
        let arrived = far.take_one()?;
        if arrived.bytes != payload {
            return Err(protocol_error("written, but what was read back differs"));
        }
        Ok(arrived)
    }
}
