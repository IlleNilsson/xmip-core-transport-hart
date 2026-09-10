//! One field device's worth of HART: the commands it answers, and the
//! two status bytes every answer opens with. Command numbers are sixteen
//! bits because `WirelessHART` carries them so; the wired frame carries the
//! low byte.
//!
//! Command 0 identifies the device, command 1 reads its primary variable,
//! and two device-specific commands carry a Stream: 130 writes it a chunk
//! at a time, 131 reads it back the same way. That is how a HART device
//! takes something larger than a frame — configuration, a totaliser log —
//! and it is what a Location sends.

use std::sync::Mutex;

use transport::error::{Result, protocol_error};

use crate::frame::Address;

/// Read unique identifier.
pub const IDENTIFY: u16 = 0;
/// Read primary variable.
pub const READ_PRIMARY_VARIABLE: u16 = 1;
/// Write one chunk of a Stream: a flags byte, then the bytes.
pub const WRITE_STREAM: u16 = 130;
/// Read one chunk of a Stream: a chunk index, answered with a flags byte
/// and the bytes.
pub const READ_STREAM: u16 = 131;

/// The chunk is the Stream's first.
pub const FIRST: u8 = 0x40;
/// The chunk is the Stream's last.
pub const LAST: u8 = 0x80;
/// The most a chunk holds on the wire: the frame's data less a flags byte
/// and, on the way back, the two status bytes.
pub const MAX_CHUNK: usize = 252;

/// The response codes a device answers with in the first status byte.
pub const OK: u8 = 0;
pub const INVALID_SELECTION: u8 = 2;
pub const TOO_FEW_BYTES: u8 = 5;
pub const NOT_IMPLEMENTED: u8 = 64;

/// What command 0 says.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Identity {
    pub manufacturer: u8,
    pub device_type: u8,
    pub device_id: u32,
}

impl Identity {
    /// The long-frame address this identity answers to.
    #[must_use]
    pub const fn address(&self) -> Address {
        Address::Long {
            manufacturer: self.manufacturer,
            device_type: self.device_type,
            device_id: self.device_id,
        }
    }

    /// The twelve bytes command 0 answers with: the 254 that says
    /// "expanded", the manufacturer, the device type, five preambles, the
    /// revisions, the flags and the device identifier.
    #[must_use]
    pub fn identify(&self) -> Vec<u8> {
        let mut out = vec![254, self.manufacturer, self.device_type, 5, 5, 1, 1, 1, 0];
        out.extend_from_slice(&self.device_id.to_be_bytes()[1..]);
        out
    }

    /// The identity a command 0 answer names.
    ///
    /// # Errors
    /// An answer shorter than twelve bytes or not opening with 254.
    pub fn from_identify(data: &[u8]) -> Result<Self> {
        if data.len() < 12 || data[0] != 254 {
            return Err(protocol_error("not a command 0 answer"));
        }
        Ok(Self {
            manufacturer: data[1],
            device_type: data[2],
            device_id: u32::from_be_bytes([0, data[9], data[10], data[11]]),
        })
    }
}

/// A field device: an identity, a polling address, a primary variable and
/// the Stream it holds.
pub struct Device {
    identity: Identity,
    polling: u8,
    units: u8,
    primary_variable: f32,
    chunk: usize,
    held: Mutex<Vec<u8>>,
    arriving: Mutex<Vec<u8>>,
}

impl Device {
    /// A device at polling address 0, reading 20.0 in units code 6 (psi),
    /// answering reads in chunks of [`MAX_CHUNK`].
    #[must_use]
    pub fn new(identity: Identity) -> Self {
        Self {
            identity,
            polling: 0,
            units: 6,
            primary_variable: 20.0,
            chunk: MAX_CHUNK,
            held: Mutex::new(Vec::new()),
            arriving: Mutex::new(Vec::new()),
        }
    }

    #[must_use]
    pub const fn at_polling_address(mut self, polling: u8) -> Self {
        self.polling = polling & 0x3f;
        self
    }

    #[must_use]
    pub const fn measuring(mut self, units: u8, value: f32) -> Self {
        self.units = units;
        self.primary_variable = value;
        self
    }

    /// Answer command 131 with chunks of at most `chunk` bytes — the wire's
    /// frame allows [`MAX_CHUNK`], a radio's payload less.
    #[must_use]
    pub const fn chunking(mut self, chunk: usize) -> Self {
        self.chunk = chunk;
        self
    }

    #[must_use]
    pub const fn identity(&self) -> &Identity {
        &self.identity
    }

    /// Whether a frame at `address` is for this device.
    #[must_use]
    pub fn answers_to(&self, address: &Address) -> bool {
        match address {
            Address::Short(polling) => *polling == self.polling,
            Address::Long { .. } => *address == self.identity.address(),
        }
    }

    /// The Stream the device holds, as the last complete write left it.
    #[must_use]
    pub fn held(&self) -> Vec<u8> {
        self.held
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }

    /// Command 1's answer: the units code and the value as a float.
    #[must_use]
    pub fn primary_variable(&self) -> Vec<u8> {
        let mut out = vec![self.units];
        out.extend_from_slice(&self.primary_variable.to_be_bytes());
        out
    }

    /// The answer to `command` with `data`: two status bytes, then the
    /// command's data where the code is [`OK`].
    #[must_use]
    pub fn answer(&self, command: u16, data: &[u8]) -> Vec<u8> {
        let (code, body) = match command {
            IDENTIFY => (OK, self.identity.identify()),
            READ_PRIMARY_VARIABLE => (OK, self.primary_variable()),
            WRITE_STREAM => (self.write(data), Vec::new()),
            READ_STREAM => self.read(data),
            _ => (NOT_IMPLEMENTED, Vec::new()),
        };
        let mut out = vec![code, 0];
        out.extend(body);
        out
    }

    fn write(&self, data: &[u8]) -> u8 {
        let Some((flags, chunk)) = data.split_first() else {
            return TOO_FEW_BYTES;
        };
        let mut arriving = self
            .arriving
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        if flags & FIRST != 0 {
            arriving.clear();
        }
        arriving.extend_from_slice(chunk);
        if flags & LAST != 0 {
            *self
                .held
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner) = std::mem::take(&mut arriving);
        }
        OK
    }

    fn read(&self, data: &[u8]) -> (u8, Vec<u8>) {
        let Ok(index) = <[u8; 4]>::try_from(data) else {
            return (TOO_FEW_BYTES, Vec::new());
        };
        let index = u32::from_be_bytes(index) as usize;
        let held = self.held();
        let chunks = held.chunks(self.chunk).count().max(1);
        if index >= chunks {
            return (INVALID_SELECTION, Vec::new());
        }
        let last = if index + 1 == chunks { LAST } else { 0 };
        let first = if index == 0 { FIRST } else { 0 };
        let mut out = vec![first | last];
        out.extend_from_slice(held.chunks(self.chunk).nth(index).unwrap_or(&[]));
        (OK, out)
    }
}

/// The data past an answer's two status bytes.
///
/// # Errors
/// An answer without its status bytes, or a response code that is not
/// [`OK`] — a device that refused is not a device to ask again.
pub fn answered(response: &[u8]) -> Result<&[u8]> {
    match response {
        [OK, _, data @ ..] => Ok(data),
        [code, ..] => Err(protocol_error(format!(
            "the device answered with response code {code}"
        ))),
        [] => Err(protocol_error("an answer without its status bytes")),
    }
}

/// `bytes` as the command 130 requests that write it, `chunk` bytes at a
/// time, each a flags byte then the bytes. An empty Stream is one request.
#[must_use]
pub fn write_requests(bytes: &[u8], chunk: usize) -> Vec<Vec<u8>> {
    let chunk = chunk.clamp(1, MAX_CHUNK);
    let pieces: Vec<&[u8]> = if bytes.is_empty() {
        vec![&[][..]]
    } else {
        bytes.chunks(chunk).collect()
    };
    let count = pieces.len();
    pieces
        .into_iter()
        .enumerate()
        .map(|(i, piece)| {
            let flags = if i == 0 { FIRST } else { 0 } | if i + 1 == count { LAST } else { 0 };
            let mut out = Vec::with_capacity(piece.len() + 1);
            out.push(flags);
            out.extend_from_slice(piece);
            out
        })
        .collect()
}

/// The command 131 request for chunk `index`.
#[must_use]
pub fn read_request(index: u32) -> Vec<u8> {
    index.to_be_bytes().to_vec()
}

/// A command 131 answer's data: whether it was the last chunk, and the bytes.
///
/// # Errors
/// An answer without its flags byte.
pub fn read_answer(data: &[u8]) -> Result<(bool, &[u8])> {
    let (flags, chunk) = data
        .split_first()
        .ok_or_else(|| protocol_error("a chunk without its flags"))?;
    Ok((flags & LAST != 0, chunk))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device() -> Device {
        Device::new(Identity {
            manufacturer: 0x26,
            device_type: 0xe5,
            device_id: 0x0a_1b2c,
        })
    }

    #[test]
    fn command_zero_identifies_and_command_one_reads_the_variable() {
        let device = device().measuring(12, 4.5);
        let zero = device.answer(IDENTIFY, &[]);
        assert_eq!(&zero[..2], &[OK, 0]);
        let identity = Identity::from_identify(answered(&zero).expect("ok")).expect("id");
        assert_eq!(&identity, device.identity());
        assert!(device.answers_to(&identity.address()));
        assert!(device.answers_to(&Address::Short(0)));
        assert!(!device.answers_to(&Address::Short(1)));
        let pv = device.answer(READ_PRIMARY_VARIABLE, &[]);
        assert_eq!(answered(&pv).expect("ok"), [12, 0x40, 0x90, 0, 0]);
        assert_eq!(device.answer(77, &[])[0], NOT_IMPLEMENTED);
        assert!(answered(&[NOT_IMPLEMENTED, 0]).is_err());
        assert!(answered(&[]).is_err());
        assert!(Identity::from_identify(&[1, 2, 3]).is_err());
    }

    #[test]
    fn a_stream_is_written_in_chunks_and_read_back_in_chunks() {
        let device = device().chunking(4);
        let stream: Vec<u8> = (0..10).collect();
        let requests = write_requests(&stream, 3);
        assert_eq!(requests.len(), 4);
        assert_eq!(requests[0], [FIRST, 0, 1, 2]);
        assert_eq!(requests[3], [LAST, 9]);
        for request in &requests {
            assert_eq!(device.answer(WRITE_STREAM, request), [OK, 0]);
        }
        assert_eq!(device.held(), stream);
        let mut back = Vec::new();
        for index in 0.. {
            let answer = device.answer(READ_STREAM, &read_request(index));
            let (last, chunk) = read_answer(answered(&answer).expect("ok")).expect("chunk");
            back.extend_from_slice(chunk);
            if last {
                break;
            }
        }
        assert_eq!(back, stream);
        assert_eq!(
            device.answer(READ_STREAM, &read_request(3))[0],
            INVALID_SELECTION
        );
        assert_eq!(device.answer(READ_STREAM, &[1])[0], TOO_FEW_BYTES);
        assert_eq!(device.answer(WRITE_STREAM, &[])[0], TOO_FEW_BYTES);
        assert!(read_answer(&[]).is_err());
    }

    #[test]
    fn an_empty_stream_is_one_chunk_each_way() {
        let device = device();
        assert_eq!(write_requests(&[], 100), vec![vec![FIRST | LAST]]);
        let _ = device.answer(WRITE_STREAM, &[FIRST | LAST]);
        assert!(device.held().is_empty());
        let answer = device.answer(READ_STREAM, &read_request(0));
        assert_eq!(answered(&answer).expect("ok"), [FIRST | LAST]);
    }

    #[test]
    fn a_write_that_starts_over_forgets_what_came_before() {
        let device = device();
        let _ = device.answer(WRITE_STREAM, &[FIRST, 1, 2]);
        let _ = device.answer(WRITE_STREAM, &[FIRST | LAST, 3]);
        assert_eq!(device.held(), [3]);
    }
}
