//! The HART frame as it travels the line: a run of preambles, a delimiter
//! that says who sent it and how it is addressed, a short or long address,
//! a command, a byte count, the data and an XOR checksum over everything
//! after the preambles.

use std::fmt;

use transport::error::{Result, protocol_error};

/// The byte a frame opens with: at least two of them, five by default.
pub const PREAMBLE: u8 = 0xff;
/// How many preambles a frame is sent with.
pub const PREAMBLES: usize = 5;
/// The fewest preambles a frame is read with.
pub const MIN_PREAMBLES: usize = 2;
/// The most data one frame carries: the byte count is one byte.
pub const MAX_DATA: usize = 255;

/// The primary master's bit in the first address byte.
const PRIMARY_MASTER: u8 = 0x80;
/// The burst-mode bit in the first address byte.
const BURST: u8 = 0x40;
/// The long-frame bit in the delimiter.
const LONG_FRAME: u8 = 0x80;

/// Who sent the frame, and whether anyone asked for it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A master's request: start of text.
    Stx,
    /// A field device's answer to one.
    Ack,
    /// A field device in burst mode, answering nobody.
    Back,
}

impl Kind {
    const fn bits(self) -> u8 {
        match self {
            Self::Stx => 0x02,
            Self::Ack => 0x06,
            Self::Back => 0x01,
        }
    }

    fn from_bits(bits: u8) -> Result<Self> {
        match bits & 0x07 {
            0x02 => Ok(Self::Stx),
            0x06 => Ok(Self::Ack),
            0x01 => Ok(Self::Back),
            other => Err(protocol_error(format!(
                "a delimiter HART does not use: {other:#04x}"
            ))),
        }
    }
}

/// Where a frame is going, or where it came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Address {
    /// A polling address, 0 to 63, in the short frame.
    Short(u8),
    /// The unique address in the long frame: the manufacturer, the device
    /// type and the device identifier, which every HART 5 and later device
    /// answers to whatever its polling address.
    Long {
        manufacturer: u8,
        device_type: u8,
        device_id: u32,
    },
}

impl Address {
    /// `7` for a polling address, `26-e5-0a1b2c` for a unique one:
    /// manufacturer, device type and device identifier in hex.
    ///
    /// # Errors
    /// A polling address over 63, or hex that does not read.
    pub fn parse(text: &str) -> Result<Self> {
        let refused = || protocol_error(format!("{text:?} is not a HART address"));
        let mut parts = text.split('-');
        let (Some(first), second, third, None) =
            (parts.next(), parts.next(), parts.next(), parts.next())
        else {
            return Err(refused());
        };
        match (second, third) {
            (None, None) => {
                let polling: u8 = first.parse().map_err(|_| refused())?;
                if polling > 63 {
                    return Err(protocol_error("a polling address over 63"));
                }
                Ok(Self::Short(polling))
            }
            (Some(device_type), Some(device_id)) => Ok(Self::Long {
                manufacturer: u8::from_str_radix(first, 16).map_err(|_| refused())?,
                device_type: u8::from_str_radix(device_type, 16).map_err(|_| refused())?,
                device_id: u32::from_str_radix(device_id, 16).map_err(|_| refused())?,
            }),
            _ => Err(refused()),
        }
    }

    const fn is_long(&self) -> bool {
        matches!(self, Self::Long { .. })
    }

    fn encode(&self, kind: Kind, out: &mut Vec<u8>) {
        let flags = PRIMARY_MASTER | if kind == Kind::Back { BURST } else { 0 };
        match self {
            Self::Short(polling) => out.push(flags | (polling & 0x3f)),
            Self::Long {
                manufacturer,
                device_type,
                device_id,
            } => {
                out.push(flags | (manufacturer & 0x3f));
                out.push(*device_type);
                out.extend_from_slice(&device_id.to_be_bytes()[1..]);
            }
        }
    }

    fn decode(bytes: &[u8], long: bool) -> Result<(Self, usize)> {
        let short = || protocol_error("a frame cut off inside its address");
        let first = *bytes.first().ok_or_else(short)?;
        if !long {
            return Ok((Self::Short(first & 0x3f), 1));
        }
        let rest = bytes.get(1..5).ok_or_else(short)?;
        Ok((
            Self::Long {
                manufacturer: first & 0x3f,
                device_type: rest[0],
                device_id: u32::from_be_bytes([0, rest[1], rest[2], rest[3]]),
            },
            5,
        ))
    }
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Short(polling) => write!(f, "{polling}"),
            Self::Long {
                manufacturer,
                device_type,
                device_id,
            } => write!(f, "{manufacturer:02x}-{device_type:02x}-{device_id:06x}"),
        }
    }
}

/// One frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub kind: Kind,
    pub address: Address,
    pub command: u8,
    pub data: Vec<u8>,
}

impl Frame {
    /// A frame, refusing more data than the byte count can say.
    ///
    /// # Errors
    /// Data over [`MAX_DATA`].
    pub fn new(kind: Kind, address: Address, command: u8, data: &[u8]) -> Result<Self> {
        if data.len() > MAX_DATA {
            return Err(protocol_error("more data than one HART frame carries"));
        }
        Ok(Self {
            kind,
            address,
            command,
            data: data.to_vec(),
        })
    }

    /// The frame as the line carries it, preambles first.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut out = vec![PREAMBLE; PREAMBLES];
        let long = if self.address.is_long() {
            LONG_FRAME
        } else {
            0
        };
        out.push(self.kind.bits() | long);
        self.address.encode(self.kind, &mut out);
        out.push(self.command);
        out.push(u8::try_from(self.data.len()).unwrap_or(u8::MAX));
        out.extend_from_slice(&self.data);
        let check = checksum(&out[PREAMBLES..]);
        out.push(check);
        out
    }

    /// The frame at the start of `bytes`, and how many bytes it took.
    ///
    /// # Errors
    /// Fewer than two preambles, a delimiter HART does not use, a frame cut
    /// off before its checksum, or a checksum that does not check.
    pub fn read(bytes: &[u8]) -> Result<(Self, usize)> {
        let preambles = bytes.iter().take_while(|b| **b == PREAMBLE).count();
        if preambles < MIN_PREAMBLES {
            return Err(protocol_error("fewer than two preambles"));
        }
        let cut = || protocol_error("a frame cut off before its checksum");
        let body = &bytes[preambles..];
        let delimiter = *body.first().ok_or_else(cut)?;
        let kind = Kind::from_bits(delimiter)?;
        let (address, taken) = Address::decode(&body[1..], delimiter & LONG_FRAME != 0)?;
        let at = 1 + taken;
        let command = *body.get(at).ok_or_else(cut)?;
        let count = usize::from(*body.get(at + 1).ok_or_else(cut)?);
        let data = body.get(at + 2..at + 2 + count).ok_or_else(cut)?;
        let end = at + 2 + count;
        let check = *body.get(end).ok_or_else(cut)?;
        if checksum(&body[..end]) != check {
            return Err(protocol_error("a checksum that does not check"));
        }
        Ok((
            Self {
                kind,
                address,
                command,
                data: data.to_vec(),
            },
            preambles + end + 1,
        ))
    }

    /// Exactly one frame, nothing after it.
    ///
    /// # Errors
    /// As [`Self::read`], or bytes after the checksum.
    pub fn decode(bytes: &[u8]) -> Result<Self> {
        let (frame, taken) = Self::read(bytes)?;
        if taken != bytes.len() {
            return Err(protocol_error("bytes after the checksum"));
        }
        Ok(frame)
    }
}

/// The XOR of every byte from the delimiter to the last data byte.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, byte| acc ^ byte)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_and_long_frames_round_trip_with_their_checksum() {
        let short = Frame::new(Kind::Stx, Address::Short(0), 0, &[]).expect("frame");
        let bytes = short.encode();
        assert_eq!(
            bytes,
            [0xff, 0xff, 0xff, 0xff, 0xff, 0x02, 0x80, 0x00, 0x00, 0x82]
        );
        assert_eq!(Frame::decode(&bytes).expect("decode"), short);
        let long = Frame::new(
            Kind::Ack,
            Address::Long {
                manufacturer: 0x26,
                device_type: 0xe5,
                device_id: 0x0a_1b2c,
            },
            1,
            &[0, 0, 0x20, 0x41, 0x20, 0, 0],
        )
        .expect("frame");
        let bytes = long.encode();
        assert_eq!(bytes[5], 0x86, "long frame ACK");
        assert_eq!(&bytes[6..11], &[0xa6, 0xe5, 0x0a, 0x1b, 0x2c]);
        assert_eq!(Frame::decode(&bytes).expect("decode"), long);
        let burst = Frame::new(Kind::Back, Address::Short(3), 1, &[0; 7]).expect("frame");
        assert_eq!(burst.encode()[6], 0xc3, "burst bit set");
        let mut two = burst.encode();
        two.extend(long.encode());
        let (first, taken) = Frame::read(&two).expect("first");
        assert_eq!(first, burst);
        assert_eq!(Frame::read(&two[taken..]).expect("second").0, long);
    }

    #[test]
    fn what_is_not_a_frame_is_refused() {
        let frame = Frame::new(Kind::Stx, Address::Short(1), 1, &[]).expect("frame");
        let bytes = frame.encode();
        assert!(Frame::decode(&bytes[4..]).is_err(), "one preamble");
        let mut bad = bytes.clone();
        bad[5] = 0x03;
        assert!(Frame::decode(&bad).is_err(), "delimiter");
        let mut bad = bytes.clone();
        bad[9] ^= 1;
        assert!(Frame::decode(&bad).is_err(), "checksum");
        assert!(Frame::decode(&bytes[..8]).is_err(), "cut off");
        let mut trailing = bytes;
        trailing.push(0);
        assert!(Frame::decode(&trailing).is_err(), "bytes after");
        assert!(Frame::new(Kind::Stx, Address::Short(1), 1, &[0; 256]).is_err());
    }

    #[test]
    fn an_address_reads_from_text_and_writes_back_to_it() {
        assert_eq!(Address::parse("7").expect("short"), Address::Short(7));
        assert_eq!(Address::Short(7).to_string(), "7");
        let long = Address::parse("26-e5-0a1b2c").expect("long");
        assert_eq!(
            long,
            Address::Long {
                manufacturer: 0x26,
                device_type: 0xe5,
                device_id: 0x0a_1b2c
            }
        );
        assert_eq!(long.to_string(), "26-e5-0a1b2c");
        assert!(Address::parse("64").is_err());
        assert!(Address::parse("zz").is_err());
        assert!(Address::parse("26-e5").is_err());
        assert!(Address::parse("26-e5-0a-1b").is_err());
    }
}
