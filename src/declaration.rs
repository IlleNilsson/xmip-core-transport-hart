//! What a Location of this technology is configured with: its declaration,
//! which `Configured` reads, built with the crate's `declaration` feature
//! alone and the code behind the default (ADR-0064, amendments 2026-10-22
//! and 2026-11-26), for the library the surfaces load to carry (open
//! problem 48). The wait a Location takes unless told otherwise is declared
//! here, and the code takes it from here.

use std::time::Duration;

use xcore::settings::{Applies, Kind, Presence, Setting, Settings};

/// How long a master waits for a device's answer unless a Location says.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

/// Every setting a Location takes beyond its address, the line the master
/// speaks on: a HART modem's serial port, `/dev/ttyUSB0` or `COM3`, or the
/// SDK's in-process multi-drop bus, `loopback` (open problem 24).
pub const SETTINGS: &Settings = &Settings {
    technology: env!("CARGO_PKG_NAME"),
    settings: &[
        Setting {
            name: "device",
            kind: Kind::Address,
            presence: Presence::Required,
            meaning: "The device a Stream is written to unless the target names another: \
                      a polling address, 7, or a unique one, 26-e5-0a1b2c.",
            applies: Applies::Send,
        },
        transport::configured::timeout(
            Applies::Both,
            Some(DEFAULT_TIMEOUT),
            "How long a device's answer, or a quiet line, is waited on.",
        ),
    ],
};

/// The schemes a Location's address is entered with to choose this technology.
pub const SCHEMES: &[&str] = &["hart"];
