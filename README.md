# xmip-core-transport-hart

HART transport: the field device protocol on the 4-20 mA loop — short and long frames with an XOR checksum, commands 0 and 1, a Stream written and read in chunks through device-specific commands, burst mode taken as it comes; over a modem's serial port, or field devices on the SDK's in-process bus. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A Location's address is its line: the serial port a HART modem is on, `/dev/ttyUSB0` or `COM3`, at 1200 baud, each frame read by the length it says (`Frame::measure`), as [xmip-core-transport-serial](https://github.com/IlleNilsson/xmip-core-transport-serial) reads M-Bus; or `loopback`, the SDK's in-process multi-drop bus. Every Location naming one port on a node goes through that port, as every serial line does (`Configured::shared_by`). Until 2026-11-25 every address but `loopback` was refused.

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls: scheme, authority, path and decoded query. Until 2026-09-28 it was read through the transport capability's `socket::target`, which split it on its first slash and left the query in the path.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Acknowledgement

Acceptance is at-most-once here. A receive takes what the line carries, a
burst-mode frame the device sends on its own, which nobody answers: it is off
the line as it is read, and nobody is left to tell how the receive cycle
ended. Each frame's data arrives whole.

## The declaration

What a Location of this technology is configured with is two `const`s in its `declaration` module, `src/declaration.rs`, beside the code, which `Configured` reads (ADR-0064, amendment 2026-11-26): `declaration::SETTINGS`, `device` and `timeout`, and `SCHEMES`, `hart`; `timeout`'s default, `declaration::DEFAULT_TIMEOUT`, is declared there too, the code takes it from there, and the crate root names it as before. Built with its `declaration` feature alone (`--no-default-features --features declaration`), the crate is those and nothing more: its code and every crate the code is built on — the serial technology and the SDK's multi-drop bus among them — are behind the default feature. So the library the surfaces load carries it without the code, through the gathering crate xmip-core-declaration (open problem 48), and the language server and the desktop suggest and check a HART Location's settings though no node in their process loaded the technology. `test/Declaration.Test.ps1` in the estate holds that the declaration builds nothing the transport capability does not.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
