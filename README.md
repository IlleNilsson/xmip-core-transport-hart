# xmip-core-transport-hart

HART transport: the field device protocol on the 4-20 mA loop — short and long frames with an XOR checksum, commands 0 and 1, a Stream written and read in chunks through device-specific commands, burst mode taken as it comes; a loopback line stands in for the modem. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls: scheme, authority, path and decoded query. Until 2026-09-28 it was read through the transport capability's `socket::target`, which split it on its first slash and left the query in the path.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Acknowledgement

Acceptance is at-most-once here. A receive takes what the line carries, a
burst-mode frame the device sends on its own, which nobody answers: it is off
the line as it is read, and nobody is left to tell how the receive cycle
ended. Each frame's data arrives whole.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
