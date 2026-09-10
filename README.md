# xmip-core-transport-hart

HART transport: the field device protocol on the 4-20 mA loop — short and long frames with an XOR checksum, commands 0 and 1, a Stream written and read in chunks through device-specific commands, burst mode taken as it comes; a loopback line stands in for the modem. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.
