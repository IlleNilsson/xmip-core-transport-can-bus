# xmip-core-transport-can-bus

CAN bus transport: one classical CAN frame is one Stream, the identifier beside it; SocketCAN on Linux, a loopback bus everywhere for the protocols above it. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls. Until 2026-09-28 this technology stripped its scheme by hand.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Toolchain

`rust-toolchain.toml` pins the toolchain for the whole estate. Do not change it
here.

## Verification

The included workflow is manual-only and calls the versioned shared workflow at
`IlleNilsson/.github@v1`.

## The bus

`Loopback` is the in-process bus every test and every box without a CAN
interface drives; canopen and j1939 build on it. The `socketcan` feature, off
by default, adds the Linux kernel bus (`can0` and its kind). Windows and macOS
have no kernel CAN socket; an adapter's vendor stack is a further bus behind
the same `Bus` trait.

## The loopback session

`loopback::Session` is the two nodes every loopback over CAN stands up: a
near node and a far node on one fresh simulated bus, each hearing what the
other transmits. This crate's own loopback, ISO-TP's tester and ECU (and so
UDS's and OBD-II's) and J1939's sender and receiver all stand up this one
pair; ISO-TP and J1939 each carried a type of their own for it until
2026-09-25. What the two nodes say to each other is the technology's.
