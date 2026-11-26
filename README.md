# xmip-core-transport-can-bus

CAN bus transport: one classical CAN frame is one Stream, the identifier beside it; SocketCAN on Linux, a loopback bus everywhere for the protocols above it. A technology of [xmip-core-transport](https://github.com/IlleNilsson/xmip-core-transport).

A send target is read by `net::Target` in [xmip-core-library-net](https://github.com/IlleNilsson/xmip-core-library-net), the one reading of a URI every technology calls. Until 2026-09-28 this technology stripped its scheme by hand.

A `0x` number in a target is read by `codec::hex::prefixed_number` in [xmip-core-library-codec](https://github.com/IlleNilsson/xmip-core-library-codec), which refuses a sign; until 2026-09-28 it was read with `from_str_radix`, which took `0x+7e8`.

## Acknowledgement

Acceptance is at-most-once here. A CAN frame is acknowledged in its ACK slot
by every controller that hears it, before any receiver reads it, and has no
reply above that: nobody is left to tell how the receive cycle ended. Each
frame arrives whole. The protocols above that have a reply, ISO-TP's flow
control or a CANopen SDO response, answer it themselves.

## The declaration

What a Location of this technology is configured with is two `const`s in its `declaration` module, `src/declaration.rs`, beside the code, which `Configured` reads (ADR-0064, amendment 2026-11-26): `declaration::SETTINGS`, `id` and `timeout`, and `SCHEMES`, `can`; `timeout`'s default, the second a receive waits for a frame, is declared there too, and the code takes it from there. Built with its `declaration` feature alone (`--no-default-features --features declaration`), the crate is those and nothing more: its code and every crate the code is built on — the SDK's simulated bus among them — are behind the default feature, and `socketcan`, which is code, turns the default on with it. So the library the surfaces load can carry it without the code, for the language server and the desktop to suggest and check a CAN Location's settings though no node in their process loaded the technology, once the gathering crate of open problem 48 links it. `test/Declaration.Test.ps1` in the estate holds that the declaration builds nothing the transport capability does not.

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
