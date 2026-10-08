#![forbid(unsafe_code)]

//! Streams that arrive as CAN frames. One classical CAN frame is one Stream:
//! up to eight data bytes, with the identifier beside them.
//!
//! CAN is the vehicle and the machine: every `ECU`, every drive, every safety
//! controller on a two-wire bus. The frame is tiny and the identifier is most
//! of the meaning — `CANopen` puts a node and a function in it, J1939 a parameter
//! group and addresses — so this transport carries the identifier in the
//! origin URI, `can://can0/0x181?extended=false`, and the protocols above it
//! read it back from there. Which frames belong together is theirs.
//!
//! Two buses. [`Bus`] is the boundary. A node on the SDK's broadcast medium
//! is one — every test and every box without a CAN interface drives it, and
//! every other node on it hears a frame as on a real bus; `socketcan`, behind
//! the feature of that name, is the Linux kernel bus. A transport is built on
//! whichever bus the node has; both live in `bus.rs`.
//!
//! **Acceptance is at-most-once here** ([`AT_MOST_ONCE`]): a CAN frame is
//! acknowledged in its ACK slot by every controller that hears it, before
//! any receiver reads it, and has no reply above that. Each frame arrives
//! whole. The protocols above that have one — ISO-TP's flow control, a
//! `CANopen` SDO response — answer it themselves.

mod bus;
pub mod loopback;

use std::sync::Arc;
use std::time::Duration;

#[cfg(all(feature = "socketcan", target_os = "linux"))]
pub use bus::SocketCan;
pub use bus::{Bus, open_bus};
use codec::hex::prefixed_number;
use context::property::CAN_IDENTIFIER;
use net::Target;
use transport::error::{Result, protocol_error};
use transport::held::Held;
// The trait shares its name with the bus below; it is reached by path.
use transport::loopback::{FarEnd, LOOPBACK_TIMEOUT};
use transport::{Acknowledgement, Arrived, Configured, Directions, Taken, Transport};
use xcore::settings::{Applies, Fixed, Kind, Presence, Read, Setting, Settings};

/// Why a CAN frame cannot be acknowledged after the receive cycle.
pub const AT_MOST_ONCE: &str = "a CAN frame is acknowledged on the wire by every controller \
                                that hears it, before any receiver reads it; there is no \
                                reply to defer";

/// How long a receive waits for a frame when a Location says nothing else.
const RECEIVE_TIMEOUT: Duration = Duration::from_secs(1);

/// One classical CAN frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub extended: bool,
    pub data: Vec<u8>,
}

impl Frame {
    /// A frame, refusing what CAN cannot carry: more than eight bytes, an
    /// 11-bit identifier over `0x7ff`, a 29-bit one over `0x1fff_ffff`.
    ///
    /// # Errors
    /// Outside those bounds.
    pub fn new(id: u32, extended: bool, data: &[u8]) -> Result<Self> {
        if data.len() > 8 {
            return Err(protocol_error(
                "a classical CAN frame carries at most eight bytes",
            ));
        }
        let limit = if extended { 0x1fff_ffff } else { 0x7ff };
        if id > limit {
            return Err(protocol_error("an identifier wider than the frame"));
        }
        Ok(Self {
            id,
            extended,
            data: data.to_vec(),
        })
    }

    /// `can://<bus>/0x<id>?extended=<bool>`.
    #[must_use]
    pub fn origin(&self, bus: &str) -> String {
        format!("can://{bus}/{:#x}?extended={}", self.id, self.extended)
    }
}

#[derive(Clone)]
pub struct CanTransport {
    bus: Arc<dyn Bus>,
    receive_timeout: Duration,
    id: u32,
    extended: bool,
    /// On a loopback, the far end's node on the same medium.
    far: Option<Arc<dyn Bus>>,
}

impl CanTransport {
    /// A transport over `bus`, sending under identifier `id`.
    #[must_use]
    pub fn new(bus: Arc<dyn Bus>, id: u32) -> Self {
        Self {
            bus,
            receive_timeout: RECEIVE_TIMEOUT,
            id,
            extended: id > 0x7ff,
            far: None,
        }
    }

    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.receive_timeout = timeout;
        self
    }

    /// The next frame on the bus as a Stream, whole, or `None` when the bus
    /// is quiet. Acceptance is at-most-once ([`AT_MOST_ONCE`]).
    ///
    /// # Errors
    /// Where the bus could not be read.
    pub fn receive_one(&self) -> Result<Option<Arrived>> {
        Ok(self.receive_frame()?.map(|frame| {
            Arrived::whole(
                frame.origin(self.bus.name()),
                frame.data,
                Acknowledgement::at_most_once(AT_MOST_ONCE),
            )
            .observing(CAN_IDENTIFIER, format!("{:#x}", frame.id))
        }))
    }

    /// The next frame on the bus, or `None` when the bus is quiet.
    ///
    /// # Errors
    /// Where the bus could not be read.
    pub fn receive_frame(&self) -> Result<Option<Frame>> {
        self.bus.receive(self.receive_timeout)
    }
}

impl Transport for CanTransport {
    fn name(&self) -> &'static str {
        "can-bus"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    fn arrivals(&self) -> transport::Arrivals {
        transport::Arrivals::Ordered("one line or bus, answered in the order it speaks")
    }

    /// Nothing on the bus is not an error: an empty vector. Acceptance is
    /// at-most-once here: a frame has no reply to defer ([`AT_MOST_ONCE`]).
    fn receive(&self) -> Result<Vec<Arrived>> {
        Ok(self.receive_one()?.into_iter().collect())
    }

    /// `target` may name an identifier, `0x181`, overriding the transport's.
    fn send(&self, target: &str, bytes: &[u8]) -> Result<()> {
        let named = Target::under(&["can"], target).map_or(target, |named| named.path());
        let id = match named.rsplit('/').next() {
            Some(hex) if hex.starts_with("0x") => prefixed_number(hex)
                .map_err(|_| protocol_error(format!("{hex} is not a CAN identifier")))?,
            _ => self.id,
        };
        let extended = self.extended || id > 0x7ff;
        self.bus.transmit(&Frame::new(id, extended, bytes)?)
    }
}

impl Configured for CanTransport {
    /// The address is the bus, as [`open_bus`] opens it.
    const SETTINGS: &'static Settings = &Settings {
        technology: env!("CARGO_PKG_NAME"),
        settings: &[
            Setting {
                name: "id",
                kind: Kind::Integer {
                    minimum: 0,
                    maximum: 0x1fff_ffff,
                },
                presence: Presence::Required,
                meaning: "The identifier a frame is sent under where the target names none; \
                          one over 0x7ff is extended.",
                applies: Applies::Send,
            },
            Setting {
                name: "timeout",
                kind: Kind::Duration,
                presence: Presence::Default(Fixed::Duration(RECEIVE_TIMEOUT)),
                meaning: "How long a receive waits for a frame before it finds none.",
                applies: Applies::Receive,
            },
        ],
    };

    fn configured(address: &str, settings: &Read) -> Result<Self> {
        // A Receive Location reads no identifier: it hears every frame.
        let id = settings.optional_integer("id").unwrap_or_default();
        let id = u32::try_from(id).map_err(|_| protocol_error("an identifier over 29 bits"))?;
        let transport = Self::new(open_bus(address)?, id);
        Ok(match settings.optional_duration("timeout") {
            Some(timeout) => transport.timing_out_after(timeout),
            None => transport,
        })
    }
}

impl CanTransport {
    /// Both ends on one simulated bus: this node sending under identifier
    /// `0x181`, a second node on the same medium its far end, the loopback
    /// timeout standing where a kernel bus would wait for quiet.
    #[must_use]
    pub fn loopback() -> Self {
        let session = loopback::Session::fresh();
        let mut near = Self::new(session.near, 0x181).timing_out_after(LOOPBACK_TIMEOUT);
        near.far = Some(session.far);
        near
    }

    /// `can://<bus>/0x<id>`: where the near end sends.
    fn target(&self) -> String {
        format!("can://{}/{:#x}", self.bus.name(), self.id)
    }
}

/// A Stream longer than one frame travels as frames of at most eight bytes
/// under one identifier, until the bus is quiet. An empty Stream is one
/// frame with no data: CAN carries it, and the far end sees it arrive.
impl transport::loopback::Loopback for CanTransport {
    fn arrival_identity(&self) -> transport::ArrivalIdentity {
        transport::ArrivalIdentity::Named(&[context::property::CAN_IDENTIFIER])
    }

    /// The bus the frames went on. Nothing waits: the round is in order, and
    /// the far end reads the bus until it is quiet.
    ///
    /// The first frame is waited for; the rest are already there. The
    /// loopback's bus is simulated, and a simulated node hears a frame when
    /// it is transmitted, so quiet after the first frame is quiet at once —
    /// waiting the timeout for it cost every round the whole timeout.
    fn far_end(&self) -> Result<Box<dyn FarEnd>> {
        let far = self
            .far
            .clone()
            .ok_or_else(|| protocol_error("a bus with no far end: not a loopback"))?;
        let transport = Self {
            bus: far,
            far: None,
            ..self.clone()
        };
        let name = transport.bus.name().to_string();
        Ok(Box::new(Held::new(self.target(), move || {
            let first = transport
                .receive_frame()?
                .ok_or_else(|| protocol_error("nothing came over the bus"))?;
            let rest = transport.timing_out_after(Duration::ZERO);
            let origin = first.origin(&name);
            let identifier = format!("{:#x}", first.id);
            let mut bytes = first.data;
            while let Some(frame) = rest.receive_frame()? {
                bytes.extend_from_slice(&frame.data);
            }
            Ok(Taken::new(origin, bytes).observing(CAN_IDENTIFIER, identifier))
        })))
    }

    fn send_to(&self, address: &str, payload: &[u8]) -> Result<()> {
        if payload.is_empty() {
            return self.send(address, payload);
        }
        for frame in payload.chunks(8) {
            self.send(address, frame)?;
        }
        Ok(())
    }

    /// In order on one thread: a bus does not listen, so the frames go on
    /// first and the read-back takes them off.
    fn exchanges_in_order(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdk::broadcast::Medium;
    use transport::loopback::Loopback as _;
    use transport::payload::edge_payloads;
    use xcore::settings::Given;

    /// On the simulated bus: with `socketcan` on Linux the address opens a
    /// kernel socket, which a unit test does not.
    #[test]
    #[cfg(not(all(feature = "socketcan", target_os = "linux")))]
    fn can_bus_declares_its_settings_and_reads_through_them() {
        assert_eq!(CanTransport::SETTINGS.problems(), Vec::<String>::new());
        let given = [("id".to_string(), Given::Integer(0x18fe_f100))];
        let sending = CanTransport::open("vcan0", Applies::Send, &given).expect("send");
        assert_eq!(sending.id, 0x18fe_f100);
        assert!(sending.extended);
        let given = [("timeout".to_string(), Given::Text("50ms".to_string()))];
        let receiving = CanTransport::open("vcan0", Applies::Receive, &given).expect("receive");
        assert_eq!(receiving.receive_timeout, Duration::from_millis(50));
        let Err(refused) = CanTransport::open("vcan0", Applies::Send, &[]) else {
            panic!("id is required on a Send Location");
        };
        assert!(refused.message.contains("\"id\""), "{}", refused.message);
    }

    #[test]
    fn a_loopback_round_carries_a_stream_as_frames() {
        let loopback = CanTransport::loopback();
        let arrived = loopback.round(b"seventeen bytes!!").expect("three frames");
        assert_eq!(arrived.bytes, b"seventeen bytes!!");
        assert_eq!(arrived.origin_uri, "can://loopback/0x181?extended=false");
        let arrived = loopback.round(b"").expect("one empty frame");
        assert!(arrived.bytes.is_empty());
        assert_eq!(arrived.origin_uri, "can://loopback/0x181?extended=false");
        assert!(loopback.ceiling().is_none());
        assert!(loopback.refuses(b"seventeen bytes!!").is_none());
    }

    #[test]
    fn the_loopback_returns_the_edges_whole() {
        let loopback = CanTransport::loopback();
        for (name, bytes) in edge_payloads() {
            let arrived = loopback
                .round(&bytes)
                .unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(arrived.bytes, bytes, "{name}");
        }
    }

    #[test]
    fn a_frame_is_at_most_eight_bytes_under_a_fitting_identifier() {
        assert!(Frame::new(0x181, false, &[1, 2, 3]).is_ok());
        assert!(Frame::new(0x181, false, &[0; 9]).is_err());
        assert!(Frame::new(0x800, false, &[]).is_err());
        assert!(Frame::new(0x18fe_f100, true, &[0; 8]).is_ok());
        assert_eq!(
            Frame::new(0x181, false, &[]).expect("frame").origin("can0"),
            "can://can0/0x181?extended=false"
        );
    }

    #[test]
    fn another_node_reads_the_frames_in_order_and_the_sender_does_not() {
        let session = loopback::Session::fresh();
        let quiet = Duration::from_millis(10);
        let sender = CanTransport::new(session.near, 0x181).timing_out_after(quiet);
        let transport = CanTransport::new(session.far, 0x181).timing_out_after(quiet);
        sender
            .send("can://loopback/0x181", &[0xaa])
            .expect("sending");
        sender
            .send("can://loopback/0x18fef100", &[0xbb, 0xcc])
            .expect("sending extended");
        let first = transport
            .receive_one()
            .expect("receiving")
            .expect("a frame");
        assert!(!first.defers(), "a CAN frame is at-most-once");
        let first = first.taken().expect("taken");
        assert_eq!(first.bytes, [0xaa]);
        assert_eq!(first.origin_uri, "can://loopback/0x181?extended=false");
        let second = transport
            .receive_one()
            .expect("receiving")
            .expect("a frame");
        assert_eq!(second.origin_uri, "can://loopback/0x18fef100?extended=true");
        assert!(
            transport.receive().expect("quiet").is_empty(),
            "nothing is not an error"
        );
        assert!(sender.receive().expect("quiet").is_empty(), "not its own");
        assert!(sender.send("can://loopback/0xzz", &[]).is_err());
    }

    #[test]
    fn a_frame_no_other_node_hears_is_not_acknowledged() {
        let alone = CanTransport::new(Arc::new(Medium::<Frame>::new("can0").node()), 0x181);
        let refused = alone.send("can://can0/0x181", &[1]).expect_err("nobody");
        assert!(refused.retryable, "{refused}");
        assert!(alone.far_end().is_err(), "not a loopback");
    }

    #[test]
    fn a_bus_has_no_artefact_to_claim() {
        let transport = CanTransport::new(Arc::new(Medium::<Frame>::new("can0").node()), 0x181);
        assert!(transport.claims().is_none());
        assert_eq!(transport.name(), "can-bus");
    }
}
