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
//! Two buses. [`Bus`] is the boundary; [`Loopback`] is an in-process bus that
//! every test and every box without a CAN interface can drive; `socketcan`,
//! behind the feature of that name, is the Linux kernel bus. A transport is
//! built on whichever bus the node has.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use transport::error::{Result, protocol_error};
#[cfg(feature = "socketcan")]
use transport::error::classify;
use transport::{Arrived, Directions, Transport};

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

/// Where frames go and come from.
pub trait Bus: Send + Sync {
    /// The bus's name, for the origin URI.
    fn name(&self) -> &str;
    /// The next frame, or `None` when nothing arrived within `timeout`.
    ///
    /// # Errors
    /// Where the bus could not be read.
    fn receive(&self, timeout: Duration) -> Result<Option<Frame>>;
    /// Put a frame on the bus.
    ///
    /// # Errors
    /// Where the bus refused it.
    fn transmit(&self, frame: &Frame) -> Result<()>;
}

/// An in-process bus: what is transmitted is received, in order.
#[derive(Clone, Default)]
pub struct Loopback {
    frames: Arc<Mutex<VecDeque<Frame>>>,
}

impl Loopback {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

impl Bus for Loopback {
    fn name(&self) -> &'static str {
        "loopback"
    }

    fn receive(&self, _timeout: Duration) -> Result<Option<Frame>> {
        Ok(self
            .frames
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .pop_front())
    }

    fn transmit(&self, frame: &Frame) -> Result<()> {
        self.frames
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .push_back(frame.clone());
        Ok(())
    }
}

/// The Linux kernel bus, `can0` and its kind.
#[cfg(feature = "socketcan")]
pub struct SocketCan {
    interface: String,
    socket: socketcan::CanSocket,
}

#[cfg(feature = "socketcan")]
impl SocketCan {
    /// Open `interface`.
    ///
    /// # Errors
    /// Where the interface does not exist or cannot be opened.
    pub fn open(interface: &str) -> Result<Self> {
        use socketcan::Socket;
        let socket = socketcan::CanSocket::open(interface)
            .map_err(|e| classify("opening the CAN interface", &std::io::Error::other(e)))?;
        Ok(Self {
            interface: interface.to_string(),
            socket,
        })
    }
}

#[cfg(feature = "socketcan")]
impl Bus for SocketCan {
    fn name(&self) -> &str {
        &self.interface
    }

    fn receive(&self, timeout: Duration) -> Result<Option<Frame>> {
        use socketcan::{EmbeddedFrame, Socket};
        self.socket
            .set_read_timeout(timeout)
            .map_err(|e| classify("setting the read timeout", &e))?;
        match self.socket.read_frame() {
            Ok(socketcan::CanFrame::Data(frame)) => Ok(Some(Frame {
                id: match frame.id() {
                    socketcan::Id::Standard(id) => u32::from(id.as_raw()),
                    socketcan::Id::Extended(id) => id.as_raw(),
                },
                extended: frame.is_extended(),
                data: frame.data().to_vec(),
            })),
            Ok(_) => Ok(None),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(None),
            Err(e) => Err(classify("reading the bus", &e)),
        }
    }

    fn transmit(&self, frame: &Frame) -> Result<()> {
        use socketcan::{EmbeddedFrame, Socket};
        let id = if frame.extended {
            socketcan::Id::Extended(
                socketcan::ExtendedId::new(frame.id)
                    .ok_or_else(|| protocol_error("an identifier wider than 29 bits"))?,
            )
        } else {
            socketcan::Id::Standard(
                socketcan::StandardId::new(u16::try_from(frame.id).unwrap_or(u16::MAX))
                    .ok_or_else(|| protocol_error("an identifier wider than 11 bits"))?,
            )
        };
        let can = socketcan::CanFrame::new(id, &frame.data)
            .ok_or_else(|| protocol_error("a frame the bus refused"))?;
        self.socket
            .write_frame(&can)
            .map_err(|e| classify("writing the bus", &e))
    }
}

pub struct CanTransport {
    bus: Arc<dyn Bus>,
    receive_timeout: Duration,
    id: u32,
    extended: bool,
}

impl CanTransport {
    /// A transport over `bus`, sending under identifier `id`.
    #[must_use]
    pub fn new(bus: Arc<dyn Bus>, id: u32) -> Self {
        Self {
            bus,
            receive_timeout: Duration::from_secs(1),
            id,
            extended: id > 0x7ff,
        }
    }

    #[must_use]
    pub const fn timing_out_after(mut self, timeout: Duration) -> Self {
        self.receive_timeout = timeout;
        self
    }

    /// The next frame on the bus as a Stream, or `None` when the bus is quiet.
    ///
    /// # Errors
    /// Where the bus could not be read.
    pub fn receive_one(&self) -> Result<Option<Arrived>> {
        Ok(self
            .bus
            .receive(self.receive_timeout)?
            .map(|frame| Arrived::new(frame.origin(self.bus.name()), frame.data)))
    }
}

impl Transport for CanTransport {
    fn name(&self) -> &'static str {
        "can-bus"
    }

    fn directions(&self) -> Directions {
        Directions::BOTH
    }

    /// Nothing on the bus is not an error: an empty vector.
    fn receive(&self) -> Result<Vec<Arrived>> {
        Ok(self.receive_one()?.into_iter().collect())
    }

    /// `target` may name an identifier, `0x181`, overriding the transport's.
    fn send(&self, target: &str, bytes: &[u8]) -> Result<()> {
        let id = match target.trim_start_matches("can://").rsplit('/').next() {
            Some(hex) if hex.starts_with("0x") => u32::from_str_radix(&hex[2..], 16)
                .map_err(|_| protocol_error(format!("{hex} is not a CAN identifier")))?,
            _ => self.id,
        };
        let extended = self.extended || id > 0x7ff;
        self.bus.transmit(&Frame::new(id, extended, bytes)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn the_loopback_bus_carries_frames_in_order_and_the_transport_reads_them() {
        let bus: Arc<dyn Bus> = Arc::new(Loopback::new());
        let transport = CanTransport::new(Arc::clone(&bus), 0x181);
        transport
            .send("can://loopback/0x181", &[0xaa])
            .expect("sending");
        transport
            .send("can://loopback/0x18fef100", &[0xbb, 0xcc])
            .expect("sending extended");
        let first = transport
            .receive_one()
            .expect("receiving")
            .expect("a frame");
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
        assert!(transport.send("can://loopback/0xzz", &[]).is_err());
    }

    #[test]
    fn a_bus_has_no_artefact_to_claim() {
        let transport = CanTransport::new(Arc::new(Loopback::new()), 0x181);
        assert!(transport.claims().is_none());
        assert_eq!(transport.name(), "can-bus");
    }
}
