//! The bus a transport is built on, and the two there are.
//!
//! [`Bus`] is the boundary. A node on the SDK's broadcast medium is one —
//! every test and every box without a CAN interface drives it, and every
//! other node on it hears a frame as on a real bus; `SocketCan`, behind the
//! `socketcan` feature, is the Linux kernel bus. [`open_bus`] is the one way
//! from a Location's address to either.

use std::sync::Arc;
use std::time::Duration;

use sdk::broadcast::Node;

use transport::error::Result;
#[cfg(all(feature = "socketcan", target_os = "linux"))]
use transport::error::{classify, protocol_error};

use crate::Frame;

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

/// A node on the SDK's broadcast medium is a bus: it hears what the other
/// nodes transmit, never its own frames, and a frame nobody else is there to
/// hear is not acknowledged. It replaced this crate's own in-process bus on
/// 2026-09-24, a single queue a node read its own frames back from.
impl Bus for Node<Frame> {
    fn name(&self) -> &str {
        Node::name(self)
    }

    fn receive(&self, timeout: Duration) -> Result<Option<Frame>> {
        Node::receive(self, timeout)
    }

    fn transmit(&self, frame: &Frame) -> Result<()> {
        Node::transmit(self, frame)
    }
}

/// The Linux kernel bus, `can0` and its kind.
#[cfg(all(feature = "socketcan", target_os = "linux"))]
pub struct SocketCan {
    interface: String,
    socket: socketcan::CanSocket,
}

#[cfg(all(feature = "socketcan", target_os = "linux"))]
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

#[cfg(all(feature = "socketcan", target_os = "linux"))]
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

/// The bus a Location's address names: the kernel interface, `can0`, where
/// the build has `socketcan` on Linux; a node on a simulated bus of that name
/// otherwise. The one way from an address to a bus, for every technology
/// riding on CAN.
///
/// # Errors
/// An interface the kernel will not open.
pub fn open_bus(address: &str) -> Result<Arc<dyn Bus>> {
    #[cfg(all(feature = "socketcan", target_os = "linux"))]
    let bus: Arc<dyn Bus> = Arc::new(SocketCan::open(address)?);
    #[cfg(not(all(feature = "socketcan", target_os = "linux")))]
    let bus: Arc<dyn Bus> = Arc::new(sdk::broadcast::Medium::<Frame>::new(address).node());
    Ok(bus)
}
