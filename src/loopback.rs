//! The two nodes every loopback over CAN stands up (ADR-0051): a near node
//! and a far node on one fresh simulated bus, each hearing what the other
//! transmits.
//!
//! One pair for every technology riding on CAN. ISO-TP's tester and ECU,
//! which UDS and OBD-II stand up for their rounds, and J1939's sender and
//! receiver were each this pair until 2026-09-25; what the two nodes say to
//! each other — ISO 15765-2's flow control, J1939-21's TP.CM and TP.DT — is
//! the technology's, and the bus they say it on is this crate's.

use std::sync::Arc;

use sdk::broadcast::Medium;

use crate::Bus;

/// One loopback session: the near node and the far node on one simulated
/// bus. A session is two nodes because the in-process bus, like a real one,
/// does not hand a node its own frames.
#[derive(Clone)]
pub struct Session {
    /// The node the near end transmits from.
    pub near: Arc<dyn Bus>,
    /// The node the far end listens on.
    pub far: Arc<dyn Bus>,
}

impl Session {
    /// A fresh bus with the two nodes on it.
    #[must_use]
    pub fn fresh() -> Self {
        let medium = Medium::new("loopback");
        Self {
            near: Arc::new(medium.node()),
            far: Arc::new(medium.node()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frame;
    use std::time::Duration;

    #[test]
    fn each_node_hears_the_other_and_not_itself() {
        let session = Session::fresh();
        let frame = Frame::new(0x181, false, b"hi").expect("frame");

        session.near.transmit(&frame).expect("transmit");

        let quiet = Duration::from_millis(10);
        assert_eq!(session.far.receive(quiet).expect("read"), Some(frame));
        assert_eq!(session.near.receive(quiet).expect("read"), None);
        assert_eq!(session.far.name(), "loopback");
    }
}
