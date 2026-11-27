//! This technology's declaration: the transport guide's "The declaration",
//! `module/core/capability/transport/doc/adding-a-transport.md`. How long a
//! receive waits for a frame unless told otherwise is declared here, and the
//! code takes it from here.

use std::time::Duration;

use xcore::settings::{Applies, Kind, Presence, Setting, Settings};

/// How long a receive waits for a frame when a Location says nothing else.
pub(crate) const RECEIVE_TIMEOUT: Duration = Duration::from_secs(1);

/// Every setting a Location takes beyond its address, the bus, as
/// `open_bus` opens it.
pub const SETTINGS: &Settings = &Settings {
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
        transport::configured::timeout(
            Applies::Receive,
            Some(RECEIVE_TIMEOUT),
            "How long a receive waits for a frame before it finds none.",
        ),
    ],
};

/// This technology's schemes, as [`transport::Configured::SCHEMES`] defines
/// them.
pub const SCHEMES: &[&str] = &["can"];
