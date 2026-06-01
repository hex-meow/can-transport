//! Async, cross-platform CAN / CAN-FD transport abstraction.
//!
//! This crate defines two small traits, [`CanBus`] and [`CanRx`], that
//! represent a shared CAN bus and a filtered receive subscription. The
//! bus implementation is responsible for the fan-out: every subscriber
//! gets its own queue of frames matching its [`CanFilter`].
//!
//! Backends:
//! - `socketcan` (feature) — Linux SocketCAN, CAN 2.0 and CAN-FD.
//!
//! See the `examples/` directory for end-to-end usage with SocketCAN.

pub mod bus;
pub mod error;
pub mod filter;
pub mod frame;

pub use bus::{CanBus, CanCapabilities, CanRx};
pub use error::CanIoError;
pub use filter::CanFilter;
pub use frame::{CanFrame, CanId, FrameKind, MAX_DLEN};

#[cfg(feature = "socketcan")]
pub mod socketcan;
