//! The `CanBus` and `CanRx` traits.
//!
//! A bus is a long-lived, shared object. Anyone who wants to receive
//! traffic calls [`CanBus::subscribe`] with a filter and gets back a
//! [`CanRx`] that delivers only matching frames.
//!
//! Multiple subscribers are expected; the bus is responsible for
//! fan-out. Slow subscribers must not block other subscribers — they
//! get a `CanIoError::Lagged` on their next `recv`.

use async_trait::async_trait;

use crate::error::CanIoError;
use crate::filter::CanFilter;
use crate::frame::CanFrame;

/// Static capabilities reported by a backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanCapabilities {
    /// Backend can transmit and receive CAN-FD frames.
    pub fd: bool,
    /// Maximum payload bytes per frame the backend will accept.
    pub max_dlen: usize,
}

/// Runtime bit-timing information reported by a CAN interface.
///
/// Both fields are optional because some backends can recover the sample
/// point from the applied time segments without knowing the controller clock,
/// while others may only expose a bitrate.  The sample point uses the same
/// per-mille representation as Linux CAN netlink (`800` means `0.800`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CanBitTiming {
    /// Applied bitrate in bits per second.
    pub bitrate: Option<u32>,
    /// Applied sample point in per-mille (`800` = `0.800`).
    pub sample_point_per_mille: Option<u16>,
}

/// Best-effort snapshot of the configuration currently applied to a link.
///
/// This deliberately describes runtime state rather than backend capability:
/// a SocketCAN backend can support CAN-FD while the selected interface is
/// currently configured for Classic CAN.  Optional fields mean "not exposed
/// by this backend/interface", never that a particular policy was accepted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CanLinkConfig {
    /// Whether CAN-FD is enabled on the link.
    pub fd_enabled: Option<bool>,
    /// Arbitration/nominal-phase timing, when observable.
    pub nominal: Option<CanBitTiming>,
    /// CAN-FD data-phase timing, when configured and observable.
    pub data: Option<CanBitTiming>,
}

/// CAN controller fault-confinement state (ISO 11898-1). Ordered from healthy
/// to dead; the numeric thresholds are the classic REC/TEC boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanControllerState {
    /// Error counters < 96 — normal operation.
    ErrorActive,
    /// A counter reached 96 — still transmitting, worth investigating.
    ErrorWarning,
    /// A counter reached 128 — only recessive error flags, ACKs still work.
    ErrorPassive,
    /// TEC reached 256 — the controller left the bus.
    BusOff,
    /// Controller stopped / not started.
    Stopped,
    /// Controller in a sleep state.
    Sleeping,
}

/// Best-effort snapshot of controller health, returned by
/// [`CanBus::bus_state`]. Every field is optional: backends report what they
/// can and leave the rest `None` (e.g. a driver may expose the state but not
/// the raw error counters, or vice versa).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CanBusState {
    /// Fault-confinement state of the controller.
    pub state: Option<CanControllerState>,
    /// Transmit error counter (TEC).
    pub tx_errors: Option<u16>,
    /// Receive error counter (REC).
    pub rx_errors: Option<u16>,
}

/// A shared CAN bus.
///
/// All methods take `&self`, so a `CanBus` can be wrapped in an `Arc`
/// and shared across tasks freely.
#[async_trait]
pub trait CanBus: Send + Sync {
    /// Transmit a frame. Implementations must internally serialize
    /// concurrent sends.
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError>;

    /// Open a new receive subscription. Frames not matching `filter`
    /// will not be delivered to the returned receiver.
    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError>;

    /// Static description of what the backend supports.
    fn capabilities(&self) -> CanCapabilities;

    /// Best-effort snapshot of controller state + error counters.
    ///
    /// `Ok(None)` means this backend cannot report bus health (the default —
    /// existing and minimal implementations need no code). Wrapper/decorator
    /// implementations should forward this explicitly, or they will mask the
    /// inner backend's support with the default.
    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        Ok(None)
    }

    /// Best-effort snapshot of the currently applied link configuration.
    ///
    /// `Ok(None)` means this backend cannot inspect or reconstruct its runtime
    /// configuration.  A backend that supports this query should return an
    /// error when the query itself fails instead of silently substituting
    /// unknown values.  Wrapper/decorator implementations should forward this
    /// explicitly, or they will mask support in the inner backend.
    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        Ok(None)
    }
}

/// A single receive subscription. Drop to unsubscribe.
#[async_trait]
pub trait CanRx: Send {
    /// Wait for the next frame. Returns `Disconnected` if the bus has
    /// shut down.
    async fn recv(&mut self) -> Result<CanFrame, CanIoError>;

    /// Non-blocking receive. `Ok(None)` means "no frame ready right now".
    fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError>;
}

// Blanket impl: `Box<dyn CanBus>` is itself a `CanBus`.
// NOTE: `bus_state` must be forwarded explicitly — relying on the trait
// default here would silently mask the inner backend's implementation.
#[async_trait]
impl<T: CanBus + ?Sized> CanBus for Box<T> {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        (**self).send(frame).await
    }
    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        (**self).subscribe(filter).await
    }
    fn capabilities(&self) -> CanCapabilities {
        (**self).capabilities()
    }
    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        (**self).bus_state().await
    }
    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        (**self).link_config().await
    }
}

#[async_trait]
impl<T: CanBus + ?Sized> CanBus for std::sync::Arc<T> {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        (**self).send(frame).await
    }
    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        (**self).subscribe(filter).await
    }
    fn capabilities(&self) -> CanCapabilities {
        (**self).capabilities()
    }
    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        (**self).bus_state().await
    }
    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        (**self).link_config().await
    }
}
