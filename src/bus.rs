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
}
