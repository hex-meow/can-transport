//! Error type for `CanBus` implementations.

use thiserror::Error;

/// Errors returned by [`crate::CanBus`] and [`crate::CanRx`].
#[derive(Debug, Error)]
pub enum CanIoError {
    /// The bus is disconnected, closed, or the backing driver has gone away.
    #[error("CAN bus disconnected")]
    Disconnected,

    /// The provided CAN identifier was out of range for its width.
    #[error("invalid CAN identifier")]
    InvalidId,

    /// Frame payload is too long for the requested frame kind.
    #[error("frame data too long: got {got} bytes, max {max}")]
    DataTooLong { got: usize, max: usize },

    /// A subscriber's queue overflowed and frames were dropped.
    /// The number reports how many frames were lost since the last
    /// successful `recv`.
    #[error("subscriber lagged behind, {dropped} frames dropped")]
    Lagged { dropped: u64 },

    /// Any backend-specific error.
    #[error(transparent)]
    Backend(#[from] Box<dyn std::error::Error + Send + Sync>),
}

impl CanIoError {
    pub fn backend<E>(err: E) -> Self
    where
        E: std::error::Error + Send + Sync + 'static,
    {
        Self::Backend(Box::new(err))
    }
}
