//! CAN / CAN-FD frame and identifier types.

use crate::error::CanIoError;

/// Maximum data length for a single CAN-FD frame, in bytes.
pub const MAX_DLEN: usize = 64;

/// 11-bit standard or 29-bit extended CAN identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CanId {
    Standard(u16),
    Extended(u32),
}

impl CanId {
    pub const STANDARD_MAX: u16 = 0x7FF;
    pub const EXTENDED_MAX: u32 = 0x1FFF_FFFF;

    pub fn raw(self) -> u32 {
        match self {
            CanId::Standard(id) => id as u32,
            CanId::Extended(id) => id,
        }
    }

    pub fn is_standard(self) -> bool {
        matches!(self, CanId::Standard(_))
    }

    pub fn is_extended(self) -> bool {
        matches!(self, CanId::Extended(_))
    }

    pub fn new_standard(id: u16) -> Result<Self, CanIoError> {
        if id > Self::STANDARD_MAX {
            Err(CanIoError::InvalidId)
        } else {
            Ok(CanId::Standard(id))
        }
    }

    pub fn new_extended(id: u32) -> Result<Self, CanIoError> {
        if id > Self::EXTENDED_MAX {
            Err(CanIoError::InvalidId)
        } else {
            Ok(CanId::Extended(id))
        }
    }
}

impl From<u16> for CanId {
    fn from(id: u16) -> Self {
        CanId::Standard(id & Self::STANDARD_MAX)
    }
}

/// Frame classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameKind {
    /// Classic CAN data frame (≤ 8 bytes).
    Data,
    /// CAN-FD data frame (≤ 64 bytes). `brs` toggles the bit-rate switch.
    Fd { brs: bool },
    /// Classic CAN remote transmission request.
    Remote,
}

/// A CAN / CAN-FD frame.
///
/// Backed by a fixed-size buffer so the type is `Copy`-friendly and
/// allocation-free, which matters when fan-out happens on every received
/// frame. The actual payload length is stored separately in `len`.
///
/// Received frames may carry a hardware timestamp (see [`Self::timestamp_us`])
/// when the backend supports and enables it; constructed frames never do.
#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    id: CanId,
    kind: FrameKind,
    len: u8,
    data: [u8; MAX_DLEN],
    /// Backend-provided receive timestamp, µs. `None` = not available.
    ts_us: Option<u64>,
}

/// Equality is over the frame *content* (id, kind, payload) — the receive
/// timestamp is deliberately excluded, so a constructed frame compares equal
/// to the same frame received off the wire.
impl PartialEq for CanFrame {
    fn eq(&self, other: &Self) -> bool {
        // Bytes beyond `len` are always zeroed by the constructors, so a
        // full-array compare is equivalent to comparing the payloads.
        self.id == other.id
            && self.kind == other.kind
            && self.len == other.len
            && self.data == other.data
    }
}

impl Eq for CanFrame {}

impl CanFrame {
    /// Build a classic CAN data frame (≤ 8 bytes).
    pub fn new_data(id: impl Into<CanId>, payload: &[u8]) -> Result<Self, CanIoError> {
        if payload.len() > 8 {
            return Err(CanIoError::DataTooLong {
                got: payload.len(),
                max: 8,
            });
        }
        let mut data = [0u8; MAX_DLEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id: id.into(),
            kind: FrameKind::Data,
            len: payload.len() as u8,
            data,
            ts_us: None,
        })
    }

    /// Build a CAN-FD frame (≤ 64 bytes).
    pub fn new_fd(
        id: impl Into<CanId>,
        payload: &[u8],
        brs: bool,
    ) -> Result<Self, CanIoError> {
        if payload.len() > MAX_DLEN {
            return Err(CanIoError::DataTooLong {
                got: payload.len(),
                max: MAX_DLEN,
            });
        }
        let mut data = [0u8; MAX_DLEN];
        data[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            id: id.into(),
            kind: FrameKind::Fd { brs },
            len: payload.len() as u8,
            data,
            ts_us: None,
        })
    }

    /// Build a Remote Transmission Request (RTR) frame.
    pub fn new_remote(id: impl Into<CanId>, dlc: u8) -> Result<Self, CanIoError> {
        if dlc > 8 {
            return Err(CanIoError::DataTooLong {
                got: dlc as usize,
                max: 8,
            });
        }
        Ok(Self {
            id: id.into(),
            kind: FrameKind::Remote,
            len: dlc,
            data: [0u8; MAX_DLEN],
            ts_us: None,
        })
    }

    pub fn id(&self) -> CanId {
        self.id
    }

    pub fn kind(&self) -> FrameKind {
        self.kind
    }

    /// Payload bytes (empty for RTR frames).
    pub fn data(&self) -> &[u8] {
        if matches!(self.kind, FrameKind::Remote) {
            &[]
        } else {
            &self.data[..self.len as usize]
        }
    }

    /// Reported DLC. For RTR this is the requested length.
    pub fn dlc(&self) -> usize {
        self.len as usize
    }

    pub fn is_fd(&self) -> bool {
        matches!(self.kind, FrameKind::Fd { .. })
    }

    pub fn is_remote(&self) -> bool {
        matches!(self.kind, FrameKind::Remote)
    }

    pub fn brs(&self) -> bool {
        matches!(self.kind, FrameKind::Fd { brs: true })
    }

    /// Receive timestamp in microseconds, if the backend captured one.
    ///
    /// The epoch is backend-defined (e.g. for gs_usb it is the device's
    /// free-running clock, unwrapped to a monotonic 64-bit count by the
    /// backend); only *differences* between timestamps from the same bus are
    /// meaningful. `None` on constructed frames and on backends without
    /// timestamp support.
    pub fn timestamp_us(&self) -> Option<u64> {
        self.ts_us
    }

    /// Return a copy of this frame carrying a receive timestamp. Intended for
    /// backend implementations stamping frames on arrival.
    pub fn with_timestamp_us(mut self, us: u64) -> Self {
        self.ts_us = Some(us);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classic_frame_round_trip() {
        let f = CanFrame::new_data(0x123u16, &[1, 2, 3]).unwrap();
        assert_eq!(f.data(), &[1, 2, 3]);
        assert_eq!(f.id(), CanId::Standard(0x123));
        assert!(!f.is_fd());
    }

    #[test]
    fn fd_frame_64_bytes() {
        let payload = [0xAB; 64];
        let f = CanFrame::new_fd(CanId::Extended(0x1ABCDEF), &payload, true).unwrap();
        assert_eq!(f.data().len(), 64);
        assert!(f.is_fd());
        assert!(f.brs());
    }

    #[test]
    fn classic_rejects_over_8() {
        assert!(CanFrame::new_data(0x100u16, &[0; 9]).is_err());
    }

    #[test]
    fn timestamp_default_none_and_round_trip() {
        let f = CanFrame::new_data(0x123u16, &[1, 2]).unwrap();
        assert_eq!(f.timestamp_us(), None);
        let g = f.with_timestamp_us(42);
        assert_eq!(g.timestamp_us(), Some(42));
        // Payload untouched by stamping.
        assert_eq!(g.data(), &[1, 2]);
    }

    #[test]
    fn equality_ignores_timestamp() {
        let a = CanFrame::new_data(0x123u16, &[1, 2, 3]).unwrap();
        let b = a.with_timestamp_us(999);
        assert_eq!(a, b);
    }
}
