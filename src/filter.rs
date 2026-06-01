//! Frame filtering used by `CanBus::subscribe`.

use crate::frame::{CanFrame, CanId};

/// A bit-mask filter, applied to incoming frames before they are
/// delivered to a subscriber.
///
/// A frame `f` matches the filter iff
/// `(f.id().raw() & mask) == (id & mask)` *and* the frame's id width
/// (standard / extended) matches `extended`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanFilter {
    pub id: u32,
    pub mask: u32,
    pub extended: bool,
}

impl CanFilter {
    /// Match a single standard frame id exactly.
    pub fn exact_standard(id: u16) -> Self {
        Self {
            id: id as u32,
            mask: 0x7FF,
            extended: false,
        }
    }

    /// Match a single extended frame id exactly.
    pub fn exact_extended(id: u32) -> Self {
        Self {
            id,
            mask: 0x1FFF_FFFF,
            extended: true,
        }
    }

    /// Bit-mask filter on standard frames.
    pub fn standard(id: u16, mask: u16) -> Self {
        Self {
            id: id as u32,
            mask: mask as u32,
            extended: false,
        }
    }

    /// Bit-mask filter on extended frames.
    pub fn extended(id: u32, mask: u32) -> Self {
        Self {
            id,
            mask,
            extended: true,
        }
    }

    /// Pass-all (standard frames only).
    pub fn pass_all_standard() -> Self {
        Self {
            id: 0,
            mask: 0,
            extended: false,
        }
    }

    /// Pass-all (extended frames only).
    pub fn pass_all_extended() -> Self {
        Self {
            id: 0,
            mask: 0,
            extended: true,
        }
    }

    pub fn matches(&self, frame: &CanFrame) -> bool {
        match (frame.id(), self.extended) {
            (CanId::Standard(id), false) => (id as u32 & self.mask) == (self.id & self.mask),
            (CanId::Extended(id), true) => (id & self.mask) == (self.id & self.mask),
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_standard_matches_only_one_id() {
        let f = CanFilter::exact_standard(0x123);
        assert!(f.matches(&CanFrame::new_data(0x123u16, &[]).unwrap()));
        assert!(!f.matches(&CanFrame::new_data(0x124u16, &[]).unwrap()));
        assert!(!f.matches(&CanFrame::new_fd(CanId::Extended(0x123), &[], false).unwrap()));
    }

    #[test]
    fn mask_filter_groups_ids() {
        // CANopen TSDO of node 0x10 = 0x590; mask the lower 7 bits open.
        let f = CanFilter::standard(0x580 | 0x10, 0x7F0);
        assert!(f.matches(&CanFrame::new_data(0x590u16, &[]).unwrap()));
        assert!(f.matches(&CanFrame::new_data(0x59Fu16, &[]).unwrap()));
        assert!(!f.matches(&CanFrame::new_data(0x5A0u16, &[]).unwrap()));
    }
}
