//! gs_usb (candleLight) userspace backend — CAN 2.0 and CAN-FD.
//!
//! Enable with the `gs_usb` feature.
//!
//! This talks the [gs_usb] vendor protocol directly over USB using [`nusb`]
//! (pure-Rust, no libusb). It is the backend to use on **Windows** and
//! **macOS**, where there is no kernel CAN stack, and on **Linux** when the
//! in-kernel `gs_usb` driver is too old for CAN-FD (it detaches that driver
//! and drives the device from userspace instead).
//!
//! Platform notes:
//! - **macOS**: works with no driver install and no `sudo` — the OS does not
//!   claim vendor-specific devices.
//! - **Windows**: the device's interface must be bound to WinUSB. If you
//!   control the firmware, ship Microsoft OS 2.0 descriptors so Windows binds
//!   WinUSB automatically; otherwise bind it once with Zadig.
//! - **Linux**: needs write access to the usbfs node (root or a udev rule),
//!   and detaches the kernel `gs_usb` driver, so the SocketCAN `canX` interface
//!   for this device disappears while the backend is open.
//!
//! [gs_usb]: https://github.com/candle-usb/candleLight_fw
//!
//! ```no_run
//! # #[cfg(feature = "gs_usb")]
//! # async fn _doc() -> Result<(), can_transport::CanIoError> {
//! use can_transport::{CanBus, CanFilter, CanFrame};
//! use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};
//!
//! // 1 Mbit nominal / 5 Mbit data, 80 MHz device clock.
//! let bus = GsUsbBus::open(GsUsbConfig::fd_1m_5m()).await?;
//! let mut rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
//! let frame = rx.recv().await?;
//! println!("got: {:?}", frame);
//! # Ok(()) }
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};
use nusb::{Device, Interface};
use tokio::sync::{mpsc, Mutex};

use crate::bus::{CanBus, CanBusState, CanCapabilities, CanControllerState, CanRx};
use crate::error::CanIoError;
use crate::filter::CanFilter;
use crate::frame::{CanFrame, CanId, FrameKind, MAX_DLEN};

/// USB VID/PID pairs known to speak the gs_usb protocol. Used by
/// [`GsUsbBus::open`] when no explicit VID/PID is given.
pub const KNOWN_IDS: &[(u16, u16)] = &[
    (0x1209, 0x2323), // generic candleLight / bytewerk.org
    (0x1d50, 0x606f), // Geschwister Schneider / candleLight
    (0x1d50, 0x600f), // Geschwister Schneider (older)
];

const CONTROL_TIMEOUT: Duration = Duration::from_secs(1);
const BULK_IN: u8 = 0x81;
const BULK_OUT: u8 = 0x01;
/// One USB max packet on this high-speed device; holds any single frame.
const READ_LEN: usize = 512;
/// How many IN transfers we keep in flight, to not drop frames between reads.
const IN_FLIGHT: usize = 8;
/// Per-subscriber inbox depth; overflow surfaces as `CanIoError::Lagged`.
const SUBSCRIBER_QUEUE: usize = 256;

// ---------- gs_usb wire protocol ----------

// Vendor request codes (`bRequest`).
const BREQ_HOST_FORMAT: u8 = 0;
const BREQ_BITTIMING: u8 = 1;
const BREQ_MODE: u8 = 2;
const BREQ_BT_CONST: u8 = 4;
const BREQ_DATA_BITTIMING: u8 = 10;
const BREQ_GET_STATE: u8 = 14;

// Device mode words.
const MODE_START: u32 = 1;
// Mode feature-enable flags.
const MODE_LISTEN_ONLY: u32 = 1 << 0;
const MODE_LOOP_BACK: u32 = 1 << 1;
const MODE_HW_TIMESTAMP: u32 = 1 << 4;
const MODE_FD: u32 = 1 << 8;

// Device feature bits (first u32 of the BT_CONST reply).
const FEATURE_HW_TIMESTAMP: u32 = 1 << 4;
const FEATURE_GET_STATE: u32 = 1 << 13;

/// BT_CONST reply length: feature + fclk_can + 8 timing-limit words.
const BT_CONST_LEN: usize = 40;
/// GET_STATE reply: `{ state: u32, rxerr: u32, txerr: u32 }`.
const GET_STATE_LEN: usize = 12;

// Per-frame flags (`gs_host_frame.flags`).
const FLAG_FD: u8 = 1 << 1;
const FLAG_BRS: u8 = 1 << 2;

// SocketCAN-style bits in `gs_host_frame.can_id`.
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_ERR_FLAG: u32 = 0x2000_0000;
const CAN_SFF_MASK: u32 = 0x0000_07FF;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;

/// `echo_id` value the device uses to mark a genuinely received frame.
/// Anything else is the echo of one of our own transmissions.
const ECHO_ID_RX: u32 = 0xFFFF_FFFF;

/// gs_host_frame header size: echo_id, can_id, can_dlc, channel, flags, reserved.
const HDR_LEN: usize = 12;

/// CAN-FD DLC -> payload length.
fn fd_dlc2len(dlc: u8) -> usize {
    const LUT: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];
    LUT[(dlc & 0x0F) as usize]
}

/// Payload length -> smallest CAN-FD DLC that fits it.
fn fd_len2dlc(len: usize) -> u8 {
    const LUT: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];
    for (dlc, &l) in LUT.iter().enumerate() {
        if l >= len {
            return dlc as u8;
        }
    }
    15
}

// ---------- public config ----------

/// One CAN bit-timing segment set, in the device's raw units. The bit time is
/// `1 + (prop_seg + phase_seg1) + phase_seg2` time quanta, each `brp / fclk`
/// seconds long.
#[derive(Debug, Clone, Copy)]
pub struct GsTiming {
    pub prop_seg: u32,
    pub phase_seg1: u32,
    pub phase_seg2: u32,
    pub sjw: u32,
    pub brp: u32,
}

impl GsTiming {
    fn to_bytes(self) -> [u8; 20] {
        let mut b = [0u8; 20];
        b[0..4].copy_from_slice(&self.prop_seg.to_le_bytes());
        b[4..8].copy_from_slice(&self.phase_seg1.to_le_bytes());
        b[8..12].copy_from_slice(&self.phase_seg2.to_le_bytes());
        b[12..16].copy_from_slice(&self.sjw.to_le_bytes());
        b[16..20].copy_from_slice(&self.brp.to_le_bytes());
        b
    }
}

/// How to bring up a gs_usb channel.
///
/// Timing is given explicitly in device units rather than computed from a
/// target bit-rate: the values below are exactly what a known-good
/// configuration produced, which keeps this backend honest about what it sends
/// to the hardware. A general bit-rate -> timing solver is a future addition.
#[derive(Debug, Clone, Copy)]
pub struct GsUsbConfig {
    /// Channel index on a multi-channel device (`can0` = 0).
    pub channel: u16,
    /// Enable CAN-FD. Requires `data` to be set.
    pub fd: bool,
    /// Nominal (arbitration) bit timing.
    pub nominal: GsTiming,
    /// Data-phase bit timing; used only when `fd` is true.
    pub data: Option<GsTiming>,
    /// Receive-only: never emit dominant bits on the bus.
    pub listen_only: bool,
    /// Internal loopback (for self-test without a bus).
    pub loopback: bool,
    /// Ask the device to stamp received frames with its hardware clock
    /// (µs, surfaced via [`CanFrame::timestamp_us`]). Requires firmware
    /// support (`GS_CAN_FEATURE_HW_TIMESTAMP`); silently disabled with a
    /// warning if the device doesn't advertise it.
    pub hw_timestamp: bool,
}

impl GsUsbConfig {
    /// Classic CAN, 1 Mbit, 80 MHz device clock (sample point 0.8).
    pub fn classic_1m() -> Self {
        Self {
            channel: 0,
            fd: false,
            nominal: GsTiming {
                prop_seg: 31,
                phase_seg1: 32,
                phase_seg2: 16,
                sjw: 5,
                brp: 1,
            },
            data: None,
            listen_only: false,
            loopback: false,
            hw_timestamp: false,
        }
    }

    /// CAN-FD, 1 Mbit nominal / 5 Mbit data, 80 MHz device clock.
    ///
    /// Matches `ip link set canX type can bitrate 1000000 sample-point 0.8
    /// dbitrate 5000000 dsample-point 0.75 sjw 5 dsjw 3 fd on`.
    pub fn fd_1m_5m() -> Self {
        Self {
            channel: 0,
            fd: true,
            nominal: GsTiming {
                prop_seg: 31,
                phase_seg1: 32,
                phase_seg2: 16,
                sjw: 5,
                brp: 1,
            },
            data: Some(GsTiming {
                prop_seg: 5,
                phase_seg1: 6,
                phase_seg2: 4,
                sjw: 3,
                brp: 1,
            }),
            listen_only: false,
            loopback: false,
            hw_timestamp: false,
        }
    }

    /// Select which channel of a multi-channel adapter to use (`can0` = 0).
    pub fn with_channel(mut self, channel: u16) -> Self {
        self.channel = channel;
        self
    }

    /// Request device hardware timestamps on received frames (if the
    /// firmware supports them).
    pub fn with_hw_timestamp(mut self, on: bool) -> Self {
        self.hw_timestamp = on;
        self
    }
}

// ---------- fan-out registry (mirrors the SocketCAN backend) ----------

type SubId = u64;

struct Subscriber {
    filter: CanFilter,
    tx: mpsc::Sender<CanFrame>,
    dropped: Arc<AtomicU64>,
}

struct Registry {
    subs: Mutex<HashMap<SubId, Subscriber>>,
    next_id: AtomicU64,
}

impl Registry {
    fn new() -> Self {
        Self {
            subs: Mutex::new(HashMap::new()),
            next_id: AtomicU64::new(1),
        }
    }
}

// ---------- the bus ----------

/// gs_usb-backed [`CanBus`]. Wrap in `Arc` to share.
pub struct GsUsbBus {
    out_ep: Mutex<nusb::Endpoint<Bulk, Out>>,
    registry: Arc<Registry>,
    reader: tokio::task::JoinHandle<()>,
    echo: AtomicU32,
    fd: bool,
    channel: u16,
    /// Device feature word from BT_CONST (0 if the probe failed).
    features: u32,
    /// Hardware timestamps were requested *and* the device supports them.
    hw_ts: bool,
    // Kept alive so the device/interface stay claimed for the bus's lifetime.
    // Also used for on-demand control requests (GET_STATE).
    interface: Interface,
    _device: Device,
}

impl GsUsbBus {
    /// Open the first connected device matching one of [`KNOWN_IDS`].
    pub async fn open(config: GsUsbConfig) -> Result<Self, CanIoError> {
        let info = nusb::list_devices()
            .await
            .map_err(CanIoError::backend)?
            .find(|d| KNOWN_IDS.contains(&(d.vendor_id(), d.product_id())))
            .ok_or(CanIoError::Disconnected)?;
        let device = info.open().await.map_err(CanIoError::backend)?;
        Self::from_device(device, config).await
    }

    /// Open a specific device by USB vendor/product id.
    pub async fn open_vid_pid(
        vid: u16,
        pid: u16,
        config: GsUsbConfig,
    ) -> Result<Self, CanIoError> {
        let info = nusb::list_devices()
            .await
            .map_err(CanIoError::backend)?
            .find(|d| d.vendor_id() == vid && d.product_id() == pid)
            .ok_or(CanIoError::Disconnected)?;
        let device = info.open().await.map_err(CanIoError::backend)?;
        Self::from_device(device, config).await
    }

    async fn from_device(device: Device, config: GsUsbConfig) -> Result<Self, CanIoError> {
        if config.fd && config.data.is_none() {
            return Err(CanIoError::backend(ConfigError(
                "fd = true requires a data bit timing",
            )));
        }

        // Detaches the kernel gs_usb driver on Linux; a plain claim elsewhere.
        let interface = device
            .detach_and_claim_interface(0)
            .await
            .map_err(CanIoError::backend)?;

        let chan = config.channel;

        // 1. Tell the device our byte order (little-endian magic 0x0000beef).
        control_out(&interface, BREQ_HOST_FORMAT, 1, &0x0000_beefu32.to_le_bytes()).await?;

        // 1b. Probe the device feature word (first u32 of BT_CONST). Gates
        // hardware timestamps and GET_STATE below.
        let features = match control_in(&interface, BREQ_BT_CONST, chan, BT_CONST_LEN).await {
            Ok(buf) if buf.len() >= 4 => u32::from_le_bytes(buf[0..4].try_into().unwrap()),
            Ok(_) | Err(_) => {
                log::warn!("gs_usb: BT_CONST probe failed; assuming no optional features");
                0
            }
        };

        let hw_ts = config.hw_timestamp && (features & FEATURE_HW_TIMESTAMP != 0);
        if config.hw_timestamp && !hw_ts {
            log::warn!(
                "gs_usb: hardware timestamps requested but the device does not \
                 advertise GS_CAN_FEATURE_HW_TIMESTAMP; falling back to none"
            );
        }

        // 2. Bit timing (nominal, and data phase for FD).
        control_out(&interface, BREQ_BITTIMING, chan, &config.nominal.to_bytes()).await?;
        if config.fd {
            let data = config.data.expect("checked above");
            control_out(&interface, BREQ_DATA_BITTIMING, chan, &data.to_bytes()).await?;
        }

        // 3. Start the channel with the chosen feature flags.
        let mut flags = 0u32;
        if config.fd {
            flags |= MODE_FD;
        }
        if config.listen_only {
            flags |= MODE_LISTEN_ONLY;
        }
        if config.loopback {
            flags |= MODE_LOOP_BACK;
        }
        if hw_ts {
            flags |= MODE_HW_TIMESTAMP;
        }
        let mut mode = [0u8; 8];
        mode[0..4].copy_from_slice(&MODE_START.to_le_bytes());
        mode[4..8].copy_from_slice(&flags.to_le_bytes());
        control_out(&interface, BREQ_MODE, chan, &mode).await?;

        let in_ep = interface
            .endpoint::<Bulk, In>(BULK_IN)
            .map_err(CanIoError::backend)?;
        let out_ep = interface
            .endpoint::<Bulk, Out>(BULK_OUT)
            .map_err(CanIoError::backend)?;

        let registry = Arc::new(Registry::new());
        let reader = tokio::spawn(reader_task(in_ep, registry.clone(), chan as u8, hw_ts));

        Ok(Self {
            out_ep: Mutex::new(out_ep),
            registry,
            reader,
            echo: AtomicU32::new(0),
            fd: config.fd,
            channel: chan,
            features,
            hw_ts,
            interface,
            _device: device,
        })
    }

    /// `true` when the device is stamping received frames with its hardware
    /// clock (requested via [`GsUsbConfig::hw_timestamp`] and supported by
    /// the firmware).
    pub fn hw_timestamps_active(&self) -> bool {
        self.hw_ts
    }

    /// Raw device feature word from BT_CONST (`GS_CAN_FEATURE_*` bits).
    pub fn device_features(&self) -> u32 {
        self.features
    }
}

impl Drop for GsUsbBus {
    fn drop(&mut self) {
        // Best-effort: stop the reader. The CAN channel itself keeps running on
        // the device until it is reset or unplugged; re-opening re-starts it.
        self.reader.abort();
    }
}

/// Issue a vendor control-OUT to interface 0.
async fn control_out(
    iface: &Interface,
    request: u8,
    value: u16,
    data: &[u8],
) -> Result<(), CanIoError> {
    iface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index: 0, // interface number
                data,
            },
            CONTROL_TIMEOUT,
        )
        .await
        .map_err(CanIoError::backend)
}

/// Issue a vendor control-IN to interface 0, reading up to `len` bytes.
async fn control_in(
    iface: &Interface,
    request: u8,
    value: u16,
    len: usize,
) -> Result<Vec<u8>, CanIoError> {
    iface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index: 0, // interface number
                length: len as u16,
            },
            CONTROL_TIMEOUT,
        )
        .await
        .map_err(CanIoError::backend)
}

async fn reader_task(
    mut in_ep: nusb::Endpoint<Bulk, In>,
    registry: Arc<Registry>,
    channel: u8,
    hw_ts: bool,
) {
    for _ in 0..IN_FLIGHT {
        in_ep.submit(Buffer::new(READ_LEN));
    }
    // The device timestamp is a free-running 32-bit µs counter that wraps
    // every ~71.6 minutes; unwrap it into a monotonic u64. Frames arrive in
    // order on this single task, so a raw value smaller than the previous
    // one means exactly one wrap (a >71-minute silent gap on this channel
    // would be missed — acceptable, only deltas are meaningful).
    let mut ts_last: u32 = 0;
    let mut ts_epoch: u64 = 0;
    loop {
        let completion = in_ep.next_complete().await;
        if let Err(e) = completion.status {
            // Cancelled on shutdown, or the device went away.
            log::warn!("gs_usb read error: {e:?}; reader exiting");
            return;
        }
        let bytes = &completion.buffer[..completion.actual_len];
        if let Some((frame, raw_ts)) = parse_host_frame(bytes, channel, hw_ts) {
            let frame = match raw_ts {
                Some(raw) => frame.with_timestamp_us(unwrap_ts(&mut ts_last, &mut ts_epoch, raw)),
                None => frame,
            };
            dispatch(&registry, frame).await;
        }
        in_ep.submit(Buffer::new(READ_LEN));
    }
}

/// Unwrap the device's free-running 32-bit µs counter into a monotonic u64.
/// A raw value below the previous one means the counter wrapped (every
/// ~71.6 min); gaps longer than one full wrap on a silent channel are
/// undetectable and fold into one wrap. Note the tracker only sees *received*
/// frames — TX echoes and error frames are dropped before the timestamp is
/// parsed, so an RX-silent (but transmitting) stretch >71.6 min also folds.
fn unwrap_ts(ts_last: &mut u32, ts_epoch: &mut u64, raw: u32) -> u64 {
    if raw < *ts_last {
        *ts_epoch += 1u64 << 32;
    }
    *ts_last = raw;
    *ts_epoch | raw as u64
}

async fn dispatch(registry: &Registry, frame: CanFrame) {
    let subs = registry.subs.lock().await;
    for sub in subs.values() {
        if !sub.filter.matches(&frame) {
            continue;
        }
        match sub.tx.try_send(frame) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(_)) => {
                sub.dropped.fetch_add(1, Ordering::Relaxed);
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {}
        }
    }
}

/// Parse one `gs_host_frame` off the bulk-IN endpoint. Returns `None` for our
/// own transmit echoes, frames on a different channel, CAN error frames, and
/// runts. When `hw_ts` is set, also extracts the device timestamp: the wire
/// layout is per-frame — header(12) + data arm (64 for FD frames, 8 for
/// classic/RTR) + a trailing `u32` µs counter.
fn parse_host_frame(
    buf: &[u8],
    expected_channel: u8,
    hw_ts: bool,
) -> Option<(CanFrame, Option<u32>)> {
    if buf.len() < HDR_LEN {
        return None;
    }
    let echo_id = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if echo_id != ECHO_ID_RX {
        // Echo of one of our own sends (a TX completion ack); not a received frame.
        return None;
    }
    // On a multi-channel adapter, ignore frames belonging to other channels.
    if buf[9] != expected_channel {
        return None;
    }
    let raw_id = u32::from_le_bytes(buf[4..8].try_into().unwrap());
    let dlc = buf[8];
    let flags = buf[10];

    if raw_id & CAN_ERR_FLAG != 0 {
        log::debug!("gs_usb CAN error frame ignored: {raw_id:#010x}");
        return None;
    }

    let extended = raw_id & CAN_EFF_FLAG != 0;
    let id = if extended {
        CanId::Extended(raw_id & CAN_EFF_MASK)
    } else {
        CanId::Standard((raw_id & CAN_SFF_MASK) as u16)
    };

    let is_fd = flags & FLAG_FD != 0;
    // The device timestamp sits after the frame's data arm.
    let ts = if hw_ts {
        let arm = if is_fd { MAX_DLEN } else { 8 };
        let off = HDR_LEN + arm;
        if buf.len() >= off + 4 {
            Some(u32::from_le_bytes(buf[off..off + 4].try_into().unwrap()))
        } else {
            log::debug!("gs_usb: hw timestamp expected but frame too short ({})", buf.len());
            None
        }
    } else {
        None
    };

    if raw_id & CAN_RTR_FLAG != 0 {
        return CanFrame::new_remote(id, dlc.min(8)).ok().map(|f| (f, ts));
    }

    let len = if is_fd {
        fd_dlc2len(dlc)
    } else {
        (dlc as usize).min(8)
    };
    if buf.len() < HDR_LEN + len {
        return None;
    }
    let payload = &buf[HDR_LEN..HDR_LEN + len];

    let frame = if is_fd {
        CanFrame::new_fd(id, payload, flags & FLAG_BRS != 0).ok()
    } else {
        CanFrame::new_data(id, payload).ok()
    };
    frame.map(|f| (f, ts))
}

#[async_trait]
impl CanBus for GsUsbBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        let echo = self.echo.fetch_add(1, Ordering::Relaxed) & 0x7FFF_FFFF;
        let bytes = encode_host_frame(&frame, echo, self.fd, self.channel)?;

        let mut ep = self.out_ep.lock().await;
        ep.submit(Buffer::from(bytes));
        let completion = ep.next_complete().await;
        completion.status.map_err(CanIoError::backend)
    }

    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        let (tx, rx) = mpsc::channel(SUBSCRIBER_QUEUE);
        let dropped = Arc::new(AtomicU64::new(0));
        let id = self.registry.next_id.fetch_add(1, Ordering::Relaxed);
        {
            let mut subs = self.registry.subs.lock().await;
            subs.insert(
                id,
                Subscriber {
                    filter,
                    tx,
                    dropped: dropped.clone(),
                },
            );
        }
        Ok(Box::new(GsUsbRx {
            rx,
            id,
            registry: self.registry.clone(),
            dropped,
        }))
    }

    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: self.fd,
            max_dlen: if self.fd { MAX_DLEN } else { 8 },
        }
    }

    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        if self.features & FEATURE_GET_STATE == 0 {
            // Firmware without GET_STATE — health is simply unknown.
            return Ok(None);
        }
        let buf = control_in(&self.interface, BREQ_GET_STATE, self.channel, GET_STATE_LEN).await?;
        if buf.len() < GET_STATE_LEN {
            return Err(CanIoError::backend(ProtocolError(
                "short GET_STATE reply from device",
            )));
        }
        // struct gs_device_state { u32 state; u32 rxerr; u32 txerr } (LE).
        let state = u32::from_le_bytes(buf[0..4].try_into().unwrap());
        let rxerr = u32::from_le_bytes(buf[4..8].try_into().unwrap());
        let txerr = u32::from_le_bytes(buf[8..12].try_into().unwrap());
        let state = match state {
            0 => Some(CanControllerState::ErrorActive),
            1 => Some(CanControllerState::ErrorWarning),
            2 => Some(CanControllerState::ErrorPassive),
            3 => Some(CanControllerState::BusOff),
            4 => Some(CanControllerState::Stopped),
            5 => Some(CanControllerState::Sleeping),
            other => {
                log::debug!("gs_usb: unknown gs_can_state {other}");
                None
            }
        };
        Ok(Some(CanBusState {
            state,
            tx_errors: Some(txerr.min(u16::MAX as u32) as u16),
            rx_errors: Some(rxerr.min(u16::MAX as u32) as u16),
        }))
    }
}

/// Serialize a [`CanFrame`] into a `gs_host_frame`. In FD mode the frame is the
/// 64-byte-data variant (header + 64), even for classic frames.
fn encode_host_frame(
    frame: &CanFrame,
    echo_id: u32,
    fd_mode: bool,
    channel: u16,
) -> Result<Vec<u8>, CanIoError> {
    let mut raw_id = frame.id().raw();
    if frame.id().is_extended() {
        raw_id |= CAN_EFF_FLAG;
    }

    let (dlc, payload, flags) = match frame.kind() {
        FrameKind::Remote => (frame.dlc() as u8, &[][..], 0u8),
        FrameKind::Data => (frame.data().len() as u8, frame.data(), 0u8),
        FrameKind::Fd { brs } => {
            let len = frame.data().len();
            let f = FLAG_FD | if brs { FLAG_BRS } else { 0 };
            (fd_len2dlc(len), frame.data(), f)
        }
    };
    if matches!(frame.kind(), FrameKind::Remote) {
        raw_id |= CAN_RTR_FLAG;
    }

    let data_field = if fd_mode { MAX_DLEN } else { 8 };
    let mut buf = vec![0u8; HDR_LEN + data_field];
    buf[0..4].copy_from_slice(&echo_id.to_le_bytes());
    buf[4..8].copy_from_slice(&raw_id.to_le_bytes());
    buf[8] = dlc;
    buf[9] = channel as u8;
    buf[10] = flags;
    // buf[11] reserved = 0
    buf[HDR_LEN..HDR_LEN + payload.len()].copy_from_slice(payload);
    Ok(buf)
}

// ---------- receive subscription ----------

struct GsUsbRx {
    rx: mpsc::Receiver<CanFrame>,
    id: SubId,
    registry: Arc<Registry>,
    dropped: Arc<AtomicU64>,
}

impl Drop for GsUsbRx {
    fn drop(&mut self) {
        let registry = self.registry.clone();
        let id = self.id;
        if let Ok(handle) = tokio::runtime::Handle::try_current() {
            handle.spawn(async move {
                registry.subs.lock().await.remove(&id);
            });
        }
    }
}

#[async_trait]
impl CanRx for GsUsbRx {
    async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        if dropped > 0 {
            return Err(CanIoError::Lagged { dropped });
        }
        self.rx.recv().await.ok_or(CanIoError::Disconnected)
    }

    fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        if dropped > 0 {
            return Err(CanIoError::Lagged { dropped });
        }
        match self.rx.try_recv() {
            Ok(f) => Ok(Some(f)),
            Err(mpsc::error::TryRecvError::Empty) => Ok(None),
            Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
        }
    }
}

#[derive(Debug)]
struct ConfigError(&'static str);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for ConfigError {}

#[derive(Debug)]
struct ProtocolError(&'static str);

impl std::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for ProtocolError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dlc_len_round_trip() {
        for &len in &[0, 1, 8, 12, 16, 20, 24, 32, 48, 64] {
            assert_eq!(fd_dlc2len(fd_len2dlc(len)), len);
        }
        // Non-canonical lengths round up to the next FD size.
        assert_eq!(fd_dlc2len(fd_len2dlc(13)), 16);
        assert_eq!(fd_dlc2len(fd_len2dlc(60)), 64);
    }

    #[test]
    fn rx_echo_frames_are_skipped() {
        let mut buf = vec![0u8; HDR_LEN + 8];
        buf[0..4].copy_from_slice(&7u32.to_le_bytes()); // echo_id != RX sentinel
        assert!(parse_host_frame(&buf, 0, false).is_none());
    }

    #[test]
    fn rx_other_channel_is_skipped() {
        let mut buf = vec![0u8; HDR_LEN + 8];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&0x123u32.to_le_bytes());
        buf[9] = 1; // channel 1
        // A bus listening on channel 0 must not see channel 1's traffic.
        assert!(parse_host_frame(&buf, 0, false).is_none());
        // ...but a channel-1 bus does.
        assert!(parse_host_frame(&buf, 1, false).is_some());
    }

    #[test]
    fn rx_standard_data_frame() {
        let mut buf = vec![0u8; HDR_LEN + 8];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&0x123u32.to_le_bytes());
        buf[8] = 3; // dlc
        buf[HDR_LEN..HDR_LEN + 3].copy_from_slice(&[0xAA, 0xBB, 0xCC]);
        let (f, ts) = parse_host_frame(&buf, 0, false).unwrap();
        assert_eq!(f.id(), CanId::Standard(0x123));
        assert_eq!(f.data(), &[0xAA, 0xBB, 0xCC]);
        assert!(!f.is_fd());
        assert_eq!(ts, None);
    }

    #[test]
    fn rx_extended_fd_brs_frame() {
        let mut buf = vec![0u8; HDR_LEN + 64];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&(0x1ABCDEF | CAN_EFF_FLAG).to_le_bytes());
        buf[8] = 10; // FD dlc 10 -> 16 bytes
        buf[10] = FLAG_FD | FLAG_BRS;
        let (f, _) = parse_host_frame(&buf, 0, false).unwrap();
        assert_eq!(f.id(), CanId::Extended(0x1ABCDEF));
        assert_eq!(f.data().len(), 16);
        assert!(f.is_fd());
        assert!(f.brs());
    }

    #[test]
    fn tx_round_trips_through_parse() {
        // 13 bytes is not a valid FD length; it must round up to 16 (zero-padded).
        let payload: Vec<u8> = (1..=13).collect();
        let orig = CanFrame::new_fd(CanId::Extended(0x1234567), &payload, true).unwrap();
        let mut bytes = encode_host_frame(&orig, ECHO_ID_RX, true, 0).unwrap();
        // encode uses a real echo id on the wire; force RX sentinel to parse back.
        bytes[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        let (back, _) = parse_host_frame(&bytes, 0, false).unwrap();
        assert_eq!(back.id(), orig.id());
        assert!(back.is_fd());
        assert!(back.brs());
        assert_eq!(back.data().len(), 16);
        assert_eq!(&back.data()[..13], payload.as_slice());
        assert_eq!(&back.data()[13..], &[0, 0, 0]);
    }

    #[test]
    fn rx_classic_frame_with_hw_timestamp() {
        // header(12) + classic arm (8) + u32 ts.
        let mut buf = vec![0u8; HDR_LEN + 8 + 4];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&0x123u32.to_le_bytes());
        buf[8] = 2;
        buf[HDR_LEN..HDR_LEN + 2].copy_from_slice(&[0x11, 0x22]);
        buf[HDR_LEN + 8..HDR_LEN + 12].copy_from_slice(&123_456u32.to_le_bytes());
        let (f, ts) = parse_host_frame(&buf, 0, true).unwrap();
        assert_eq!(f.data(), &[0x11, 0x22]);
        assert_eq!(ts, Some(123_456));
    }

    #[test]
    fn rx_fd_frame_with_hw_timestamp() {
        // header(12) + FD arm (64) + u32 ts.
        let mut buf = vec![0u8; HDR_LEN + 64 + 4];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&0x321u32.to_le_bytes());
        buf[8] = 9; // FD dlc 9 -> 12 bytes
        buf[10] = FLAG_FD;
        buf[HDR_LEN + 64..HDR_LEN + 68].copy_from_slice(&777u32.to_le_bytes());
        let (f, ts) = parse_host_frame(&buf, 0, true).unwrap();
        assert!(f.is_fd());
        assert_eq!(f.data().len(), 12);
        assert_eq!(ts, Some(777));
    }

    #[test]
    fn rx_rtr_frame_with_hw_timestamp() {
        // RTR uses the classic arm: header(12) + 8 + u32 ts.
        let mut buf = vec![0u8; HDR_LEN + 8 + 4];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&(0x701u32 | CAN_RTR_FLAG).to_le_bytes());
        buf[8] = 1;
        buf[HDR_LEN + 8..HDR_LEN + 12].copy_from_slice(&42u32.to_le_bytes());
        let (f, ts) = parse_host_frame(&buf, 0, true).unwrap();
        assert!(f.is_remote());
        assert_eq!(f.dlc(), 1);
        assert_eq!(ts, Some(42));
    }

    #[test]
    fn hw_timestamp_missing_bytes_degrades_to_none() {
        // hw_ts on, but no trailing u32 — frame still parses, ts is None.
        let mut buf = vec![0u8; HDR_LEN + 8];
        buf[0..4].copy_from_slice(&ECHO_ID_RX.to_le_bytes());
        buf[4..8].copy_from_slice(&0x123u32.to_le_bytes());
        buf[8] = 1;
        let (_, ts) = parse_host_frame(&buf, 0, true).unwrap();
        assert_eq!(ts, None);
    }

    #[test]
    fn ts_unwrap_handles_32bit_wrap() {
        let (mut last, mut epoch) = (0u32, 0u64);
        assert_eq!(unwrap_ts(&mut last, &mut epoch, 100), 100);
        assert_eq!(unwrap_ts(&mut last, &mut epoch, 4_000_000_000), 4_000_000_000);
        // Counter wrapped: 5 < 4e9 → next epoch.
        assert_eq!(unwrap_ts(&mut last, &mut epoch, 5), (1u64 << 32) | 5);
        // Monotonic across the wrap.
        assert!(unwrap_ts(&mut last, &mut epoch, 6) > 4_000_000_000);
    }
}
