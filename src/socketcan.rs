//! Linux SocketCAN backend (CAN 2.0 and CAN-FD).
//!
//! Enable with the `socketcan` feature.
//!
//! ```no_run
//! # #[cfg(feature = "socketcan")]
//! # async fn _doc() -> Result<(), can_transport::CanIoError> {
//! use can_transport::{CanBus, CanFilter, CanFrame};
//! use can_transport::socketcan::SocketCanBus;
//!
//! let bus = SocketCanBus::open("can0")?;
//! let mut rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
//! bus.send(CanFrame::new_data(0x123u16, &[1, 2, 3])?).await?;
//! let frame = rx.recv().await?;
//! println!("got: {:?}", frame);
//! # Ok(()) }
//! ```

use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use socketcan::tokio::CanFdSocket;
use socketcan::{
    CanAnyFrame, CanDataFrame, CanFdFrame, CanRemoteFrame, EmbeddedFrame, ExtendedId, Id as ScId,
    StandardId,
};
use tokio::sync::{mpsc, Mutex};

use crate::bus::{
    CanBitTiming, CanBus, CanBusState, CanCapabilities, CanControllerState, CanLinkConfig, CanRx,
};
use crate::error::CanIoError;
use crate::filter::CanFilter;
use crate::frame::{CanFrame, CanId, FrameKind, MAX_DLEN};

/// Per-subscriber inbox depth. Frames overflowing this are dropped and
/// surfaced as `CanIoError::Lagged` on the next `recv` call.
const SUBSCRIBER_QUEUE: usize = 256;

/// How long to wait between attempts once the transmit queue reports full.
const TX_BACKPRESSURE_POLL: Duration = Duration::from_millis(1);
/// How long a full transmit queue is treated as back-pressure rather than a
/// fault. A CAN device that has not drained a single frame within this long is
/// not merely busy.
const TX_BACKPRESSURE_LIMIT: Duration = Duration::from_millis(250);

/// A full transmit queue reports `ENOBUFS`, which is back-pressure, not failure.
///
/// Write readiness tracks the socket send buffer, not the device queue, so the
/// socket polls writable while the qdisc is full and tokio's readiness loop
/// never retries: it only re-arms on `EWOULDBLOCK`. A caller streaming a
/// multi-frame payload would see a hard error partway through a frame it had
/// already half-transmitted. CAN transmit queues are shallow (`txqueuelen` is 10
/// by default) and drain at bus rate, so waiting is almost always correct.
fn transmit_queue_full(error: &std::io::Error) -> bool {
    error.raw_os_error() == Some(libc::ENOBUFS)
}

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

/// SocketCAN-backed [`CanBus`]. Wrap in `Arc` to share.
pub struct SocketCanBus {
    socket: Arc<CanFdSocket>,
    registry: Arc<Registry>,
    reader: tokio::task::JoinHandle<()>,
    iface: String,
}

impl SocketCanBus {
    /// Open a SocketCAN interface (e.g. `"can0"`, `"vcan0"`).
    /// Spawns one background task that fans incoming frames out to
    /// subscribers; the task is aborted when this `SocketCanBus` is dropped.
    pub fn open(iface: &str) -> Result<Self, CanIoError> {
        let socket = CanFdSocket::open(iface).map_err(CanIoError::backend)?;
        let socket = Arc::new(socket);
        let registry = Arc::new(Registry::new());

        let reader = tokio::spawn(reader_task(socket.clone(), registry.clone()));

        Ok(Self {
            socket,
            registry,
            reader,
            iface: iface.to_string(),
        })
    }

    pub fn interface(&self) -> &str {
        &self.iface
    }
}

impl Drop for SocketCanBus {
    fn drop(&mut self) {
        self.reader.abort();
    }
}

async fn reader_task(socket: Arc<CanFdSocket>, registry: Arc<Registry>) {
    loop {
        let frame = match socket.read_frame().await {
            Ok(f) => f,
            Err(e) => {
                log::warn!("socketcan read error: {e}; reader exiting");
                return;
            }
        };
        let Some(frame) = sc_to_canframe(&frame) else {
            continue;
        };
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
                Err(mpsc::error::TrySendError::Closed(_)) => {
                    // Subscription dropped; the slot will be cleaned up
                    // by the subscriber's Drop.
                }
            }
        }
    }
}

#[async_trait]
impl CanBus for SocketCanBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        let any = canframe_to_sc(&frame)?;
        if matches!(any, CanAnyFrame::Error(_)) {
            return Err(CanIoError::InvalidId);
        }
        // CanFdSocket::write_frame takes &self; concurrent senders are fine.
        let started = Instant::now();
        loop {
            let result = match &any {
                CanAnyFrame::Normal(f) => self.socket.write_frame(f).await,
                CanAnyFrame::Fd(f) => self.socket.write_frame(f).await,
                CanAnyFrame::Remote(f) => self.socket.write_frame(f).await,
                CanAnyFrame::Error(_) => unreachable!("rejected before the send loop"),
            };
            match result {
                Ok(()) => return Ok(()),
                Err(error)
                    if transmit_queue_full(&error)
                        && started.elapsed() < TX_BACKPRESSURE_LIMIT =>
                {
                    tokio::time::sleep(TX_BACKPRESSURE_POLL).await;
                }
                Err(error) => return Err(CanIoError::backend(error)),
            }
        }
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
        Ok(Box::new(SocketCanRx {
            rx,
            id,
            registry: self.registry.clone(),
            dropped,
        }))
    }

    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: true,
            max_dlen: MAX_DLEN,
        }
    }

    async fn bus_state(&self) -> Result<Option<CanBusState>, CanIoError> {
        let iface = self.iface.clone();
        // Netlink queries are blocking syscalls; keep them off the reactor.
        let state = tokio::task::spawn_blocking(move || {
            let nl = ::socketcan::nl::CanInterface::open(&iface).map_err(CanIoError::backend)?;
            // Both are Ok(None) when the netdev exposes no CAN netlink params
            // (e.g. vcan or a non-CAN interface) — surface that as unknown. A
            // down *physical* CAN device reports Some(Stopped) instead.
            let state = nl.state().map_err(CanIoError::backend)?;
            let berr = nl.berr_counter().map_err(CanIoError::backend)?;
            Ok::<CanBusState, CanIoError>(CanBusState {
                state: state.map(|s| match s {
                    ::socketcan::nl::CanState::ErrorActive => CanControllerState::ErrorActive,
                    ::socketcan::nl::CanState::ErrorWarning => CanControllerState::ErrorWarning,
                    ::socketcan::nl::CanState::ErrorPassive => CanControllerState::ErrorPassive,
                    ::socketcan::nl::CanState::BusOff => CanControllerState::BusOff,
                    ::socketcan::nl::CanState::Stopped => CanControllerState::Stopped,
                    ::socketcan::nl::CanState::Sleeping => CanControllerState::Sleeping,
                }),
                tx_errors: berr.map(|b| b.txerr),
                rx_errors: berr.map(|b| b.rxerr),
            })
        })
        .await
        .map_err(CanIoError::backend)??;
        Ok(Some(state))
    }

    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        let iface = self.iface.clone();
        // Netlink queries are blocking syscalls; keep them off the reactor.
        // This only reads the current link state and never changes the
        // interface configuration.
        let config = tokio::task::spawn_blocking(move || {
            let nl = ::socketcan::nl::CanInterface::open(&iface).map_err(CanIoError::backend)?;
            let details = nl.details().map_err(CanIoError::backend)?;

            let fd_enabled = details
                .can
                .ctrl_mode
                .map(|modes| modes.has_mode(::socketcan::CanCtrlMode::Fd))
                // vcan and some drivers expose the MTU but no CAN ctrl-mode
                // netlink attribute.  The MTU is the next-best observation.
                .or_else(|| details.mtu.map(|mtu| mtu == ::socketcan::nl::Mtu::Fd));

            Ok::<CanLinkConfig, CanIoError>(CanLinkConfig {
                fd_enabled,
                nominal: details.can.bit_timing.map(netlink_bit_timing),
                data: details.can.data_bit_timing.map(netlink_bit_timing),
            })
        })
        .await
        .map_err(CanIoError::backend)??;

        Ok(Some(config))
    }
}

fn netlink_bit_timing(timing: ::socketcan::nl::CanBitTiming) -> CanBitTiming {
    CanBitTiming {
        bitrate: (timing.bitrate != 0).then_some(timing.bitrate),
        sample_point_per_mille: u16::try_from(timing.sample_point)
            .ok()
            .filter(|point| (1..=1000).contains(point)),
    }
}

struct SocketCanRx {
    rx: mpsc::Receiver<CanFrame>,
    id: SubId,
    registry: Arc<Registry>,
    dropped: Arc<AtomicU64>,
}

impl Drop for SocketCanRx {
    fn drop(&mut self) {
        // Best-effort: blocking lock attempt in async context isn't ideal, but
        // unsubscribing is cheap. We try to acquire without blocking the
        // runtime; if that fails, the entry just sticks around until the next
        // dispatcher pass sees `Closed` and ignores it.
        let registry = self.registry.clone();
        let id = self.id;
        if let Ok(handle) = tokio::runtime::Handle::try_current() {
            handle.spawn(async move {
                let mut subs = registry.subs.lock().await;
                subs.remove(&id);
            });
        }
    }
}

#[async_trait]
impl CanRx for SocketCanRx {
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

// ---------- conversion helpers ----------

fn sc_to_canframe(frame: &CanAnyFrame) -> Option<CanFrame> {
    let id = match frame.id() {
        ScId::Standard(s) => CanId::Standard(s.as_raw()),
        ScId::Extended(e) => CanId::Extended(e.as_raw()),
    };
    match frame {
        CanAnyFrame::Normal(f) => CanFrame::new_data(id, f.data()).ok(),
        CanAnyFrame::Fd(f) => CanFrame::new_fd(id, f.data(), f.is_brs()).ok(),
        CanAnyFrame::Remote(f) => CanFrame::new_remote(id, f.dlc() as u8).ok(),
        CanAnyFrame::Error(e) => {
            log::debug!("CAN error frame ignored: {e:?}");
            None
        }
    }
}

fn canframe_to_sc(frame: &CanFrame) -> Result<CanAnyFrame, CanIoError> {
    let id: ScId = match frame.id() {
        CanId::Standard(s) => StandardId::new(s).ok_or(CanIoError::InvalidId)?.into(),
        CanId::Extended(e) => ExtendedId::new(e).ok_or(CanIoError::InvalidId)?.into(),
    };
    match frame.kind() {
        FrameKind::Data => {
            let f = CanDataFrame::new(id, frame.data()).ok_or(CanIoError::DataTooLong {
                got: frame.data().len(),
                max: 8,
            })?;
            Ok(CanAnyFrame::Normal(f))
        }
        FrameKind::Fd { brs } => {
            let mut f = CanFdFrame::new(id, frame.data()).ok_or(CanIoError::DataTooLong {
                got: frame.data().len(),
                max: MAX_DLEN,
            })?;
            f.set_brs(brs);
            Ok(CanAnyFrame::Fd(f))
        }
        FrameKind::Remote => {
            let f = CanRemoteFrame::new_remote(id, frame.dlc()).ok_or(CanIoError::DataTooLong {
                got: frame.dlc(),
                max: 8,
            })?;
            Ok(CanAnyFrame::Remote(f))
        }
    }
}
