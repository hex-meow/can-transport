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

use async_trait::async_trait;
use socketcan::tokio::CanFdSocket;
use socketcan::{
    CanAnyFrame, CanDataFrame, CanFdFrame, CanRemoteFrame, EmbeddedFrame, ExtendedId, Id as ScId,
    StandardId,
};
use tokio::sync::{mpsc, Mutex};

use crate::bus::{CanBus, CanCapabilities, CanRx};
use crate::error::CanIoError;
use crate::filter::CanFilter;
use crate::frame::{CanFrame, CanId, FrameKind, MAX_DLEN};

/// Per-subscriber inbox depth. Frames overflowing this are dropped and
/// surfaced as `CanIoError::Lagged` on the next `recv` call.
const SUBSCRIBER_QUEUE: usize = 256;

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
        // CanFdSocket::write_frame takes &self; concurrent senders are fine.
        match any {
            CanAnyFrame::Normal(f) => self
                .socket
                .write_frame(&f)
                .await
                .map_err(CanIoError::backend),
            CanAnyFrame::Fd(f) => self
                .socket
                .write_frame(&f)
                .await
                .map_err(CanIoError::backend),
            CanAnyFrame::Remote(f) => self
                .socket
                .write_frame(&f)
                .await
                .map_err(CanIoError::backend),
            CanAnyFrame::Error(_) => Err(CanIoError::InvalidId),
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
            let f = CanRemoteFrame::new_remote(id, frame.dlc())
                .ok_or(CanIoError::DataTooLong {
                    got: frame.dlc(),
                    max: 8,
                })?;
            Ok(CanAnyFrame::Remote(f))
        }
    }
}
