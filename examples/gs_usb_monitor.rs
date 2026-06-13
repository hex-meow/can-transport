//! Receive frames from a gs_usb / candleLight device.
//!
//! Run with the device's kernel driver detached (Linux) or on Windows/macOS:
//!
//! ```sh
//! cargo run --example gs_usb_monitor --features gs_usb
//! ```

use std::time::Duration;

use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};
use can_transport::{CanBus, CanFilter};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // Optional channel index for multi-channel adapters (default 0).
    let channel: u16 = std::env::args()
        .nth(1)
        .map(|s| s.parse())
        .transpose()?
        .unwrap_or(0);

    println!("opening gs_usb device channel {channel} (CAN-FD, 1M/5M)...");
    let bus = GsUsbBus::open(GsUsbConfig::fd_1m_5m().with_channel(channel)).await?;
    println!("opened: {:?}", bus.capabilities());

    let mut rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
    let mut rx_ext = bus.subscribe(CanFilter::pass_all_extended()).await?;

    println!("listening for frames (Ctrl-C to stop)...");
    let mut count = 0u64;
    loop {
        let frame = tokio::select! {
            f = rx.recv() => f,
            f = rx_ext.recv() => f,
            _ = tokio::time::sleep(Duration::from_secs(5)) => {
                println!("(no frames in 5s)");
                continue;
            }
        };
        match frame {
            Ok(f) => {
                count += 1;
                println!(
                    "[{count}] id={:?} kind={:?} dlc={} data={:02X?}",
                    f.id(),
                    f.kind(),
                    f.dlc(),
                    f.data()
                );
            }
            Err(e) => {
                eprintln!("recv error: {e}");
                break;
            }
        }
    }
    Ok(())
}
