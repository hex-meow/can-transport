//! Dump every frame on a SocketCAN interface.
//!
//! Run with:
//! ```text
//! cargo run --example monitor --features socketcan -- can0
//! ```

use can_transport::socketcan::SocketCanBus;
use can_transport::{CanBus, CanFilter};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args().nth(1).unwrap_or_else(|| "can0".to_string());
    println!("Opening {iface} (CAN-FD capable)");
    let bus = SocketCanBus::open(&iface)?;

    let mut std_rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
    let mut ext_rx = bus.subscribe(CanFilter::pass_all_extended()).await?;

    tokio::spawn(async move {
        loop {
            match ext_rx.recv().await {
                Ok(f) => println!("[ext] {:08X?}  {:02X?}", f.id().raw(), f.data()),
                Err(e) => {
                    eprintln!("ext rx error: {e}");
                    break;
                }
            }
        }
    });

    loop {
        match std_rx.recv().await {
            Ok(f) => println!(
                "[std] {:03X}  {:02X?}  (fd={}, brs={})",
                f.id().raw(),
                f.data(),
                f.is_fd(),
                f.brs()
            ),
            Err(e) => {
                eprintln!("std rx error: {e}");
                break;
            }
        }
    }
    Ok(())
}
