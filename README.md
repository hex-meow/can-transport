# can-transport

Async, cross-platform CAN / CAN-FD transport abstraction.

A tiny trait surface (`CanBus` + `CanRx`) modelling a shared bus with
filtered receive subscriptions. Designed so that higher-level CAN
protocol crates (CANopen SDO, J1939, CiA-402, …) can be written once
and run on any platform that has a backend implementation.

## Features

| Feature | What you get |
| ------- | ------------ |
| (none)  | Just the traits + types (`CanFrame`, `CanFilter`, `CanIoError`) |
| `socketcan` | Linux SocketCAN backend supporting both classic CAN and CAN-FD |

## Quick start (SocketCAN)

```toml
[dependencies]
can-transport = { version = "0.1", features = ["socketcan"] }
tokio = { version = "1", features = ["full"] }
```

```rust
use can_transport::{CanBus, CanFilter, CanFrame};
use can_transport::socketcan::SocketCanBus;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let bus = SocketCanBus::open("can0")?;

    // Subscribe to one specific id.
    let mut rx = bus.subscribe(CanFilter::exact_standard(0x123)).await?;

    // Send a classic CAN frame...
    bus.send(CanFrame::new_data(0x456u16, &[1, 2, 3, 4])?).await?;

    // ...and a CAN-FD frame with bit-rate switch.
    bus.send(CanFrame::new_fd(0x789u16, &[0xAA; 32], /*brs=*/ true)?).await?;

    // Receive matching traffic.
    let frame = rx.recv().await?;
    println!("got: {:?}", frame);
    Ok(())
}
```

## Writing your own backend

Implement `CanBus` (and return a `CanRx` from `subscribe`) — usually
about 100 lines. See `src/socketcan.rs` for a reference implementation.

The trait makes only two assumptions:
1. The bus is **shared** (`&self` methods, multiple senders OK).
2. The bus fans out incoming frames to all subscribers whose filter
   matches; slow subscribers may report `CanIoError::Lagged` but must
   not block other subscribers.

## License

MIT OR Apache-2.0
