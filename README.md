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
| `gs_usb` | gs_usb / candleLight backend over USB (pure-Rust, no libusb). Works on **Windows, macOS, and Linux**. CAN + CAN-FD |

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

## gs_usb / candleLight backend (Windows · macOS · Linux)

For gs_usb-class adapters (candleLight firmware, e.g. the `1209:2323`
"HPM gs_usb CAN-FD" / canable / CANtact family), the `gs_usb` backend
speaks the device's USB protocol directly via [`nusb`] — no kernel CAN
stack and no libusb C dependency. This is the backend to use **off
Linux**, and on Linux when the in-kernel `gs_usb` driver is too old for
CAN-FD (e.g. Ubuntu 20.04).

```toml
[dependencies]
can-transport = { version = "0.1", features = ["gs_usb"] }
tokio = { version = "1", features = ["full"] }
```

```rust
use can_transport::{CanBus, CanFilter};
use can_transport::gs_usb::{GsUsbBus, GsUsbConfig};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // 1 Mbit nominal / 5 Mbit data, 80 MHz device clock. Matches:
    //   ip link set can0 type can bitrate 1000000 sample-point 0.8 \
    //       dbitrate 5000000 dsample-point 0.75 sjw 5 dsjw 3 fd on
    let bus = GsUsbBus::open(GsUsbConfig::fd_1m_5m()).await?;
    let mut rx = bus.subscribe(CanFilter::pass_all_standard()).await?;
    println!("{:?}", rx.recv().await?);
    Ok(())
}
```

Runnable example:

```sh
cargo run --example gs_usb_monitor --features gs_usb
```

### Bit timing

`GsUsbConfig` takes timing in the device's raw segment units (not a
target bit-rate — a general solver is a TODO). The provided presets
target an **80 MHz** device clock:

- `GsUsbConfig::fd_1m_5m()` — CAN-FD, 1 Mbit / 5 Mbit
- `GsUsbConfig::classic_1m()` — classic CAN, 1 Mbit

For other clocks/rates, set the `nominal` / `data` `GsTiming` fields
yourself. The kernel's `ip -details link show can0` prints the segment
values it computed, which you can copy verbatim.

### Per-platform driver story

| Platform | What's needed | sudo / admin? |
| -------- | ------------- | ------------- |
| **macOS** | Nothing — the OS doesn't claim vendor-specific devices | **No** |
| **Windows** | Interface bound to **WinUSB** (ship MS OS 2.0 descriptors in firmware for auto-bind, or use Zadig once) | No (once bound) |
| **Linux** | usbfs write access (root or a udev rule); the backend detaches the kernel `gs_usb` driver | Yes, unless a udev rule grants access |

A udev rule to avoid `sudo` on Linux (replace ids as needed), in
`/etc/udev/rules.d/70-gs-usb.rules`:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="2323", MODE="0660", GROUP="plugdev"
```

> **Linux + SocketCAN coexistence:** while this backend is open it
> detaches the kernel driver, so the device's `canX` SocketCAN interface
> disappears. To go back to SocketCAN, re-plug the device or rebind the
> driver, then reconfigure:
> ```sh
> echo -n '3-2:1.0' | sudo tee /sys/bus/usb/drivers/gs_usb/bind   # path from `ls /sys/bus/usb/drivers/gs_usb/`
> sudo ip link set can0 type can bitrate 1000000 sample-point 0.8 \
>     dbitrate 5000000 dsample-point 0.75 sjw 5 dsjw 3 fd on
> sudo ip link set can0 up
> ```

### Verified

Receive was validated end-to-end against real hardware (`1209:2323`,
HPMicro gs_usb CAN-FD, 80 MHz, CAN-FD bus at 1M/5M): device enumerate →
detach kernel driver → `HOST_FORMAT`/`BITTIMING`/`DATA_BITTIMING`/
`MODE_START` bring-up → bulk-IN frames parsed and delivered through
`subscribe`/`recv` (decoded a `0x701` CANopen heartbeat). The same code
path runs unchanged on macOS and Windows. Transmit is implemented but
not yet hardware-tested.

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

[`nusb`]: https://docs.rs/nusb
