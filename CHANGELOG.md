# Changelog

## 0.1.4

### Fixed
- SocketCAN: a full transmit queue no longer fails the send. The kernel reports
  a full qdisc as `ENOBUFS`, and tokio's readiness loop does not retry it — it
  re-arms only on `EWOULDBLOCK`, and write readiness tracks the socket send
  buffer rather than the device queue, so the socket polls writable while the
  queue is full. A caller streaming a multi-frame payload saw a hard error
  partway through a payload it had already half-transmitted. `send` now polls
  every 1 ms for up to 250 ms before giving up. CAN transmit queues are shallow
  (`txqueuelen` defaults to 10, and gs_usb adds only ~10 in-flight URBs) and
  drain at bus rate, so a full queue is back-pressure rather than a fault.

### Changed
- `repository` now points at `hex-meow/can-transport`. The previous `hexmecha`
  URL survives only as a GitHub rename redirect.

### Added
- `LICENSE-MIT` and `LICENSE-APACHE` files, matching the `license` field that
  earlier releases already declared.

## 0.1.3

### Added
- `CanBus::link_config()` — best-effort link timing snapshot (`CanLinkConfig`
  with `fd_enabled` plus nominal and data `CanBitTiming`). SocketCAN reads it
  over netlink; the default implementation returns `Ok(None)`.

## 0.1.2

Additive API, no behavior change for existing code paths.

### Added
- `CanFrame::timestamp_us()` / `CanFrame::with_timestamp_us()` — optional
  backend-provided receive timestamp (µs, backend-defined epoch, only deltas
  are meaningful). Constructed frames carry `None`. **Note:** `PartialEq` for
  `CanFrame` now compares frame *content only* (id, kind, payload) and
  deliberately ignores the timestamp, so constructed frames still compare
  equal to the same frame received off the wire.
- `CanBus::bus_state()` — best-effort controller health snapshot
  (`CanBusState { state, tx_errors, rx_errors }`, new `CanControllerState`
  enum). Default implementation returns `Ok(None)` ("not supported"), so
  existing and minimal (e.g. MCU) implementations need no code. Wrapper
  implementations should forward it explicitly.
  - SocketCAN: implemented via netlink (`CanInterface::state()` +
    `berr_counter()`); `vcan`/non-CAN netdevs report `None` fields.
  - gs_usb: implemented via `GS_USB_BREQ_GET_STATE`, gated on the device
    advertising `GS_CAN_FEATURE_GET_STATE`; returns `Ok(None)` otherwise.
- gs_usb hardware timestamps: `GsUsbConfig::with_hw_timestamp(true)` requests
  device-clock stamping (`GS_CAN_MODE_HW_TIMESTAMP`, gated on the feature
  bit; falls back with a warning if unsupported). The 32-bit device counter
  is unwrapped to a monotonic `u64` across its ~71.6-minute wrap.
  `GsUsbBus::hw_timestamps_active()` / `device_features()` report the result.

### Compatibility note
- `GsUsbConfig` gained a `pub hw_timestamp: bool` field. If you construct it
  with a struct literal (rather than the `classic_1m()` / `fd_1m_5m()`
  presets + `with_*` builders), add the new field. All known consumers use
  the builders; prefer them going forward — the struct may gain fields as
  gs_usb features are surfaced.
