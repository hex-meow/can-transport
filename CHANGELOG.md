# Changelog

## 0.1.2 (unreleased)

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
