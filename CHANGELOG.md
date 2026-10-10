# Changelog

## 4.0.0 (unreleased)

Host-side rewrite along HCS-USB.md (v4): the outer API now follows the same
conventions as hcs-ecat (explicit lifecycle, pull-based channels, exception-free
RT paths, runtime port declaration by name), and the transport becomes a
strategy interface with a fake implementation. The wire format and the firmware
code are unchanged, but the SDK API breaks, hence the major bump. The version
gate compares the whole product string, so a board flashed before the v4.0.0 tag
fails the check against a v4.0.0 host: rebuild and re-flash every board from the
tagged tree (or run with dangerously_skip_version_checks).

### Added

- `libhcs/board.hpp`: `libhcs::Board` -- one board as a runtime object (pimpl,
  no template). `BoardOptions` names the board type ("hpm5321" / "mc02" /
  "c_board" / "hpm6e8y"), serial filter, thread placement and channel
  capacities; the second constructor parameter injects a `Transport` (tests
  inject `FakeTransport`, default is libusb).
- Declaration before `start()`: `add_can(name, CanSetting)`,
  `add_uart(name, UartSetting)`, `add_imu(ImuSetting)` -- the name is the
  silkscreen one ("CAN1", "UART7", "DBUS"), looked up in the board type's
  port table; an unknown name throws `PortNotFoundError` listing what the
  board has.
- Lifecycle (HCS-USB.md 2.2): `start()` opens the device, checks versions and
  capabilities, applies the EP0 manifest and verifies by read-back (throws
  with the difference on failure); `run()` / `arm()` gate the transmit path;
  `stop()` is idempotent. `BoardState` distinguishes Idle / Armed (receiving,
  safe frames only) / Running; open and configure are steps inside start().
- Pull-based channels (`libhcs/channel.hpp`): the IO thread feeds a
  drop-oldest SPSC ring per channel with each record's `SampleTime`;
  `board.receive()` snapshots the tick, `frames()` / `chunks()` / `samples()`
  read it; `stage()` holds outgoing records (noexcept, false + counter when
  full or invalid); `board.send()` commits and rings the IO-thread doorbell,
  which packs all ports' records into as few USB transfers as it can.
- Safety: `CanChannel::set_safe_frames` / `UartChannel::set_safe_bytes`
  declare the disable output; `request_safe(bool)` from any thread, session
  loss and the Armed state restrict transmits to it (the IO thread also sends
  one round immediately on each of those edges -- the stand-in until the
  board-side watchdog, U3, lands).
- `libhcs/transport.hpp`: the public `Transport` strategy interface; the
  existing libusb implementation sits behind it (`host/src/transport/usb`),
  and `libhcs/testing/fake_transport.hpp` plays a whole board in memory for
  tests (injects uplink records, captures downlink, simulates disconnect and
  removal, answers the session and time-base protocols synchronously).
- `libhcs/status.hpp`: `LinkState` (unchanged semantics), `BoardState`,
  `Statistics` (monotonic counters plus a log2 keepalive-RTT histogram,
  shaped like hcs-ecat's) and per-channel counters alongside the board-pushed
  port status.
- `host/tests/board_v4_test.cpp`: 26 end-to-end tests over FakeTransport --
  declaration errors, manifest/version/capability checks at start, ticked
  receive (multi-frame, cross-tick, drop-oldest overflow accounting),
  stage-full, safe frames in every gated mode, disconnect -> kSessionDown ->
  recovery, removal -> kFaulted, stop/restart, and timestamp conversion
  through the shared axis.

### Changed

- `libhcs/time/sample_time.hpp`: `SampleTime` -- the moment an uplink record
  was taken, carried with every record into the channels. `host` is the
  instant on CLOCK_MONOTONIC (always present; falls back to the arrival time,
  with `Source` saying which), `board` the 64-bit, wrap-free instant on that
  board's own quarter-microsecond timer (IMU and timestamped GPIO only).
- The link engine (v3's `Handler`) moved to `host/src/session/engine.cpp` and
  now runs the IO thread: session keepalive, time anchors and the doorbell-
  driven transmit commit all live on it; statistics (RTT histogram, session
  losses, uplink stream errors) accumulate there.
- IO-thread logging is counters plus occurrence-throttled lines only
  (1st/2nd/4th/...); new hot paths (ring overflow, stage overflow, safe-frame
  injection) count and never log.

### Removed

- `Board<Spec>` and the per-board classes (`Hpm5321`, `Mc02`, `CBoard`,
  `Hpm6e8y`): superseded by the runtime `libhcs::Board`.
- The push-based `DataCallback` interface and `PacketBuilder` from the public
  headers: superseded by the pull-based channels. The old HCS stays on the
  frozen v3 tag.
- `libhcs::host::time::Timeline` / `timeline()` / `Timeline::instance()`,
  `AxisMap`, the old `BoardClock` struct, `MicroframeSource`,
  `MicroframeTimebase` from the public headers: the time machinery is now an
  implementation detail (`host/src/time/`). `SofStamp` stays public
  (wire format) on `CanDataView::sof_stamp` for diagnostics.

### Deferred (not in 4.0.0)

- GPIO and buzzer channels, run-time redeclaration (`configure_*`), the
  board-side watchdog (U3; no `kBoardCapWatchdog` in core yet -- the manifest
  carries no timeout, the host-side safe-frame resend is the stand-in), the
  `OwnThread` clock mode, and `board_time_at` / pulse scheduling on the public
  Board.
