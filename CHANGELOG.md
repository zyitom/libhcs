# Changelog

## 4.0.0 (unreleased)

Host-side time subsystem rewritten along host/TIME_DESIGN.md. The wire format
and the firmware code are unchanged, but the SDK API breaks, hence the major
bump. The version gate compares the whole product string, so a board flashed
before the v4.0.0 tag fails the check against a v4.0.0 host: rebuild and
re-flash every board from the tagged tree (or run with
dangerously_skip_version_checks).

### Added

- `libhcs/time/sample_time.hpp`: `SampleTime` -- the moment an uplink record
  was taken, handed to every data callback as a parameter. `host` is the
  instant on CLOCK_MONOTONIC (always present; falls back to the arrival time,
  with `Source` saying which), `board` the 64-bit, wrap-free instant on that
  board's own quarter-microsecond timer (IMU and timestamped GPIO only).
- `Handler::board_time_at(HostTime)`: the inverse mapping, a host instant ->
  this board's clock.
- Internal: `Affine<From, To>` (one composable, invertible straight line
  between two clocks), one `UsbFrameAxis` per host controller (registry keyed
  by PCI address), a per-link `BoardTimebase` (32-bit -> 64-bit board clock
  extension plus the board-clock -> axis line), and the pure conversion
  functions in `sample_timer.hpp`. Timestamps are computed on the IO thread
  at decode time; one clock read per USB transfer.

### Changed

- `DataCallback` moved to `libhcs/data/callback.hpp` (host-only; `datas.hpp`
  keeps the wire views) and every data callback now takes a `const
  libhcs::time::SampleTime&`.
- `Board<Spec>::Callback`'s typed entry points carry the same parameter.
- HCS: devices store the `SampleTime` the SDK computed; `feedback_time()` /
  `sample_time()` return it instead of converting; the per-tick axis map, the
  board-clock inbox and `board_clock_lifter.hpp` are gone.

### Removed

- `libhcs::host::time::Timeline` / `timeline()` / `Timeline::instance()`,
  `AxisMap`, the old `BoardClock` struct, `MicroframeSource`,
  `MicroframeTimebase` from the public headers: the time machinery is now an
  implementation detail (`host/src/time/`). `SofStamp` stays public
  (wire format) on `CanDataView::sof_stamp` for diagnostics.
