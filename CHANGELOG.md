# Changelog

## 0.0.2 — 2026-11-14 (GitHub release)

- Publish standalone Windows x64 and macOS arm64/x64 updater packages using the
  product's existing v0.0.2 version; include source snapshots and license notices.
- Add download links and USB update instructions for existing device owners.
- Handle missing USB devices cleanly with the macOS system Bash updater.
- License project code and documentation under GPL-3.0-only; preserve third-party notices.
- Split REL into 1D (existing ABS pitch difference) and 2D (previous REL algorithm).
- Boot in 2D; cycle 2D → 1D → ABS → CAL.
- Share the stationary zero reference across both modes; maintain separate output
  filters and settling hints without resetting on mode changes.
- Preserve the 1-second zero hold, six-face calibration and calibration storage.
- Prepare the repository for GitHub: organize documentation and reference assets
  under `docs/`, document dependency installation and releases, and add host-test CI.
- Include board pin definitions locally so source builds no longer require Mylibrary.
- Add Roboto Mono attribution and license; ignore generated firmware and local files.

## 0.0.2 — 2026-09-08 (local development build)

- REL: accelerometer-only estimate; contact normal selected at zero and locked.
- Preserve three-second zero interaction; show moving readings in red and
  settled readings in white. Filter the scalar output without gyro integration.
- Disable gyro startup calibration and enable optional nonblocking USB diagnostics
  with `AXICUBE_REL_DIAGNOSTICS=1` (off by default).
- Support LVGL 8/9 display/input APIs and local LVGL configuration.
- Preserve six-face calibration workflow and legacy NVS records; retain
  compatibility with diagnostic firmware's validated `data_v3` calibration blob.
  No new calibration coefficients or five-face diagnostic estimates are applied.
- Archive gyro experiments, simulation comparisons and raw hardware logs;
  document current operating constraints and reproducible host tests.

Known limits: inclined-reference heading ambiguity remains. Recorded 45° jig
readings were 46.7° and 45.8° on two mounts; these are observations, not accuracy
specifications. Final production firmware still requires device smoke testing.
