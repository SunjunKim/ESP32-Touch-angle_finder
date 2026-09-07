# Changelog

## 0.0.2 — 2026-09-08

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
