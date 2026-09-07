# REL: Kadet-style relative plane measurement

> Historical test plan for the discarded tracker. The current implementation and
> operating procedure are documented in `README.md` and `ACCEL_REL.md`. Do not use the old
> stone-zero workflow below for the new firmware.

## Scope

REL measures the acute angle (0–90 degrees) between two contact planes, using
accelerometer/gyroscope attitude tracking. Zero may be set on a tilted plane.
The contact face must remain the same: its normal is the calibrated device Z
axis (LCD-face-up, opposite flat face resting on the surface). This does not
automatically detect or compensate for placing the enclosure on another side.
The enclosure contact surface must be parallel to the abrasive's working face.

This is not a reproduction of TSPROF's proprietary algorithm. The Kadet usage
sequence is the target: abrasive touching the blade, zero on the holder, move
the powered gauge to the reference platform, read the relative angle.

## Primary setup: modified Xarilk Gen3

The supplied development report shows a printed adapter fixed to the abrasive
mount with a steel strip on top for magnetic attachment. Use this corrected
surface, parallel to the abrasive working face, rather than the guide rod or
its bore as the measuring datum. The report attributes the former left/right
error (about 2–3 degrees on either side of center) to a skewed rod bore in the
mount. This is a mechanical datum error, not evidence for a software offset.

For the straight, flat steel-ruler setup in the report, the target is equal
readings at position 1 (left), 2 (center), and 3 (right), with the abrasive
seated consistently against the straight edge. The report's three 18.2-degree
photos describe its earlier implementation, not validation of this firmware.
For a curved blade or a contact change, equal readings are not automatically
expected: the gauge must report actual changes in the abrasive plane.

Zero once for each run. Keep the gauge attached to the corrected adapter and
measure 2 → 1 → 2 → 3 → 2, pausing for `Stable` and a further 2 seconds at each
position. Repeat in reverse order. Report all values, not only their average.
Also repeat after tilting the entire rigid setup and setting a new zero: this
checks a nonhorizontal reference while preserving the mechanical plane angle.
Do not move the whole setup after zero and expect the saved world-frame plane
to follow it.

Initial consistency target: max(left, center, right) − min(left, center, right)
≤0.3 degree, with repeatable center returns. This is a test target, not a
hardware accuracy claim. A repeatable error tied to position suggests checking
the mechanical datum and axes; an error tied to elapsed time or travel order
suggests checking tracking, settling and drift. Neither pattern alone proves
the cause. Do not introduce per-position offsets to force equal readings.

## Before testing

1. Compile/upload for the actual ESP32-S3 board settings already used by this
   device. Validation here uses `esp32:esp32:esp32s3` and LVGL 9.5.0.
2. Keep the gauge stationary during startup COD and for about 2 seconds after
   the splash disappears. Startup learns residual gyro bias over 1 second;
   another 0.75 second of stillness enables reference capture.
3. Run the existing six-face CAL if the device's mechanical calibration is
   unknown. Return to REL and wait stationary again. Entering/leaving CAL
   invalidates REL; sensor calibration and REL zero are different operations.
4. Hold the zero button for 3 seconds on a stable reference. A beep acknowledges
   successful capture, not just the button press. Pending capture times out
   after 5 seconds of valid samples without stability.

## First hardware report requested

Record displayed values only after `Stable` appears. Do not re-zero between
steps within a test. Values below are provisional engineering targets, not
verified accuracy specifications. Display resolution is 0.1 degree.

| Test | Procedure | Record / initial target |
|---|---|---|
| Flat return | Zero flat, tilt 20–40 degrees, return flat; repeat 5 times | Each return; within ±0.3 degree of zero |
| Tilted plane rotation | Zero on a fixed 20–40 degree incline, turn gauge 90/180/270 degrees in that same plane, keeping the same contact face | Each stable reading; within 0.3 degree of zero |
| Kadet transfer | Abrasive resting on blade: zero on holder; move to reference platform, then back; repeat 5 times | Platform angles and return-to-zero values; spread ≤0.3 degree |
| Known angle | Use a known wedge or a trusted gauge for comparison | Reference value, REL value, error; target ≤0.5 degree |
| Drift | Zero on a tilted plane, remain still; touch button briefly if needed to prevent sleep | Values at 0/30/60/120 seconds; note ambient warming |
| Continuity | Sleep/wake or enter/leave CAL | REL shows `--.-`, requires a new zero |

If the same-plane rotation test produces a large angle, stop accuracy testing
and report the rotation direction, approximate amount, and displayed angle.
It helps distinguish axis/sign errors from small gyro drift. Report failures
to reach `Stable` or repeated returns to `IMU settling` as well.

## Implementation and limits

- `relative_tracker.h` is a pure C++ quaternion integrator with gravity feedback.
  Startup gravity defines an arbitrary world frame; no magnetometer is assumed.
- Accelerometer calibration also supplies the orthogonal axis mapping for gyro
  data, without applying accelerometer offsets/gain to angular velocity.
- A tilted reference depends on gyro heading continuity. Gravity cannot remove
  heading drift. Software-only tests do not establish long-term accuracy.
- Sampling gaps over 100 ms, non-finite samples, near-zero acceleration, and
  near-saturation gyro samples discard the reference. UI hides REL readings
  after 250 ms without a fresh sample. Sleep explicitly discards the reference.
- Acceleration feedback is gated by gravity magnitude. Hand motion can still
  perturb attitude; pause on each surface for the first validation pass.
- CAL commands/accumulation and REL capture run on the IMU task; the UI only
  sends requests and reads snapshots. New calibration uses one `data_v3` NVS
  blob. Legacy values are read if no v3 blob exists. Save failure retains the
  previous active calibration.

## Host verification

```sh
clang++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  tests/relative_tracker_test.cpp -o /private/tmp/relative_tracker_test
/private/tmp/relative_tracker_test
```

Synthetic tests cover tilted initialization, in-plane spin, relative tilt,
mixed rotations, recapture, return to zero, constant gyro bias, continuity
loss, saturation, invalid samples and moving-zero timeout.

Official Kadet procedure (§5.12):
https://tsprof.ru/upload/iblock/5e9/jkgp083fql67ttfpny19df5yn0sjbs11/Instruktsiya_TSPROF_Kadet_Pro.pdf
