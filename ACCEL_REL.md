# Accelerometer-only REL (main-based)

## Current revision: contact-normal locked at zero

The active estimator is `accel_rel::ContactRel`. Hold the existing zero button
for 3 seconds near horizontal. The largest absolute calibrated acceleration
component selects contact normal X/Y/Z **once**, at capture. Keep the same device
contact face for all following measurements. Re-zero is the only operation that
selects another normal. No knife/stone mode or gyro is required.

The reference's gravity projection onto the contact plane supplies a fixed
in-plane sign direction. For current normal inclination `theta` and saved
inclination `theta0`, project current gravity onto that direction to choose
`+theta` or `-theta`. Display the acute form of `abs(signed_theta-theta0)`.
This handles opposite-side tilts without observing a horizontal crossing, on
flat, side, and end-face mounts. At exactly horizontal zero the branch has no
effect and yaw does not affect the ideal result. A tilted reference with changed
heading still has unobservable geometry error and a branch boundary. This is
NOT a generally heading-invariant 3D plane-angle estimator.

Both reference and target may be knife or stone. The reference is a snapshot:
its physical plane must stay fixed after zero. No previous knife-specific
asymmetric blend is applied: its beta interpretation is invalid for an
unlabelled stone reference. Prior simulation reports describe prior candidates,
not a measured error guarantee for this revision.

Only the relative scalar output is low-pass filtered (tau 0.15s), avoiding
vector-averaging artifacts during in-plane rotation. Capture stores the latest
valid calibrated sample and resets the displayed relative angle to zero.
Missing/nonfinite samples retain the saved face/zero and recover automatically;
calibration clears zero. Red/white is a settling hint and does not gate readings.
Debug UI shows `N:X/Y/Z locked`; serial prefix is `REL_CONTACT`, with `normal=0/1/2`.

`tests/contact_rel_test.cpp` covers 23,808 bidirectional central poses with all
three normal axes, both face polarities and four common in-plane mount headings;
18,360 additional 3D samples verify equivalent results across mounting faces
(not an arbitrary-yaw accuracy claim). It also checks horizontal-reference yaw at 65 degrees (no axis reselection),
re-zero, invalid recovery, scale invariance and scalar response. ASan/UBSan passed.
The original six-face calibration, its storage, IMU configuration and library
internals are unchanged by this revision. Hardware verification is still needed.

Build target: `esp32:esp32:waveshare_esp32_s3_touch_lcd_169:PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled`.
Build passed: program 1,281,539 / 3,145,728 bytes (40%); static RAM
127,936 / 327,680 bytes (39%). Toolchain GNU-stack linker warning remains.
Uploaded 2026-09-07 via `/dev/cu.usbmodem143301`; esptool verified hashes and
reset the ESP32-S3. No full-flash erase was requested; NVS was not in the listed
erase regions. `REL_CONTACT` output confirms the new firmware is running.

Initial hardware check: X end-face zero captured successfully (`normal=0`).
24 logged stationary samples after capture: relative 0.0310–0.0697 degrees,
mean 0.0511. During the user's tilt/return, all logged samples retained X and
were valid; moving samples continued reporting angles. The subsequent 31
stationary-labelled samples read 0.1444–0.2362 degrees (mean 0.1937). This is
an observed return offset, not proven absolute error or sensor drift: exact
reseating/orientation was not independently measured. Known-angle and repeat
placement checks remain pending. Logs: `simulation/hardware_logs/stationary-60s-20260907-234422.jsonl`
and `simulation/hardware_logs/stationary-30s-20260907-234529.jsonl`.

Hardware acceptance after upload:
1. On a near-horizontal reference, hold zero for 3 seconds using the end face.
   Confirm `N:X locked` when sensor X is the contact normal; same plane reads 0.
2. Keeping that same contact face, use a known 10–25 degree wedge. Return to
   reference and confirm zero return. Repeat flat/side mount, re-zeroing each time.
3. Tilt beyond 45 degrees: `N` must remain unchanged and readings must continue.
4. With a known -10 to +10 degree aligned fixture, jump directly between poses;
   settled relative reading should be 20 degrees, without tracking the crossing.
5. Verify red while disturbed and white when settled; the hint must never stop
   the displayed angle. Tilted-plane yaw accuracy is not an acceptance guarantee.

## Historical revision: X-signed absolute inclination

X-signed revision built and uploaded successfully: 1,281,027 bytes program / 127,888 bytes static RAM. ASan/UBSan unit tests and 74,898-case comparison passed; ideal errors match main within numeric tolerance for the tested face-up poses (including remaining curved-edge and yaw-sign errors). Snapshot: `simulation/X_SIGNED_VS_MAIN_RESULTS.txt`. Boot confirmed via `REL_ACC_X` log `stationary-15s-20260907-223754.jsonl`; physical opposite-side zero verification remains to be done.

User selected fixed sensor X orientation. Active calculation is now:

`theta = (ax >= 0 ? +1 : -1) * atan2(hypot(ax,ay), abs(az))`

`REL = abs(filtered_theta - reference_theta)`

This is signed **total inclination**, not X-only pitch `atan2(ax,...)`. −15° reference to +15° yields 30°; −15° to −10° yields 5°. The current sample supplies the sign, so no observed horizon crossing or gyro history is required. Scalar low-pass filtering remains, with transient lag but no accumulated angle integration. Same-plane yaw invariance is intentionally no longer claimed: reversing sensor X direction flips the branch, and ax≈0 with appreciable ay can cause sign instability. Keep the X direction consistent between reference and measurement. Curved-blade geometry errors are not eliminated.

Original 3-second zero UI, accelerometer-only settings, red/white settling hint and calibration preservation remain. Diagnostic prefix is `REL_ACC_X`. Earlier unsigned simulation results below apply to the previous revision, not this one. The comparison test now follows the active Filter; its saved unsigned report remains a historical snapshot.

---

## Historical unsigned revision

> Simulation warning: `simulation/ACCEL_VS_MAIN.md` finds up to 30° actual-angle error in the previous tilted-clamp range. This firmware is an inclination-difference meter, not a generally accurate sharpening-angle meter. Same-plane yaw invariance does not imply relative-plane angle accuracy.

## Scope

Restored the main (`2f9d288`) large 3-second zero button, immediate capture/beep, and ordinary continuous REL flow. Removed the gyro tracker from the active firmware and disabled the gyroscope, including startup COD. Retained app-side LVGL 8/9 compatibility, existing v2/v3 acceleration-calibration persistence compatibility, and nonblocking USB diagnostics. The six-face calibration sequence is unchanged. Prior gyro experiment headers/tests/logs remain as historical material, not the active measurement algorithm.

## Design and limitation

For calibrated acceleration `(ax,ay,az)`:

`inclination = atan2(hypot(ax,ay), abs(az))`

`REL = abs(filtered_inclination - saved_reference_inclination)`

The accelerometer magnitude alone `sqrt(ax²+ay²+az²)` is approximately 1g at rest and cannot encode angle. Here “absolute acceleration angle” means unsigned plane inclination derived from the calibrated gravity components. The result is an inclination difference, not a general 3D angle between arbitrary planes.

Why this design: it is exactly invariant to rotations of the instrument within the same contact plane for ideal calibrated samples; unlike a signed-X tilt, it does not flip at X=0. Filtering the scalar angle rather than a rotating acceleration vector also avoids yaw-dependent vector averaging artifacts. No gyro integration, motion buffer, stationary gate, or tracking-lost timeout exists. A missing/nonfinite sensor reading is still distinct from a valid value.

There is no universally minimum-error relative-plane estimator from these unsigned inclinations alone. Unknown relative heading leaves true acute plane angle in `[abs(theta-theta0), min(theta+theta0,90°)]`. We choose the lower endpoint for same-plane zero and heading invariance, NOT the midpoint/minimax estimator. A midpoint would fail the essential same-plane-zero requirement. This explicitly favors the user's stable old UX over claiming unobservable angle accuracy.

- If either reference or target plane is horizontal, inclination difference equals true acute plane angle.
- If both tilt along the same direction, the difference is exact.
- If the two tilts are on opposite sides, the true angle can be their sum: reference 10°, target 15° can mean 25° while this display is 5°.
- On curved/tanto edges, changing heading between tilted planes can cause systematic underestimation. No accelerometer-only scalar correction can universally remove it without external geometry/heading information.
- Slow translation and acceleration can disturb readings. Flat-plane spin is ideally invariant, but off-axis spinning adds centripetal acceleration; actual sensor calibration, mounting flex, and contact geometry also matter.

## Settings and UX

- Accelerometer ±2g, accel-only ODR 125Hz, LPF mode 00 (~3.3Hz). Gyro disabled; no COD waiting.
- Scalar exponential filter time constant 0.15s. This reduces noise; it cannot correct a sensor's systematic calibration error.
- Hold the existing button 3s while on the intended reference plane; immediate capture from filtered inclination and beep as in main. No second confirmation or angular-pose rejection.
- Continue showing angles while moving. Red indicates acceleration-norm departure >0.05g or instantaneous inclination differs from filtered inclination by >0.15°. White otherwise. This is an acceleration-only settling hint, not proof of physical immobility; in-plane yaw may remain white by design.
- Stale samples (>500ms) show `--`; renewed samples resume automatically with the saved zero. Calibration invalidates the old zero.
- Diagnostics: nonblocking `REL_ACC ... gyro=off` at ~1Hz.

## Verification

`tests/accel_rel_test.cpp`: 21,960 tilt/yaw combinations, gain-scale invariance, 100-second in-plane yaw sweep, 20° step response, invalid-input recovery. ASan/UBSan passed. Synthetic 20° step error is below 0.4° after 0.6 seconds (sensor LPF delay is not included). These tests validate computation and invariance, not real-world absolute accuracy.

Older `STATIONARY_REL.md` and `IMU_TUNING.md` describe superseded gyro implementations. Do not use their accuracy claims or operating procedure for this build.

Build and upload succeeded on the existing Waveshare 1.69 target: program 1,281,015 bytes (40%), static RAM 127,888 bytes (39%). Upload hashes verified, no full flash erase. Boot trace `simulation/hardware_logs/stationary-15s-20260907-222337.jsonl` confirms continuous `REL_ACC` output with valid acceleration readings and gyro disabled in the configured application. Physical in-plane rotation and zero-return verification remain user tests; synthetic invariance is not proof of the actual sensor's calibration accuracy.
