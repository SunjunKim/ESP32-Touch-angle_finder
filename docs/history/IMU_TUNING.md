# QMI8658 stationary REL tuning — 2026-09-07

> Superseded by the accelerometer-only implementation in ../ACCEL_REL.md. Gyro settings below are historical, not active firmware settings.

## Sources and chip identity

- [Waveshare hardware reference](https://raw.githubusercontent.com/waveshareteam/ESP32-S3-Touch-LCD-1.69/main/HARDWARE_REFERENCE.md): ESP32-S3, QMI8658C motion sensor, ST7789V2 display, CST816T touch, PCF85063 RTC. The latter chips do not determine inertial LPF bandwidth; their drivers are unchanged.
- [QST QMI8658C datasheet Rev A](https://www.qstcorp.com/upload/pdf/202210/13-52-27%20QMI8658C%20Datasheet%20Rev%20A%20%281%29.pdf), Tables 11–13 and CTRL2/3/5. Tables 11/13 were visually checked from the PDF. Waveshare also links an A-variant PDF; use C specifications for this board reference. WHOAMI/revision readback is logged but does not independently prove the device suffix on this physical board.
- Installed SensorLib 0.4.1 `SensorQMI8658.hpp` configuration methods were inspected. Libraries were not edited.

## Failure observed

`simulation/hardware_logs/stationary-60s-20260907-220538.jsonl`: no sample gaps, but all readings were tracking-lost. Stored X gyro bias 4.349289 deg/s; observed X 4.791453–4.916808 deg/s. The former 0.20 deg/s residual gate prevented stationary bias relearning. A rejected window forced motion preview, eventually exceeding the 8-second buffer. This failure is separate from the previously fixed blocking USB output.

## Sensor changes

| Setting | Before | Now |
|---|---:|---:|
| Acceleration range | ±2g | ±2g, unchanged |
| Gyroscope range | ±1024 deg/s | ±256 deg/s |
| Gyro quantization | 0.03125 deg/s/LSB | 0.0078125 deg/s/LSB |
| Both sensors' nominal 6DOF ODR | 224.2Hz | 112.1Hz |
| Both LPF modes | Mode 00, enabled | Mode 00, enabled |
| Nominal filter bandwidth | 6Hz | 3Hz |

SensorLib's `ACC_ODR_125Hz` denotes the accel-only clock: with gyro enabled, code 6 corresponds to 112.1Hz. This is sensor ODR, not a promise that the host reads every output. Matching both LPFs avoids deliberately different filter settings between accel and gyro. Reduced bandwidth trades dynamic response for noise suppression; it cannot remove DC or temperature bias. Gyro clipping is checked at 250 deg/s in sensor axes before mounting-frame rotation.

Readback `cfg=ok:WHOAMI,REVISION,CTRL2,CTRL3,CTRL5` is expected to end in `06,46,11`. `REL_V4` also reports acceleration/gyro window RMS (`asd`, `wsd`), pre-update gyro bias residual (`br`), temperature, sample gaps and dropped diagnostic lines. Transmission remains nonblocking.

## Stationary decision changes

- Window 1.2s → 0.8s; typical acquisition needs 0.8–1.6s of quiet data after filter settling.
- Window RMS limits: accel 0.003g, gyro 0.20 deg/s. Statistics use deviations from the first sample, avoiding subtracting nearly equal large squares at 1g / several deg/s.
- No single 0.003g / 0.22 deg/s deviation veto. Gross excursions remain guarded: 0.012g / 1.2 deg/s within a window; end-to-start acceleration change must be below 0.004g; acceleration norm remains within 0.08g of 1g.
- Stationary exit instantaneous gyro residual limits: 1.5 deg/s transverse to gravity, 1.2 deg/s along gravity. Relative gravity change >0.008 also exits stationary.
- Entry uses mean residual limits: transverse <1.0 deg/s, gravity-axis <0.8 deg/s, combined with window variation. This admits plausible DC bias changes without requiring old bias to remain within 0.20 deg/s forever.
- Valid stationary windows refresh bias; confirmed rest does not integrate the live preview. Moving/red and confirmed/white UX is unchanged. No automatic re-zero after actual tracking loss.

These are engineering defaults, not datasheet-guaranteed accuracy thresholds. Constant slow gravity-axis rotation can be indistinguishable from bias drift. Broader stationary admission deliberately increases that ambiguity; do not claim drift-free yaw or motion detection at every speed. Very gentle continuous tilting may also fall inside the rest thresholds. No 6-axis-only tuning can eliminate those ambiguities.

## Verification

ASan/UBSan geometry, 7–30-degree motion plus 45-degree yaw, command, discontinuity and live-preview regressions pass. Added 120-second synthetic stationary bias creep with vibration at reference inclinations 0/15/30 degrees: tracking remains valid and angle stays below 0.15 degrees. Constant 1 deg/s yaw and 0.5 deg/s changing-gravity tilt remain nonstationary in tests. These selected cases do not establish a universal detection floor.

The new stationary gate passes a sparse 1Hz recording replay (4555 interpolated valid samples, angle range 0.1611–0.1658 degrees). That uses a newly initialized reference and does not reconstruct the original lost tracking interval or unrecorded high-rate vibration. It is not absolute accuracy verification. On-device tuning verification follows build and upload.

Final build and upload succeeded (1,291,943 bytes program, 160,680 bytes static RAM; previous GNU-stack linker warning remains). Initial on-device `REL_V4` readback: `cfg=1:05,7c,06,46,11`, confirming register read success, WHOAMI=05, revision=7c, CTRL2=06, CTRL3=46, CTRL5=11. Sample gap counter is initially zero, typical host sample interval about 16–18ms (slower than the nominal 112.1Hz sensor ODR). Acquisition trace: `simulation/hardware_logs/stationary-60s-20260907-221154.jsonl`. Long stationary measurement still requires user zero capture and valid-state observation.

### First post-zero hardware hold: passed observed interval

User confirmed white display. `simulation/hardware_logs/stationary-60s-20260907-221344.jsonl` was captured for 60.15s; one stale partial prefix was excluded. All 60 complete 1Hz records across 59.40s were `valid=1 status=4`. No tracking loss was observed at the logged instants; a 1Hz trace cannot exclude brief between-record color changes. Sample gap counter remained zero, maximum since boot 21.6ms. Angle 0.003–0.075 degrees (span 0.072 degrees), mean 0.02783 degrees. This is static repeatability, not absolute angle accuracy.

Gyro X bias updated from 3.192979 to 3.031191 deg/s while valid readings continued; temperature changed from 40.48 to 40.12°C (association only, no causal calibration inferred). Largest logged window RMS: accel 0.00084g, gyro 0.0946 deg/s; pre-update bias residual maximum 0.072 deg/s. The former stationary-to-lost symptom did not recur during this sampled interval. Movement, stop/reacquisition and reference-plane return tests remain to be performed.
