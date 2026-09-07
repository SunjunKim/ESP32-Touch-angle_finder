# Current accelerometer-only REL vs main — 2026-09-07

> Historical snapshot of the unsigned-inclination revision. The later X-signed firmware is described in ACCEL_REL.md. Re-running compare_accel_main.cpp now evaluates that active X-signed Filter and will not reproduce this saved unsigned report.

## Conclusion

The current unsigned-inclination difference eliminates ideal same-plane yaw-induced sign flips, but is **not** the minimum actual sharpening-angle-error algorithm over the previous tilted-knife scenario range. It can display 0° for a true 30° bevel. Main is substantially more accurate in several tilted-reference scenarios, despite its own serious heading/sign failures. No firmware changes or uploads were made in this comparison.

## Reproduction and fairness

`clang++ -std=c++17 -O2 -Wall -Wextra -Werror tests/compare_accel_main.cpp -o /private/tmp/compare-accel-main && /private/tmp/compare-accel-main`

Also passed with AddressSanitizer/UndefinedBehaviorSanitizer. Raw output: `ACCEL_VS_MAIN_RESULTS.txt`.

- Existing `rel_extended_scenarios.cpp` fixed-pivot chef/tanto geometry is reused verbatim: 200mm blade, spine-axis clamp rotation, horizontal central tangent, central reference at 50%, pivot height solved per central angle, fixed pivot/clamp/blade within each forward case. Stone translates telescopically and pivots to include the local tangent in its face; width-axis alignment is not forced outside the center.
- Central sharpening angle 7–30° in 1° steps; local true sharpening angle must also lie in 7–30°. Clamp beta=0, minus central angle (stone horizontal at center), or −7/−15/−30°. Tip tangent 0/15/30/45°; positions 0/50/62.5/74.9/75.1/87.5/100%. Tanto samples straddle rather than sit on the undefined sharp-corner tangent.
- Same assumptions as the previous comparison: no original slider-height limit or mechanical interference check. A stone horizontal at the center is NOT forced to remain horizontal along the curved edge. For a tilted blade, such a horizontal stone generally cannot contain a nonhorizontal edge tangent.
- Main means commit `2f9d288`, signed total tilt based on calibrated acceleration-X/pitch, minus captured reference; first-reference horizon deadband 0.15°. Absolute display magnitude is compared with the actual acute plane angle, avoiding treating a negative sign alone as an error.
- Current uses deployed `accel_rel::Filter` (scalar tau=0.15s) and absolute inclination difference, including its unsigned reference capture. Both receive identical calibrated acceleration samples. Three seconds at reference and target settle the current filter. These are endpoint accuracy tests, not motion-path/transient response tests; device LPF transients are not modeled.
- Ideal forward 6,582 cases; noisy forward 65,820 (10 seeds/case); reverse 960; same-plane yaw 1,440; level sweep endpoints 96: **74,898 comparisons**. Sweep endpoints are independently evaluated; exact level-reference algebra makes the settled result equivalent to reusing a fixed level reference, but dynamic continuous-sweep behavior is not asserted.
- Original geometry's counts and main's −30° maximum error (27.15418°) are asserted as regression checks. All current calculations remained numerically valid; validity is not angle accuracy.

## Ideal input — blade reference, measure on stone

MAE = mean absolute error. All figures are degrees; no sensor noise or calibration error.

| Scenario | Cases | Main MAE | Current MAE | Main max | Current max |
|---|---:|---:|---:|---:|---:|
| Blade horizontal | 1309 | <0.0001 | <0.0001 | <0.0001 | <0.0001 |
| Stone horizontal at center, then move | 1318 | 2.95934 | 3.04093 | 19.71670 | 22.86086 |
| Blade beta −7° | 1314 | 0.48981 | 11.64534 | 4.93769 | 14.00002 |
| Blade beta −15° | 1318 | 1.43912 | 11.10568 | 13.55476 | 30.00000 |
| Blade beta −30° | 1323 | 2.05015 | 1.86773 | 27.15418 | 21.45931 |

Current ideal signed mean error is negative in every nonhorizontal group: it underestimates the true angle. Its apparent stability is not evidence of correctness.

### Representative failures

1. beta −15°, central sharpening angle 30°, central straight edge: each plane has 15° unsigned inclination on opposite sides of horizontal. **True 30°, main magnitude 30°, current 0°**. Filtering cannot recover the discarded direction sign.
2. Stone initially horizontal, beta −28°, central angle 28°, 45° chef tip at 100%: true 29.57208°, main 49.28878°, current 6.71121°. Neither method is acceptable as an accurate general plane-angle measurement here.
3. beta −30°, central angle 22°, 45° chef tip at 100%: true 23.59503°, main 50.74921°. Main's global worst in the prior forward range is reproduced.
4. Same 30° inclined plane, sensor rotated 180° in its own plane: true 0°, main magnitude 60°, current 0°. The current method successfully addresses the reported in-plane direction instability in ideal geometry.

## Other ideal scenarios

| Scenario | Cases | Main max error | Current max error |
|---|---:|---:|---:|
| Central stone reference → horizontal blade | 192 | <0.0001° | <0.0001° |
| Horizontal central stone reference → tilted blade | 192 | <0.0001° | <0.0001° |
| Central stone reference → tilted blade, both planes tilted | 576 | <0.0001° | 30.0000° |
| Fixed horizontal reference, stone angle endpoints 7→30→7° | 48 | <0.0001° | <0.0001° |
| Fixed horizontal reference, blade angle endpoints 7→30→7° | 48 | <0.0001° | <0.0001° |
| Same-plane yaw 0–359°, inclination 0/7/15/30° | 1440 | 60.0000° | <0.0001° |

Reverse cases preserve the saved **central stone plane** while the instrument is on the blade. They do not measure a stone that is moved after capture. Blade instrument heading 0/15/30/45° and joint 180° flip are included. Main's near-zero reverse result is specific to these controlled central geometries and alignments, not unrestricted yaw immunity.

## Synthetic acceleration noise

Independent Gaussian axis SD=(0.000389, 0.000546, 0.000368)g at synthetic 100Hz, chosen from earlier device observations; 10 seeds/case. Gyroscope is not involved. This is not a new calibration of the current 125Hz/3.3Hz-LPF sensor and omits correlated noise, residual axis gains/biases, temperature dependence, nonplanar contact, and dynamic linear acceleration.

| Forward scenario | Main max total error | Current max total error |
|---|---:|---:|
| Blade horizontal | 0.07774° | 0.04936° |
| Stone horizontal at center | 22.93278° | 22.86851° |
| beta −7° | 6.41502° | 14.01470° |
| beta −15° | 13.60688° | 29.99997° |
| beta −30° | 27.19217° | 21.46675° |

Current horizontal-blade MAE=0.03392°, main MAE=0.01861°: although filtering lowers the maximum in this sample, unsigned tilt noise creates a positive captured horizon offset that biases REL downward. Main's 0.15° pre-reference deadband removes that contribution in this case. Do not summarize the new method as uniformly better even with a level reference. For tilted scenarios, geometric error dominates sensor noise.

## Implication

Current firmware is suitable as a stable **inclination-difference** display, especially when one plane is horizontal. It is not a general sharpening-angle solution for the prior tilted-clamp range. If gyro remains excluded, removing the above ambiguity requires additional user-provided orientation/branch information, constrained geometry, or limiting the valid use case. An unconditional absolute-value subtraction cannot provide both arbitrary same-plane yaw immunity and correct opposite-tilt sharpening angles.
