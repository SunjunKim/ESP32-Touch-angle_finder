# Accelerometer-only candidate screening (not deployed)

Compared four formulas on the existing 6,582 ideal forward chef/tanto cases: central/local bevel 7–30°, tip tangent up to 45°, fixed blade and pivot per case, reference at center, beta 0 / −central-angle / −7 / −15 / −30. Geometry and mechanical-limit caveats are the same as ACCEL_VS_MAIN.md. No noise, filtering transient, reverse workflow or new physical test is included in this screening. Current deployed firmware remains X-signed total inclination.

`tests/accel_candidates.cpp` evaluates:

1. Current: total inclination with X sign, subtract reference.
2. XZ: atan2(ax, abs(az)), subtract reference (face-up test domain).
3. Pitch: atan2(ax, hypot(ay,az)), subtract reference.
4. Static normal approximation: reconstruct the reference normal from current gravity, reference beta, and sensor XZ direction; assumes central-like heading at every sample. It is not gyro tracking.

All displayed relative magnitudes are compared with the true acute stone/blade plane angle. In this central single-axis reference geometry, reference pitch equals reference beta; signs in the harness use up=−accel, which leaves absolute angle differences unchanged.

| Scenario | Current max | Pitch max | Current MAE | Pitch MAE |
|---|---:|---:|---:|---:|
| Blade level | 0.00001° | 3.25497° | 0.00000° | 0.65531° |
| Stone level at center | 19.71670° | 6.57284° | 2.95934° | 0.59087° |
| beta −7° | 4.93769° | 2.77737° | 0.48981° | 0.53943° |
| beta −15° | 13.55476° | 3.39034° | 1.43912° | 0.66161° |
| beta −30° | 27.15418° | 7.04379° | 2.05015° | 1.33565° |

Other candidates' overall maxima: XZ 7.50551°, static-normal 13.11190°. Pitch has the smallest maximum among these four candidates on this grid; it is not a proven globally optimal estimator.

Pitch removes the discontinuity created by multiplying nonzero total tilt by sign(ax): when ay is appreciable and ax crosses zero, current signed total tilt jumps, whereas pitch crosses zero continuously. It is instantaneous and has no gyro drift or movement history. It can preserve the existing one-button zero workflow and continuous red/white display.

The tradeoff is that pitch measures the long-axis directional inclination, not the full contact-plane angle. The rod/stone long axis need not be perpendicular to the local edge tangent, so curvature/skew creates systematic error. Main/current is exact on an ideally horizontal blade because full stone inclination then equals bevel angle; pitch discards part of that information. Hence pure pitch is not an across-the-board improvement and should not be described as full-plane-angle compensation.

Practical conclusion: pitch is a plausible replacement if limiting large tilted-clamp errors and avoiding X sign jumps is the priority. It still has up to about 7° error on this grid, far above precision sharpening requirements. A broad sub-degree guarantee is not supported by this information set. Further candidate design would need explicit additional geometry assumptions, a validated correction model with independent knife geometries, or a narrower use-case guarantee; arbitrary fitted constants are not justified by these results.
