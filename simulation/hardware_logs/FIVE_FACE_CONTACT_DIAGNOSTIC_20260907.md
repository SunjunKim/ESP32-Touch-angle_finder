# Five-face contact diagnostic — 2026-09-07

Read-only diagnosis; USB contact face excluded by user. Existing zero and
calibration remain unchanged. Four sequential headings per face: nominal
0/90/180/270 degrees, each held still. Headings are user-directed, not independently
measured. Firmware `REL_CONTACT`; A values are calibrated acceleration, not raw
calibration inputs. Infer residual errors only after all faces are captured.

## Face 1: bottom (하면), user confirmed complete

Log: `stationary-60s-20260907-235016.jsonl`.
First two seconds excluded to avoid buffered serial output. Four stable runs
identified by valid/moving flags plus vector continuity. The first has only three
1 Hz observations. Runs are listed in capture order; durations differ, so use
equal direction weighting if combining their averages.

| Run | Time (s) | Samples | Mean Ax (g) | Mean Ay (g) | Mean Az (g) | REL range (deg) |
|---|---|---|---|---|---|---|
| 1 | 2.02–4.03 | 3 | -0.032772 | -0.019357 | -1.000966 | 0.127–0.137 |
| 2 | 6.06–14.10 | 9 | -0.037694 | -0.007063 | -1.001253 | 0.082–0.132 |
| 3 | 17.12–22.16 | 6 | -0.023536 | 0.002229 | -1.001479 | 0.933–0.969 |
| 4 | 25.19–59.40 | 35 | -0.019888 | -0.011792 | -1.001536 | 0.957–1.010 |

## Face 2: USB-opposite end face, user confirmed complete

Observed gravity direction is **-Y**, regardless of the colloquial face name.
The original locked REL normal remains Z by design (no re-zero during diagnosis).
Main log: `stationary-60s-20260907-235227.jsonl`. Initial 0-degree direction
had only one fresh sample before rotation, so it was explicitly recaptured after
another 90-degree rotation from the final 270-degree direction:
`stationary-30s-20260907-235357.jsonl`. Angles are user-directed, uninstrumented.

| Nominal direction | Log segment (s) | Samples | Mean Ax (g) | Mean Ay (g) | Mean Az (g) |
|---|---|---|---|---|---|
| 0, supplemental | 14.99–30.07 | 16 | -0.025835 | -1.010642 | 0.008616 |
| 90, main | 2.30–7.32 | 6 | -0.030826 | -1.015151 | -0.007705 |
| 180, main | 9.33–15.37 | 7 | -0.015977 | -1.014892 | -0.011294 |
| 270, main | 17.38–50.58 | 34 | -0.010513 | -1.012842 | 0.002533 |

Later main-log stationary run 54.61–59.63 has nearly the same gravity as the
270-degree run and is not counted as a fourth distinct heading. Use equal
direction weighting rather than pooling unequal durations.

## Face 3: south (남쪽), user confirmed complete

Log: `stationary-60s-20260907-235525.jsonl`. Observed gravity direction **+X**.
Four stable runs captured after excluding the first two seconds. No re-zero.
Direction labels follow the requested sequence, not an independent heading sensor.

| Nominal direction | Time (s) | Samples | Mean Ax (g) | Mean Ay (g) | Mean Az (g) |
|---|---|---|---|---|---|
| 0 | 2.03–12.07 | 11 | 0.981113 | -0.025566 | -0.008060 |
| 90 | 15.10–23.13 | 9 | 0.979051 | -0.003899 | -0.014095 |
| 180 | 25.15–33.19 | 9 | 0.978233 | 0.006710 | 0.005426 |
| 270 | 36.20–59.34 | 24 | 0.977636 | -0.008332 | 0.009701 |

## Face 4: opposite south, user confirmed complete

Log: `stationary-60s-20260907-235707.jsonl`. Observed gravity direction **-X**.
Four stable runs; original zero and calibration unchanged. Nominal directions
follow the user's requested rotation sequence, not an independent heading sensor.

| Nominal direction | Time (s) | Samples | Mean Ax (g) | Mean Ay (g) | Mean Az (g) |
|---|---|---|---|---|---|
| 0 | 2.32–13.39 | 12 | -1.022570 | -0.007611 | 0.015204 |
| 90 | 15.41–24.45 | 10 | -1.022158 | -0.002460 | 0.001430 |
| 180 | 27.46–35.50 | 9 | -1.022581 | -0.016989 | -0.005366 |
| 270 | 40.53–59.65 | 20 | -1.022824 | -0.023239 | 0.008675 |

## Face 5: opposite bottom, user confirmed complete

Log: `stationary-60s-20260907-235847.jsonl`. Observed gravity direction **+Z**.
Four stable runs; original zero and calibration unchanged.

| Nominal direction | Time (s) | Samples | Mean Ax (g) | Mean Ay (g) | Mean Az (g) |
|---|---|---|---|---|---|
| 0 | 2.05–17.17 | 16 | -0.021601 | -0.020269 | 0.992844 |
| 90 | 19.18–26.21 | 8 | -0.013157 | -0.002576 | 0.992713 |
| 180 | 29.23–38.31 | 10 | -0.028717 | 0.001983 | 0.992803 |
| 270 | 40.32–51.39 | 12 | -0.034791 | -0.013622 | 0.992899 |

## Completed capture and diagnostic findings

All five requested faces captured: -Z, -Y, +X, -X, +Z. USB face (+Y) excluded.
All serial sessions closed. No new calibration or correction was stored.
Analysis: `node simulation/analyze_five_face_contact.cjs`. Aggregation gives each
direction equal weight, rather than weighting long final holds more heavily.

| Face | Equal-direction mean normal component (g) | Inclination spread over four directions (deg) |
|---|---|---|
| bottom / -Z | -1.001309 | 0.871 |
| USB-opposite / -Y | -1.013382 | 1.181 |
| south / +X | 0.979008 | 1.060 |
| opposite south / -X | -1.022533 | 1.230 |
| opposite bottom / +Z | 0.992815 | 1.381 |

The spreads above are absolute inclinations calculated from per-direction mean
vectors, not REL display ranges and not independently known surface-angle errors.

Assuming opposite contact normals are approximately antipodal, midpoint of the
two X-normal means estimates a residual calibrated-frame X bias of -0.021763 g;
the corresponding X half-span is 1.000771 g. Z midpoint is -0.004247 g, half-span
0.997062 g. The X bias magnitude corresponds to about 1.25 degrees at small
angles and is consistent in scale with the reported 0.8/1.7 degree 45-degree
test errors, but does not uniquely explain those readings.

These are residual diagnostic estimates, NOT replacement raw sensor calibration
coefficients. Current logs contain calibrated-path outputs, but do not expose
whether saved calibration is valid/loaded or the actual coefficients. Missing +Y,
unverified face alignment and hand-set headings limit a complete calibration.
Next check is saved-calibration validity/application and then controlled
calibration/45-degree retesting. Separately, inclined-reference yaw ambiguity
of the REL algorithm persists even for ideally calibrated sensor data.
