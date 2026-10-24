# Foam-clearance steering path

Confirmed trial targets, 24 October 2026. Positions are DATA step counts with limit overtravel disabled. Angles are nominal servo angles, not pod angles.

Hold 90 degrees to 100,000 steps and 100 degrees from 105,000 to 115,000. Use a monotone cubic Hermite curve between anchors, with zero slope into/out of holds and at the final endpoint. Interior slopes use weighted harmonic means of adjacent secants. No overshoot; identical curve on return. Firmware evaluates the curve at the current counted position and rounds the pulse to whole microseconds. This table is a sample, not a linear lookup table.

| Position (steps) | Servo angle (degrees) | Pulse (us) |
|---:|---:|---:|
| 0-100,000 | 90.00 | 1500 |
| 102,500 | 95.00 | 1556 |
| 105,000 | 100.00 | 1611 |
| 110,000 | 100.00 | 1611 |
| 115,000 | 100.00 | 1611 |
| 117,500 | 101.21 | 1625 |
| 120,000 | 104.06 | 1656 |
| 122,500 | 107.38 | 1693 |
| 125,000 | 110.00 | 1722 |
| 130,000 | 113.19 | 1758 |
| 135,000 | 116.00 | 1789 |
| 140,000 | 118.42 | 1816 |
| 145,000 | 120.45 | 1838 |
| 150,000 | 123.26 | 1870 |
| 155,000 | 128.00 | 1922 |
| 156,000 | 131.21 | 1958 |
| 156,500 | 133.28 | 1981 |
| 157,000 | 134.72 | 1997 |
| 157,297 | 135.00 | 2000 |

The 135-degree endpoint is the counted outward reversal at 157,297 steps, exactly 5000 steps before the observed SW1 hit at 162,297. The timer stops forward pulses at the endpoint even if main-loop processing is delayed, then the controller starts the return. A physical SW1 hit earlier still stops forward motion and reverses after debounce; autonomous overtravel is capped at the counted endpoint. With limit overtravel enabled, subtract LIMIT_OVERTRAVEL_STEPS from DATA position before evaluating this table. The straight section stays at 90 degrees through 100,000 steps.

This path is intended to improve foam clearance; the software checks cannot verify physical clearance.

The autonomous return continues to physical SW4, then completes after switch debounce and holds calibrated straight (90 degrees without calibration). The old 5000-step return offset is removed. Initial homing still uses SW4; outward reversal remains at 157,297 steps.

Calibration: this table lists TRUE/PATH angles. `calibrate(A)` adds A - 90 degrees to every angle for the ADJUSTED servo command. The accepted whole-degree range is 0-135. Defaults return on power-up/reset; positions and speeds do not change.
