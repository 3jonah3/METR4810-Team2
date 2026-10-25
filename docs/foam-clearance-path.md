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

## run2 second cut

After cut 1 reaches 135 degrees at 157,297 steps, it retraces the original curve to straight at 100,000, then retracts another 5,000 to **95,000 steps**. Cut 2 starts turning from that position and smoothly reaches **145 degrees at 157,297 steps**. Its final return retraces the second curve, reaches straight at 95,000, and continues to physical SW4.

Cut 2 stretches the original curve horizontally from the original 100,000-157,297 interval to 95,000-157,297. It expands the rounded pulse offset about straight by 611/500, rounding to microseconds. Holds and eased endpoints are retained. Positions below are rounded up to the first step reaching each mapped anchor; angles are approximate uncalibrated commands.

| Cut 2 position from SW4 (steps) | Servo angle (degrees) | Pulse (us) |
|---:|---:|---:|
| 0-95,000 | 90.00 | 1500 |
| 97,719 | 96.12 | 1568 |
| 100,437 | 102.24 | 1636 |
| 111,309 | 102.24 | 1636 |
| 122,182 | 114.39 | 1771 |
| 133,055 | 121.77 | 1853 |
| 154,800 | 136.44 | 2016 |
| 155,887 | 140.40 | 2060 |
| 156,431 | 142.92 | 2088 |
| 156,975 | 144.63 | 2107 |
| 157,297 | 144.99 | 2111 |

Calibration for run2 must be 0-125 degrees; its adjusted second endpoint is straight + 55 degrees. The first cut remains unchanged. `speed2(straight1,curve1,retract1,curve2,return2)` defaults to `speed2(750,500,2000,800,2000)`. Cut 2 uses curve2 from its start. Homing/backoff always use 2000 steps/s. With homing overtravel enabled, DATA positions add `LIMIT_OVERTRAVEL_STEPS`.
