# Firmware maintenance guide

For setup and everyday commands, see [the README](../README.md). These notes
describe commanded motion; physical position and servo angle are not measured.

## Code layout

| Module inside `METR4810_Team2_code/Core/` | Responsibility |
|---|---|
| `Src/main.c` user sections | Initialise drivers; service motion, CLI and LEDs in order. |
| `Src/robot_control.c`, `Inc/robot_control.h` | Run phases, calibration, speeds, limits and safety gates. |
| `Src/serial.c`, `Inc/serial.h` | Parse commands and queue responses without blocking. |
| `Src/stepper.c` | Count STEP pulses in TIM16 interrupts; stop counted moves in hardware. |
| `Src/servo.c`, `Src/motor.c` | Absolute servo PWM targets and signed drill duty. |
| `Src/switch.c`, `Src/status_led.c` | Raw switch reads and UART activity indicators. |

## Editing conventions

- Document public functions in headers: purpose, parameter units/ranges, returns,
  state changes and errors. Explain interrupt ownership and timing where relevant.
- Make readability edits by hand, following the existing layout and the style
  guide where helpful. Use consistent indentation and clear control bodies;
  no automatic formatter is required.
- Keep existing HAL callback names and public APIs. Do not force assignment-only
  restrictions onto embedded state or split a state machine just to meet a line limit.
- Preserve the existing shared-state reset expressions: this cleanup retains
  their original compiled reads/writes as well as their values.
- Explain intent and invariants in comments rather than obvious assignments.
- Edit only project-owned modules. In generated files edit only `USER CODE`
  sections, and leave vendor `Drivers/` code untouched.
- Preserve command spelling, telemetry columns, timing and pulse accounting
  unless deliberately changing behaviour.

## Verification

Run from the repository root:

```powershell
./tools/test-firmware.ps1
./tools/test-firmware.ps1 -OvertravelSteps 1500
./tools/build-firmware.ps1
```

Host tests simulate parsing, state transitions, limits, pauses and timer pulses.
They cannot establish physical foam clearance. For formatting/comment-only work,
compare the generated HEX image before and after as well as running the tests.

### Servo pulse widths in firmware

The servo mapping is 0 degrees = 500 us, 90 degrees = 1500 us, and 180 degrees = 2500 us at 50 Hz. The legacy driver centre is **95 degrees (1556 us)**: `SERVO_CENTRE_ANGLE` in `servo.h`. Both servos start there at power-up with their PWM off, while the run's arc starts separately at **90 degrees (1500 us)** (`DEMO_START_ANGLE` in `robot_control.h`). The CLI controls servo 2 (PB6); `servo(95)` selects degrees and `servo(1556u)` selects microseconds. Send one command per line. `stop` keeps the servo PWM running so it holds its last angle. Switching the PWM off mid-pulse can throw the servo past its limit. Individual commands are rejected during autonomous run or a latched fault.

For C code using robot state and fault checks:

```c
Robot_SetServo(2, 95);             // Degrees; enables servo 2
Robot_SetServoPulseUs(2, 1556);    // Microseconds; enables servo 2
```

Both functions return NULL on success or an error string. The low-level driver also provides `servo(2, 95)` and `Servo_SetPulseUs(2, 1556)`; these return HAL status and require `Servo_Start(2)` to enable output after initialization. In C, `1556u` is an unsigned integer, not a unit: use the separate pulse-width function. `SERVO_ANGLE_US(angle)` in `servo.h` converts an angle to its pulse width. Angles are nominal; confirm the mechanism can accommodate the commanded travel.
### Single-cut sequence (`run`; PC11/SW2 starts `run2`)

1. Home to **SW4 / PC15** with the DC motor off and servo 2 at **1500 us (90 degrees)**, the start of its arc. Zero is set where SW4 is first touched, after a 30 ms debounce.
   - If SW4 is **already pressed** when the run starts (for example when manually placed on the home switch), the carriage first **backs off 1500 steps** toward SW1. SW4 must then read released; if it is still pressed 30 ms later, a limit fault latches instead of moving further. Otherwise the carriage homes back to first touch as normal.
2. Set position to zero and push outward toward **SW1**, with DC duty **-100%**.
3. At **157,297 steps**, reverse the stepper and DC motor (**+100%**) and follow the servo schedule backwards on the return toward SW4. The counted outward endpoint controls normal reversal; SW1 remains a backup. Return travel continues until physical SW4 activates, even if the counted position reaches zero first.
4. When **physical SW4 activates**, the stepper stops immediately on detection. After 30 ms debounce confirms the switch, the DC motor stops and the run completes, holding the calibrated straight servo angle. A brief switch bounce resumes the return. There is no 5000-step return offset and no return overtravel. Send `run` for another single-cut cycle (switch OFF then ON starts `run2`); the next run backs off SW4 and homes again.

`stop` or switching the run switch OFF during a run stops the stepper and DC motor where they are and ends the run; the servo holds its angle. `pause` freezes the run the same way but keeps it, and `continue` resumes it. A pause during a switch debounce re-checks the switch for a fresh 30 ms on `continue`. A pause during the back-off or an overtravel finishes the remaining steps.

Set the three run speeds with one command while stopped:

```text
speed(750,500,2000)
run
```

The arguments are **straight, turning, return**, in STEP pulses per second (each 16-2000). Homing and back-off always use 2000 steps/s for both programs, independently of speed settings. Outward travel switches to turning speed at 100,000 steps from SW4 first touch and keeps it to the outward reversal, including the 100-degree plateau. The entire return uses the third speed and stops on physical SW4. Limit overtravel shifts the reported transition position by the configured overtravel amount. Speed changes briefly stop and restart the pulse timer, preserving counted position; this is a discrete speed change, not an acceleration ramp.

`status` shows the active speed under `STEPPER` and all three settings under `RUN SPEEDS`. Changes are rejected during running, paused runs, or manual stepper movement. `stepper(speed,n)` remains available and sets all three speeds to n, as well as the manual move speed. Manual moves otherwise use the current selected speed. Defaults are 750/500/2000 (straight/turning/return); settings persist between runs but reset on power-up/reset.

### Session straightness calibration

Use an absolute manual command such as `servo(110)` to find where the head is physically straight. With the stepper, drill and arc test stopped, enter `calibrate(110)`. This saves 110 degrees as straight and commands servo 2 there. Any whole-degree value **0-135** is accepted; the upper limit leaves room for the full 45-degree sweep inside the servo's 0-180 command range.

`straight()` commands servo 2 to the saved straight position without changing calibration. It works like a manual servo command: refused during a run (including pause) or a latched fault, and ends an active servo arc test.

Calibration shifts homing, the entire outward/return path, the completion hold, and `servo(arc)` by `straight - 90` degrees. For example, 100 produces a 100-145 degree sweep, and 110 produces 110-155. Step positions, speeds, and endpoints are unchanged. Manual `servo(angle)` and `servo(pulseu)` remain absolute so you can locate straight without compounding the offset. Calibration is optional, persists across stop/new runs, and resets to 90 on power-up/reset. `calibrate(90)` also restores the default.

`status`, `calibrate(status)`, and servo/run/stop/pause/continue/calibration responses report **TRUE (PATH)** for the original nominal path angle and **ADJUSTED (COMMAND)** for the calibrated absolute servo command. Neither is a measured position. The reported run range also shows the adjusted limits. `DATA` retains its column layout: `servo_us` is the adjusted command, while `pod_deg` stays relative to the calibrated straight position. The table below lists original path angles; add the calibration offset for adjusted angles.

### Servo steering schedule

The original run and first run2 cut hold straight through 100,000 steps,
then follow a monotone cubic Hermite curve to 135 degrees at 157,297 steps.
Cut 2 starts turning at 95,000 and ends at 145 degrees at the same endpoint.
Calibration shifts angles without moving the step coordinates. See the
[complete path tables](foam-clearance-path.md) for anchors and samples.

#### Limit overtravel (dampened switches)

`LIMIT_OVERTRAVEL_STEPS` in `Core/Inc/robot_control.h` is **0**, so the stepper uses first touch for homing at SW4 and reversal at SW1. Only once dampened switch mounts are fitted, set it to **1500** (about 3.8 mm). Each debounced SW1 press or SW4 homing press then drives up to 1500 more pulses before reversing. During an autonomous outward run, this is capped at the counted outward endpoint; the normal counted turnaround adds no overtravel. The zero point becomes the reversal point behind SW4, so the path evaluator subtracts the configured overtravel from the reported position to preserve the same physical steering path; `status` positions count from that reversal point. The run-start back-off grows to 1500 + 1500 steps so the carriage clears the overtravel zone before homing.

The host tests cover both settings, and the general parser and safety tests run in each. Test both before flashing an overtravel build: `-DLIMIT_OVERTRAVEL_STEPS=0U` and `-DLIMIT_OVERTRAVEL_STEPS=1500U`. On Windows, run `./tools/test-firmware.ps1` and `./tools/test-firmware.ps1 -OvertravelSteps 1500`. On Linux, from the repository root:

```sh
gcc -std=c11 -DLIMIT_OVERTRAVEL_STEPS=1500U -Itests/host -IMETR4810_Team2_code/Core/Inc tests/host/test_control.c METR4810_Team2_code/Core/Src/{robot_control,serial,motor,stepper,servo,switch,status_led}.c -o test_control && ./test_control
```

### Checking the return path

The drill polarity for the demo is now **-100% outward toward SW1**, **+100% returning toward SW4**. Initial homing keeps the DC motor off. Manual `motor(duty)` commands retain their signed-duty meaning.

The return uses a fixed reference captured at the actual outward turnaround: `position = outward endpoint - pulses emitted on return`. A counted outward reversal, or an early SW1 press, retraces the part of the schedule actually reached. Brief switch bounce pauses and resumes without changing the reference. Both directions use the same position-to-pulse curve, with their configured section speeds.

Each confirmed demo limit produces a serial line such as:

```text
TURN: OUTWARD LIMIT; POSITION=157297; SERVO=2000 US; NEXT=SW4; DC=100%; RETURN TARGET=SW4
```

`status` shows `OUTWARD END`, `RETURN PULSES`, `RETURN TARGET=SW4`, and `HOME ERROR`. Return travel is continuous toward SW4, with no counted stopping endpoint. It passes the former 5000-step offset and, if necessary, the estimated zero until SW4 is detected. Pause/continue resumes toward that same switch. The stepper stops on switch detection; confirmation requires 30 ms, and the DC motor then stops as the run completes. False switch bounce resumes motion. No return overtravel is driven, even when homing overtravel is enabled.

At completion, `HOME ERROR` is outward pulses minus return pulses minus configured homing overtravel. Position is referenced to SW4 first touch: zero normally, or `LIMIT_OVERTRAVEL_STEPS` if homing placed zero behind the switch. The servo holds its calibrated straight command. The outward endpoint remains 157,297 steps from SW4 first touch.

These are counted command pulses, not encoder measurements. Missed steps can change the physical stopping position.

For a return mismatch, capture the outward TURN line and `status` shortly afterward while retracting. Compare the reported servo target to the observed mechanism before changing angle calibration or compensating for backlash.

### Two-cut sequence: run2

`run2` homes to SW4, then executes:

1. Cut 1: straight to 100,000 steps, then the existing smooth 90-135 degree path to **157,297 steps**.
2. Retract along cut 1 to **95,000 steps** (5,000 steps inside the straight section). No SW4 visit between cuts.
3. Cut 2: start turning immediately at 95,000, following a smooth 90-145 degree path to **157,297 steps**.
4. Return along cut 2 to **physical SW4**, then stop at calibrated straight.

Both outward cuts turn before SW1. Positions above are measured from SW4 first touch; DATA adds `LIMIT_OVERTRAVEL_STEPS` if enabled. During counted run2 legs, an unexpected confirmed target limit stops the run and latches a fault; reset after correcting the cause. Final return and initial homing use physical SW4 as before.

```text
speed2(750,500,2000,800,2000)
run2
```

The five arguments are **straight1, curve1, retract1, curve2, return2**, in steps/second (16-2000). Those are also the defaults. Homing/backoff always run at 2000 steps/s, independent of speed profiles. Cut 2 uses curve2 immediately; there is no straight2 argument. Set speeds while stopped; `speed2` is independent of `speed` and `stepper(speed,n)`. All speeds reset on power-up.

`pause`, `continue`, `stop`, `motor(stop)`, `stepper(stop)`, telemetry and run-switch OFF work with both programs. The maintained run-switch ON edge starts `run2`. With calibration above 125, switch-start is blocked just like CLI `run2`; `status` reports the invalid range. A running program cannot be replaced by the other without stopping first. Status and DATA identify CUT1, RETRACT1, CUT2 and RETURN2.

`calibrate(A)` shifts both cuts by A-90 degrees. For example, `calibrate(110)` gives straight 110, first endpoint 155 and second endpoint 165. `run2` requires A in **0-125** so the larger sweep fits 0-180; the original `run` still supports calibration 0-135. Calibration remains optional and resets on power-up. `straight()` uses the saved straight angle while stopped. Status and command feedback show true/path and adjusted/command angles, plus both adjusted run ranges.

See [the two-cut path table](foam-clearance-path.md#run2-second-cut) for cut 2 targets. Firmware builds and host simulations do not verify foam clearance or lost mechanical steps.
