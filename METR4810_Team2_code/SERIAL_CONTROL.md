# USART1 robot control (diagnostic test)

## Hardware and configuration

The serial interface is temporarily moved from USART3 (PB10/PB11) to the existing USART1 pins PC4/PC5 for connection troubleshooting. Motor, servo, switch, and LED pins are unchanged. PB10/PB11 no longer carry the command interface in this build.

| Function | Existing connection | Configuration |
|---|---|---|
| USART1 TX | PC4, AF7 | Connect to adapter RX |
| USART1 RX | PC5, AF7 | Connect to adapter TX |
| STEP | PB4, TIM16 CH1 | 1 MHz timer, initially 500 steps/s |
| DIR | PA4 | LOW toward SW4; HIGH toward SW1 |
| DC motor | PA8/PA9, TIM1 CH1/CH2 | Existing driver, 1 kHz PWM |
| Servo 1 | PB7, TIM4 CH2 | 50 Hz, 0–120 degrees |
| Servo 2 | PB6, TIM4 CH1 | 50 Hz, 0–120 degrees |
| SW1 / SW4 | PC14 / PC15 | Active LOW, internal pull-ups |
| Run/stop / SW2 | PC11 | Active LOW, internal pull-up |
| SW3 | PC10 | Active LOW, internal pull-up |

Use a **3.3 V TTL UART adapter**, a common GND, and the crossed TX/RX wiring above.
Do not use an RS-232 voltage-level adapter. Power the board from its intended supply;
leave the adapter's power pin disconnected unless the board's power design permits it.
The existing USB peripheral is unrelated to this UART connection.

The USART1 pins PC4/PC5 were already assigned to AF7 but the peripheral was not initialized. This diagnostic build enables USART1; the original USART3 implementation had to
enable its clock/interrupt and include the UART HAL driver. The `.ioc` now enables
USART1 asynchronous 115200, 8 data bits, no parity, 1 stop bit, TX/RX, no flow control.
USART1 IRQ priority is 2; TIM1_UP_TIM16 priority is 1 (lower number is higher priority).
The shared timer handler services both TIM1 and TIM16; only manual TIM16 motion
enables update interrupts. SysTick remains priority 15. Existing clocks are unchanged:
SYSCLK/APB timers 16 MHz, timer prescalers 15, hence 1 MHz timer counters.
Other preassigned but inactive peripheral pins remain untouched. PC3 is not used for
run/stop; PC11/SW2 is the run/stop button.

## Operation and safety rules

Power-up is IDLE: DC duty zero, STEP stopped, servo PWM disabled. There is no
startup interlock: UART motion commands work immediately. PC11/SW2 is an active-LOW
run/stop toggle, polled independently of UART. A press starts the same program as
`run`; a press while RUNNING performs `stop`. Press and release each require 30 ms
of stable input. Holding the button does not repeat. A button held during reset
must be released and pressed again before it can start motion. UART and button
controls share state: either can stop a run started by the other. From individual
actuator operation (IDLE), a button press starts the full autonomous program.

`run` starts the DC motor at 100% and the existing SW1/SW4 travel sequence together.
The first target is SW4 unless SW4 is already pressed. At each target press, STEP
stops immediately; after 30 ms, a still-pressed switch reverses the target and a
false press resumes toward the same target. This debounce is nonblocking.

`stop` cancels all stepper activity, sets DC duty to zero, disables both servo PWM
outputs, and enters STOPPED. The next accepted movement command can resume motion.
RUN is restartable and repeated RUN during RUNNING does not reset travel.
Stopping servo PWM stops commands but cannot guarantee an immediate mechanical stop
or holding torque; retained servo targets are historical targets, not feedback.

Simultaneous SW1 and SW4 activation latches a fault in every operating mode and stops
all motors. Releasing the switches or sending STOP does not clear it. Correct the
cause and reset. Both button and UART starts respect this latched fault. Manual motion stops early at its target limit; moving
away from one pressed limit is allowed.

While RUNNING, individual movement and speed commands are rejected. Send STOP first.
`stepper(stop)`, `motor(stop)`, and `motor(0)` during RUNNING stop the whole robot to
cancel coordinated motion consistently. Outside RUNNING, individual stops affect
only that actuator. Standalone commands leave the global state IDLE, so inspect the
actuator status fields to determine movement; IDLE means no full autonomous program.
`stepper(auto)` replaces a manual move with standalone limit-to-limit motion, leaving
DC duty unchanged. A manual move requires the stepper to be stopped first.

Serial STOP is a software stop. For an emergency stop, use independent hardware to
disable drive enables or interrupt actuator power appropriately for the mechanism,
with a deliberate reset/re-enable procedure. UART, firmware polling, and PC11 are
not a substitute for that hardware.

## Commands

Quick example (with room for safe motion):

```text
help
servo(90)
motor(30)
motor(0)
motor(-30)
stop
stepper(speed,100)
stepper(20)
stepper(status)
stop
```


### Diagnostic LEDs

- **LED1, PB9:** lights for 150 ms after USART1 receives a byte from the PC.
  Further received bytes extend the light, including invalid commands. It is off
  at startup and while no data is arriving; it is no longer a heartbeat.
- **LED2, PF1:** lights for 150 ms after USART1 completes transmitting a buffer.
  Repeated transmissions extend the light. It should flash for the startup READY
  message after reset and for replies to PING. This indicates MCU transmission
  completion, not acknowledgement from the PC or proof of correct wiring.

Both LEDs are active HIGH and use the existing GPIO assignments. Timing uses
HAL_GetTick without blocking delays. `status_led.c/.h` owns this behavior; the
UART callback posts an event, and the main loop updates the LEDs.

PF1 should flash on reset for the startup message. Sending PING followed by Enter
should light PB9 for reception and PF1 for the reply. If PB9 never lights, check
adapter-TX-to-PC5, common ground, COM port, baud rate, and that firmware is running.
If PF1 flashes but PuTTY stays blank, check PC4-to-adapter-RX and the PC settings.

### Command reference

On reset the controller prints a welcome message, ASCII owl beneath "Welcome to METR4810 Team 2 CLI", and the complete
lowercase command guide, including descriptions, ranges, examples and motion rules. Physical servo 2 (PB6/TIM4 CH1) is the only servo exposed
by the CLI; servo 1 is never enabled by serial commands. `servo()` / `motor()` /
`stepper()` display help without moving anything. The previous space-separated
actuator commands have been replaced; use parentheses and a comma for stepper speed.

Commands are case-insensitive and accept CR, LF, or CRLF. Leading/trailing spaces
and tabs around names and arguments are accepted, e.g. `motor( -50 )`. Extra parameters, malformed parentheses, and numeric suffixes are rejected.

| Command | Result |
|---|---|
| `ping` | `OK PONG` |
| `help` | Command list |
| `status` | System state, stepper mode/target/speed/remaining count, DC duty, limits, fault, PC11 button state, dropped TX response count |
| `limits` | SW1–SW4; 1 means pressed |
| `run` / `stop` | Start full existing autonomous operation / stop everything |
| `stepper(200)` / `stepper(-200)` | 200 pulses toward SW4 / SW1 |
| `stepper(speed,500)` | Set nominal speed, 16–2000 steps/s; stepper must be stopped |
| `stepper(auto)` / `stepper(stop)` / `stepper(status)` | Standalone automatic travel / stop / inspect |
| `servo(90)` / `servo(45)` | Set physical servo 2 target and enable its PWM |
| `servo(status)` | Last accepted servo 2 target; -1 means no commanded position |
| `motor(50)` / `motor(-50)` | Signed duty from -100 to +100%; negative reverses direction |
| `motor(stop)` / `motor(status)` | Stop / inspect DC motor |

Manual counts are limited to ±1,000,000, excluding zero. TIM16 uses PWM2 one-pulse
mode for manual motion: one rising edge per hardware cycle, ending LOW at overflow.
Hardware stops the timer before the update interrupt rearms the next cycle. This
prevents ISR latency from adding pulses. ISR latency adds a small gap between manual
pulses, so actual speed is slightly below the configured nominal rate. Continuous
automatic travel still uses the original PWM1 waveform. Integer timer division also
quantizes speed. Counts describe electrical STEP pulses, not encoder-confirmed
mechanical displacement; stalls, STOP, or limits can prevent completing a move.

The original servo conversion is retained: 0–120 degrees maps to 1000–2000 microseconds,
with rounding; 60 degrees is 1500 microseconds. These pulse limits were provisional
in the existing driver and still require calibration on the actual servos.

RX uses a 256-byte ring; commands have at most 95 characters; TX uses a 4096-byte ring.
No dynamic allocation is used by application modules. Callbacks only buffer bytes,
complete TX, signal UART errors, or count/rearm pulses. Parsing and control run in
the main loop. TX never blocks STOP processing. If responses saturate TX, whole
response fragments can be dropped; STATUS reports the cumulative count. Send one
command at a time and wait for its reply during normal use.

Overlong or non-ASCII/control-character input stops the robot and discards through
the next terminator. UART overrun or receive-ring overflow also stops the robot,
flushes pending input, and requires a fresh newline before retrying. Never execute
the truncated suffix of a damaged command. PuTTY local editing avoids sending
backspace control bytes; they are deliberately rejected by the firmware.

## PuTTY acceptance test (physical board required)

1. Build/flash the firmware with actuator power initially disconnected. In Windows
   Device Manager find the USB adapter's COM number. In PuTTY select **Serial**, that
   COM port, **115200**, **8 data bits**, **1 stop bit**, **None parity**, **None flow
   control**. For readable typing, set Terminal → Local echo and Local line editing
   to **Force on**. The firmware does not send a `>` prompt or echo typed characters.
2. With PC11 HIGH, reset. Expect the welcome/help and `READY: IDLE; type help`.
   Confirm IDLE, zero DC duty, stopped stepper, button RELEASED and no servo PWM.
   Verify UART movement works without first pressing PC11, then stop.
3. Disconnect UART and press PC11: the full autonomous program must run. Holding
   it must not toggle repeatedly. Release and press again: all motion must stop.
   Reset while holding it: remain idle until release and a fresh press. Also test
   UART `run` followed by button stop, and button run followed by UART `stop`.
4. In a clear, secured mechanism, send the sequence below. Verify the DC motor and
   stepper run together, reverse at both limits, stop, and restart without reset.

   ```text
   run
   status
   stop
   status
   run
   stop
   ```

5. Send STOP during travel, during the 30 ms limit debounce, and while holding either
   limit. Confirm no later autonomous restart. Try individual movement commands
   during run; expect rejection. Try `motor(stop)` and `stepper(stop)` during run; both
   must cancel the entire program. Start RUN with SW4 pressed; first travel must
   be toward SW1. Simulate a brief switch bounce; resume toward the original target.
6. With the stepper stopped, set speed to 500, then issue `stepper(1)`, `stepper(200)`,
   `stepper(-200)` with sufficient free travel. Count rising edges on PB4 using a
   logic analyzer: exactly 1, 200, 200 unless interrupted by STOP or a limit.
   Repeat at 16 and 2000 steps/s, including simultaneous STATUS traffic. Confirm
   LOW idle, DIR setup time, clean first/last pulses, no extra edges after STOP,
   and no lost mechanical steps. STOP mid-move and verify REMAINING becomes zero.
7. Test `motor(50)`, `motor(-50)`, `motor(100)`, `motor(-100)`, `motor(stop)`, and `motor(status)`. Verify the physical direction and signed duty in status; use low duty initially with the mechanism clear. Test servo targets
   0, 60, 90, 120 on physical servo 2, then STOP. Measure 50 Hz and pulse widths
   1000, 1500, 1750, 2000 microseconds. Verify initialization does not move either
   servo; validate safe mechanical endpoints before exercising the full range.
8. Test lowercase commands, CR-only, LF-only, and CRLF (one reply per command).
   Try `stepper(speed,-100)`, `servo(200)`, `motor(101)`, `motor(-101)`, `servo(1,90)`,
   `STEPPER`, `motor(50x)`, a huge integer, `RUN extra`, and `UNKNOWN`.
   Expect errors without starting movement. Paste more than 95 characters in one
   line; expect a stop/error, then verify PING works. Flood input to exercise RX/TX
   saturation; after receive overflow send an empty line, then PING. Inject UART
   framing errors if test equipment permits and check recovery without reset.
9. Activate SW1 and SW4 together while idle and while running. All motion must stop;
   STATUS must show a latched fault. Release both and attempt every movement command;
   none may run, including via PC11. Correct the cause and reset; verify IDLE, then
   confirm a fresh button press or UART run can start again.

Measure worst-case STOP-to-STEP-low latency under command traffic; no hard real-time
latency is claimed from host tests. The only intentional motion-path delay remaining
is the existing driver's 1 ms DIR setup (HAL tick rounding can make it slightly longer).
There is no homing timeout or broken-switch detection beyond the original dual-limit
fault; those are outside this requested autonomous sequence.

## Files, building, regeneration, and validation

- `Core/Src/serial.c`, `Core/Inc/serial.h`: interrupt buffers, recovery, parser, replies.
- `Core/Src/robot_control.c`, `Core/Inc/robot_control.h`: modes, debounced run button, fault latch,
  nonblocking original travel sequence, individual-command policy.
- `Core/Src/main.c`: generated initialization plus protected calls to both modules.
- `stepper.c/.h`: retain continuous travel driver; add speed/count APIs and TIM16
  one-pulse completion callback. `servo.c/.h`: retain conversion and pin mapping;
  separate initialization from PWM enabling and add stop. `motor.c` and `switch.c`
  are unchanged.
- `.ioc`, generated MSP/IRQ files and HAL configuration: USART1 and timer IRQ setup.
  Four UART/UARTEx HAL source/header files come unmodified from the installed
  STM32Cube G4 V1.6.3 package, matching this project's firmware package.
- `tools/build-firmware.ps1`: compile/link all firmware sources with ARM GCC;
  products go to `METR4810_Team2_code/build/`.
- `tests/host/`: actual application/driver sources exercised with a minimal HAL model.
  `tools/test-firmware.ps1` runs these using installed MATLAB LCC64, or build on a
  host with a C compiler using the command below. These tests model callbacks and
  register writes; they do not emulate STM32 peripheral timing or electronics.

From the repository root in PowerShell:

```powershell
./tools/build-firmware.ps1
./tools/test-firmware.ps1
```

Pass `-ToolchainBin` to the build script if ARM GCC is elsewhere, or `-LccRoot` to
the test script. On a GCC-equipped host:

```sh
mkdir -p METR4810_Team2_code/build
gcc -std=c11 -Itests/host -IMETR4810_Team2_code/Core/Inc \
  tests/host/test_control.c METR4810_Team2_code/Core/Src/{serial,robot_control,motor,stepper,servo,switch,status_led}.c \
  -o METR4810_Team2_code/build/test_control
METR4810_Team2_code/build/test_control
```

In CubeIDE refresh the project and use Project → Clean then Build Project, so its
managed build discovers new `.c` files. Do not rely on old Debug makefiles or ELF.
The standalone build compiles and links with the installed GCC 14.3.rel1 and existing
linker script. No hardware was flashed during development.

Custom main.c calls are inside USER CODE blocks; HAL callbacks live in separate
files. The `.ioc` has KeepUserCode enabled and owns peripheral/IRQ configuration.
CubeMX regeneration was run on an isolated copy: the custom modules and main-loop
hooks survived, and USART1 plus TIM1_UP_TIM16 handlers were generated correctly.
The resulting generated initialization/IRQ files were incorporated into the project.
After future regeneration, inspect the diff, rebuild, rerun host tests, and repeat
at least the idle/button/RUN/STOP hardware tests. Do not move custom code outside
USER CODE blocks or define duplicate HAL UART/timer callbacks.

Development validation: ARM firmware compile/link passed with `-Wall -Wextra`;
host regression suite passed 861 checks. Physical wiring, electrical pulse accuracy,
servo calibration, switch noise, direction, motor dynamics, and worst-case stop
latency remain to be validated on the actual PCB.
