# METR4810 Team 2

> **DO NOT PLUG IN USB-C ON THE PCB  -  this may damage the board.** Use ST-LINK for programming and the USB-to-UART adapter for CLI communication.

## How to use

After flashing and powering the PCB, choose either option:

- **Run switch:** switch **PC11 / SW2** ON to start one two-cut `run2` cycle; switch OFF to stop. No UART or computer connection is needed.
- **UART CLI:** connect through PuTTY to use `run`, `stop`, or control the motor, servo and stepper individually.

Both controls work together: an ON-to-OFF transition can stop a CLI-started run, and `stop` can stop a switch-started run. For switch-only use, complete steps 1-3 and skip the UART/PuTTY setup.

## 1. Clone the repository

Open a terminal in the folder where you want to save the project, then run:

```sh
git clone https://github.com/3jonah3/METR4810-Team2.git
cd METR4810-Team2
```

The project is now saved locally in `METR4810-Team2`.

## 2. Set up STM32CubeIDE

1. Download and install [STM32CubeIDE for Eclipse](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Open CubeIDE and choose a workspace folder.
3. Select **File -> Import -> General -> Existing Projects into Workspace**.
4. Select the repository's `METR4810_Team2_code` folder and click **Finish**.
5. Click **Build** (hammer icon).

The project file to open/import in STM32CubeIDE is:

```text
<your clone folder>\METR4810-Team2\METR4810_Team2_code\.project
```

Select the folder containing this `.project` when importing. Leave **Copy projects into workspace** unchecked.

Note: TBH, This is very simplifed there is likely a bit more setup to this, where you have to sort out your workspace, use google/AI for help
You may also have to download stm32MX aswell...

## 3. Flash the PCB

1. Connect ST-LINK to the PCB's **J24** header: **SWDIO -> IO**, **SWCLK -> CLK**, **NRST -> RST**. The programming header is missing a GND pin, so connect the **black ST-LINK ground wire to GND on the spare GPIO header**, as shown below. Power the PCB.
2. In CubeIDE, which can be opened by clicking on the .project file, once opened click **Debug** to flash.
3. If it pauses at `main`, click **Resume (F8)** to run the code.



**PCB connection:** GND was accidentally left out of the programming header during the PCB design. Connect the black ST-LINK ground wire to GND on the spare GPIO header, marked by the arrow below.

<img src="docs/images/V2_Plugging_IN.jpg" alt="ST-LINK PCB wiring with the black ground wire connected to GND on the spare GPIO header" width="400">

**ST-LINK/V2 orientation:** connect the plug as shown, with the **red wire on the bottom row** in this view.

<img src="docs/images/V2_Orientation.jpg" alt="ST-LINK V2 connector orientation" width="400">

## 4. Connect UART

Set the USB-to-UART adapter to **3.3 V** and connect it to **J19 (UART1)**:

| PCB J19 | USB-to-UART adapter |
|---|---|
| TX  -  pin 3 / PC4 | RXD |
| RX  -  pin 4 / PC5 | TXD |
| GND  -  pin 2 | GND |

**Cross TX and RX.** Leave VCC, RTS and CTS disconnected. Power the PCB separately.

**USB-to-UART adapter:** only RXD, TXD and GND need to connect to the PCB, even though the photo shows leads on all six adapter pins.

<img src="docs/images/UART_Adapter.jpg" alt="USB-to-UART adapter with labelled RXD, TXD and GND connections" width="400">

Plug the adapter into your computer. Find its COM number in **Device Manager -> Ports (COM & LPT)**.

## 5. Set up PuTTY

Install [PuTTY](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html), then set:

| Setting | Value |
|---|---|
| Connection type | Serial |
| Serial line | Your COM port, e.g. COM3 |
| Speed | 115200 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |
| Flow control | None |

Under **Session**, select **Serial**, your COM port and **115200**. COM3 is the example shown below.

![PuTTY Session settings](docs/images/putty_serial_setup_1.png)

Data bits, stop bits, parity and flow control are under **Connection -> Serial**.

![PuTTY serial settings: 115200, 8 data bits, 1 stop bit, no parity or flow control](docs/images/putty_serial_setup_2.png)

Under **Terminal**, set **Local echo** and **Local line editing** to **Force on**.

![PuTTY Terminal settings with local echo and local line editing forced on](docs/images/putty_serial_setup_3.png)

Click **Open**, then reset the PCB to see the welcome message and command list. Type `ping` and press **Enter**  -  it should reply `OK PONG`.

## 6. CLI commands

Type one command at a time and press **Enter**.

### General

| Command | Action |
|---|---|
| `help` | Show the command guide. |
| `ping` | Check the serial connection. |
| `status` | Show robot state and actuator status. |
| `limits` | Show switch states: 1 = pressed. |
| `speed(750,500,2000)` | Set straight, turning, and return stepper speeds in steps/s. Each must be 16-2000; configure while stopped. |
| `speed2(750,500,2000,800,2000)` | Set run2 speeds: straight1, curve1, retract1, curve2, return2. Stop first; each is 16-2000 steps/s. |
| `run2` | Home, cut to 135 degrees, retract to 95,000 steps, cut to 145 degrees, then return to physical SW4. Both cuts end at 157,297 steps. |
| `run` | One cycle: home to SW4, drive out to 157,297 steps, return to physical SW4 and stop. The servo follows the schedule and the DC direction follows travel. |
| `pause` | Freeze a run where it is: the stepper and DC motor stop and the servo holds its angle. The run is kept; `run` and manual moves are refused until `continue` or `stop`. |
| `continue` | Resume a paused run from exactly where it stopped, on the same leg with the same drill duty. |
| `stop` | Stop the stepper and DC motor where they are and end the run; the servo holds its angle. |

### Servo

All servo commands use **servo 2**.

| Command | Action |
|---|---|
| `calibrate(110)` | Example: save 110 degrees as straight. Accepts 0-135 for `run`; `run2` requires 0-125. Stop all motion first; resets on power-up. |
| `calibrate(status)` | Show saved straight and true/path versus adjusted/command angles. |
| `straight()` | Move servo 2 to the current calibrated straight angle, or 90 degrees by default. Keeps the calibration. |
| `servo(95)` | Set angle; range **0-180 degrees**. 95 degrees is the legacy driver centre; calibrated straight defaults to 90. |
| `servo(1556u)` | Set pulse width directly; range **500-2500 microseconds**. 1556 us is the centre. |
| `servo(arc)` | Arc test: sweep the run's arc from 90 to 135 degrees over 4 s, hold 1 s, and sweep back over 4 s. Nothing else moves. It ends holding at 90 degrees; `stop` aborts it and the servo holds where it is. |
| `servo(status)` | Show nominal target angle and exact pulse width (not measured position). |
| `servo()` | Show help. |

With the pod fitted, keep servo 2 within **1088-2024 us**, which is the 95 degree arc start +/- 23.4 degrees of pod at the nominal scale. The keeper pocket only allows 23.5 degrees. This historical keeper allowance is not a firmware clamp: run2 reaches 145 degrees before calibration, and manual commands accept the full servo range. Check the fitted mechanism against the commanded range.

### DC motor

| Command | Action |
|---|---|
| `motor(30)` | Forward at 30% duty. |
| `motor(-30)` | Reverse at 30% duty. Range: **-100 to 100**. |
| `motor(0)` or `motor(stop)` | Stop the motor. |
| `motor(status)` | Show duty cycle. |
| `motor()` | Show help. |

### Stepper

| Command | Action |
|---|---|
| `stepper(20)` | Move 20 steps toward SW4. |
| `stepper(-20)` | Move 20 steps toward SW1. |
| `stepper(speed,100)` | Set speed; range **16-2000 steps/second**. Stop the stepper first. |
| `stepper(auto)` | Move continuously between SW1 and SW4. |
| `stepper(stop)` | Stop the stepper. |
| `stepper(status)` | Show speed, mode and remaining steps. |
| `stepper()` | Show help. |

Counted moves allow up to **1,000,000 steps** in either direction and stop early at a limit. Stop any current stepper movement before starting a new counted move.

### Run feedback

While a run is active the CLI prints its progress without being asked:

| Line | When |
|---|---|
| `RUN STARTED: HOMING TO SW4` | A run starts from `run`, `run2` or the PC11 switch. |
| `RUN STARTED: SW4 PRESSED; BACKING OFF 1500 STEPS, THEN HOMING` | A run starts with SW4 already pressed, for example after manually moving onto SW4. |
| `TURN: OUTWARD LIMIT; ...` | Normal counted outward reversal at 157,297 steps, without touching SW1. |
| `RUN2 TURN: POSITION=...; ...` | A run2 phase changes; the message identifies the next phase and target. |
| `TURN: SW4; ...` / `TURN: SW1; ...` | Physical switch events: SW4 after homing, or an earlier-than-planned SW1 hit. |
| `PAUSED AT: ... STEPS (... MM FROM SW4) DURING OUT; ...` | After `pause`. No `DATA` rows print while paused. |
| `CONTINUING: OUT` | After `continue`, followed straight away by a `DATA` row. |
| `RUN COMPLETE: SW4 REACHED; POSITION=0 STEPS; ...` | Return finished on physical SW4; servo holds calibrated straight. |
| `SERVO ARC TEST DONE` / `SERVO ARC TEST STOPPED` | A `servo(arc)` sweep finished, or was ended early by `stop`, a manual servo command, `run` or a fault. |
| `DATA,...` | Every 1000 ms (CSV, see below). |
| `STOPPED AT: ... STEPS (... MM FROM SW4) DURING OUT; SERVO HOLDING ... US` | The run is stopped early: `stop`, the switch or a fault. |
| `FAULT: LIMIT FAULT LATCHED (SW1 AND SW4 PRESSED); ...` | Once, when both limits read pressed together. |
| `FAULT: LIMIT FAULT LATCHED (SW4 STILL PRESSED AFTER BACK-OFF); ...` | Once, when SW4 has not released after the run-start back-off. |

`DATA` rows are comma-separated, with a header row at the start of each run:

```text
DATA,ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4
DATA,190512,OUT,95000,238.2,1500,0.0,-100,0,0
```

`ms` is time since reset. `leg` is `BACKOFF`, `HOMING`, `OUT` or `RETURN`; run2 uses `CUT1`, `RETRACT1`, `CUT2` and `RETURN2` after homing. `steps` and `mm` are the counted position from SW4 (398.87 steps/mm). `servo_us` is the commanded pulse, and `pod_deg` is the nominal pod-angle offset from the calibrated straight command (20 us per degree). `dc` is the drill duty, and `sw1`/`sw4` are the limit switches (1 = pressed). All values are commanded, not measured.

| Command | Action |
|---|---|
| `telemetry(on)` | Turn `DATA` rows on (default). |
| `telemetry(250)` | Print a row every 250 ms; range **100-10000**. |
| `telemetry(off)` | Turn `DATA` rows off. The other lines still print. |

To save a run, turn on PuTTY logging before starting it: **Session -> Logging -> Printable output**, then choose a file. The lines starting with `DATA,` open directly as CSV in Excel or MATLAB.

## 7. Limit switches and run switch

- **Front limit switch -> SW1**.
- **Back limit switch -> SW4**.
- **Run switch -> SW2 / PC11** (not pictured).

<img src="docs/images/Limit_switches_rotated.png" alt="Front and back limit switch connections on SW1 and SW4" width="400">

Switch **PC11 / SW2** ON (contact closed to ground) to start one `run2` cycle, and OFF (contact open) to stop the stepper and DC motor while the servo holds its angle. Both transitions use a 30 ms debounce. After completion or `stop`, switch OFF then ON to start another cycle. Leaving the switch ON never repeats the run; ON at reset requires OFF then ON first.

Homing and initial back-off always use **2,000 steps/s**. Straight calibration above 125 blocks switch-started `run2`; `status` explains the invalid range.

This works without UART connected. The `run` and `stop` commands still work too. The switch is not a UART interlock: CLI commands still work while it is OFF.

## 8. Schematics and PCB routing

Click an image to view it at full size.

### Control PCB schematic

<a href="docs/images/Control_schematic.png"><img src="docs/images/Control_schematic.png" alt="Control PCB schematic" width="1000"></a>

### Control PCB routing

<a href="docs/images/Control_PCB_Routing.png"><img src="docs/images/Control_PCB_Routing.png" alt="Control PCB routing" width="1000"></a>

### Power PCB schematic

<a href="docs/images/Power_PCB_Schematic.png"><img src="docs/images/Power_PCB_Schematic.png" alt="Power PCB schematic" width="1000"></a>

### Power PCB routing

<a href="docs/images/Power_PCB_Routing.png"><img src="docs/images/Power_PCB_Routing.png" alt="Power PCB routing" width="1000"></a>

## Important notes

- **Do not plug a USB-C cable into the PCB; it may damage the board.**

- Send `stop` before using individual actuator commands during `run`.
- During `run`, `motor(stop)`, `motor(0)` and `stepper(stop)` stop the whole program.
- If both limit switches trigger together, or SW4 is still pressed after the run-start back-off, a fault latches. Correct the cause and reset the PCB to clear it.
- Closing PuTTY does **not** stop the robot.

## If UART does not work

Check **TX -> RX**, **RX -> TX**, common GND, the COM port and PuTTY settings. Make sure CubeIDE has not paused the code.

**PB9 flashes when receiving; PF1 flashes when transmitting.**

For more detail, see [SERIAL_CONTROL.md](METR4810_Team2_code/SERIAL_CONTROL.md).

## Modifying the code

1. Create a branch before editing; see [CONTRIBUTING.md](CONTRIBUTING.md).
2. Change run phases, paths and speed defaults in `Core/Src/robot_control.c`;
   endpoint and homing constants are in `Core/Inc/robot_control.h`.
3. Change commands and terminal help in `Core/Src/serial.c`. Keep the README
   command tables in sync. These paths are inside `METR4810_Team2_code/`.
4. Keep generated-file edits inside `USER CODE` sections and leave `Drivers/` alone.
5. Run the host tests and build before flashing:

```powershell
./tools/test-firmware.ps1
./tools/test-firmware.ps1 -OvertravelSteps 1500
./tools/build-firmware.ps1
```

See the [firmware maintenance guide](docs/firmware-maintenance.md) for driver
contracts, calibration, speed profiles and return diagnostics, and the
[path tables](docs/foam-clearance-path.md) for steering values. Ask Jonah if unsure.
