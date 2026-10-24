# METR4810 Team 2

> **DO NOT PLUG IN USB-C ON THE PCB — this may damage the board.** Use ST-LINK for programming and the USB-to-UART adapter for CLI communication.

## How to use

After flashing and powering the PCB, choose either option:

- **Button:** press **PC11 / SW2** to start the autonomous run. Release and press again to stop. No UART or computer connection is needed.
- **UART CLI:** connect through PuTTY to use `run`, `stop`, or control the motor, servo and stepper individually.

Both controls work together: the button can stop a CLI-started run, and `stop` can stop a button-started run. For button-only use, complete steps 1–3 and skip the UART/PuTTY setup.

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
3. Select **File → Import → General → Existing Projects into Workspace**.
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

1. Connect ST-LINK to the PCB's **J24** header: **SWDIO → IO**, **SWCLK → CLK**, **NRST → RST**. The programming header is missing a GND pin, so connect the **black ST-LINK ground wire to GND on the spare GPIO header**, as shown below. Power the PCB.
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
| TX — pin 3 / PC4 | RXD |
| RX — pin 4 / PC5 | TXD |
| GND — pin 2 | GND |

**Cross TX and RX.** Leave VCC, RTS and CTS disconnected. Power the PCB separately.

**USB-to-UART adapter:** only RXD, TXD and GND need to connect to the PCB, even though the photo shows leads on all six adapter pins.

<img src="docs/images/UART_Adapter.jpg" alt="USB-to-UART adapter with labelled RXD, TXD and GND connections" width="400">

Plug the adapter into your computer. Find its COM number in **Device Manager → Ports (COM & LPT)**.

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

Data bits, stop bits, parity and flow control are under **Connection → Serial**.

![PuTTY serial settings: 115200, 8 data bits, 1 stop bit, no parity or flow control](docs/images/putty_serial_setup_2.png)

Under **Terminal**, set **Local echo** and **Local line editing** to **Force on**.

![PuTTY Terminal settings with local echo and local line editing forced on](docs/images/putty_serial_setup_3.png)

Click **Open**, then reset the PCB to see the welcome message and command list. Type `ping` and press **Enter** — it should reply `OK PONG`.

## 6. CLI commands

Type one command at a time and press **Enter**.

### General

| Command | Action |
|---|---|
| `help` | Show the command guide. |
| `ping` | Check the serial connection. |
| `status` | Show robot state and actuator status. |
| `limits` | Show switch states: 1 = pressed. |
| `run` | One cycle: home to SW4, drive out to SW1, return to SW4 and stop there. The servo follows the schedule and the DC direction follows travel. |
| `pause` | Freeze a run where it is: the stepper and DC motor stop and the servo holds its angle. The run is kept; `run` and manual moves are refused until `continue` or `stop`. |
| `continue` | Resume a paused run from exactly where it stopped, on the same leg with the same drill duty. |
| `stop` | Stop the stepper and DC motor where they are and end the run; the servo holds its angle. |

### Servo

All servo commands use **servo 2**.

| Command | Action |
|---|---|
| `servo(95)` | Set angle; range **0–180 degrees**. 95 degrees is the centre, where the pod is straight. |
| `servo(1556u)` | Set pulse width directly; range **500–2500 microseconds**. 1556 us is the centre. |
| `servo(arc)` | Arc test: sweep the run's arc from 95 to 135 degrees over 4 s, hold 1 s, and sweep back over 4 s. Nothing else moves. It ends holding at 95 degrees; `stop` aborts it and the servo holds where it is. |
| `servo(status)` | Show nominal target angle and exact pulse width (not measured position). |
| `servo()` | Show help. |

With the pod fitted, keep servo 2 within **1088–2024 us**, which is the 95 degree arc start ± 23.4 degrees of pod at the nominal scale. The keeper pocket only allows 23.5 degrees. The autonomous run caps itself there; manual `servo(...)` commands do not.

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
| `stepper(speed,100)` | Set speed; range **16–2000 steps/second**. Stop the stepper first. |
| `stepper(auto)` | Move continuously between SW1 and SW4. |
| `stepper(stop)` | Stop the stepper. |
| `stepper(status)` | Show speed, mode and remaining steps. |
| `stepper()` | Show help. |

Counted moves allow up to **1,000,000 steps** in either direction and stop early at a limit. Stop any current stepper movement before starting a new counted move.

### Run feedback

While a run is active the CLI prints its progress without being asked:

| Line | When |
|---|---|
| `RUN STARTED: HOMING TO SW4` | A run starts, from `run` or the PC11 button. |
| `RUN STARTED: SW4 PRESSED; BACKING OFF 1500 STEPS, THEN HOMING` | A run starts with SW4 already pressed, for example right after a completed run. |
| `TURN: SW4; …` / `TURN: SW1; …` | At SW4 after homing, and at SW1. |
| `PAUSED AT: … STEPS (… MM FROM SW4) DURING OUT; …` | After `pause`. No `DATA` rows print while paused. |
| `CONTINUING: OUT` | After `continue`, followed straight away by a `DATA` row. |
| `RUN COMPLETE: BACK AT SW4; HOME ERROR=… STEPS; …` | The run has finished its cycle back at SW4. |
| `SERVO ARC TEST DONE` / `SERVO ARC TEST STOPPED` | A `servo(arc)` sweep finished, or was ended early by `stop`, a manual servo command, `run` or a fault. |
| `DATA,…` | Every 1000 ms (CSV, see below). |
| `STOPPED AT: … STEPS (… MM FROM SW4) DURING OUT; SERVO HOLDING … US` | The run is stopped early: `stop`, the button or a fault. |
| `FAULT: LIMIT FAULT LATCHED (SW1 AND SW4 PRESSED); …` | Once, when both limits read pressed together. |
| `FAULT: LIMIT FAULT LATCHED (SW4 STILL PRESSED AFTER BACK-OFF); …` | Once, when SW4 has not released after the run-start back-off. |

`DATA` rows are comma-separated, with a header row at the start of each run:

```text
DATA,ms,leg,steps,mm,servo_us,pod_deg,dc,sw1,sw4
DATA,190512,OUT,95000,238.2,1624,3.4,-100,0,0
```

`ms` is time since reset. `leg` is `BACKOFF`, `HOMING`, `OUT` or `RETURN`. `steps` and `mm` are the counted position from SW4 (398.87 steps/mm). `servo_us` is the commanded pulse, and `pod_deg` is the nominal pod angle (20 us per degree). `dc` is the drill duty, and `sw1`/`sw4` are the limit switches (1 = pressed). All values are commanded, not measured.

| Command | Action |
|---|---|
| `telemetry(on)` | Turn `DATA` rows on (default). |
| `telemetry(250)` | Print a row every 250 ms; range **100–10000**. |
| `telemetry(off)` | Turn `DATA` rows off. The other lines still print. |

To save a run, turn on PuTTY logging before starting it: **Session → Logging → Printable output**, then choose a file. The lines starting with `DATA,` open directly as CSV in Excel or MATLAB.

## 7. Limit switches and run button

- **Front limit switch → SW1**.
- **Back limit switch → SW4**.
- **Run button → SW2 / PC11** (not pictured).

<img src="docs/images/Limit_switches_rotated.png" alt="Front and back limit switch connections on SW1 and SW4" width="400">

Press **PC11 / SW2** to start `run`. Release and press again to stop.

This works without UART connected. The `run` and `stop` commands still work too. There is no startup interlock.

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

Check **TX → RX**, **RX → TX**, common GND, the COM port and PuTTY settings. Make sure CubeIDE has not paused the code.

**PB9 flashes when receiving; PF1 flashes when transmitting.**

For more detail, see [SERIAL_CONTROL.md](METR4810_Team2_code/SERIAL_CONTROL.md).

## Modifying the code

Create a new branch before making changes. See [CONTRIBUTING.md](CONTRIBUTING.md).

Ask Jonah if you have any questions.

### Servo pulse widths in firmware

The servo mapping is 0 degrees = 500 us, 90 degrees = 1500 us, and 180 degrees = 2500 us at 50 Hz. The centre, where the pod is straight, is **95 degrees (1556 us)**: `SERVO_CENTRE_ANGLE` in `servo.h`. Both servos start there at power-up with their PWM off, and the run's arc starts there. The CLI controls servo 2 (PB6); `servo(95)` selects degrees and `servo(1556u)` selects microseconds. Send one command per line. `stop` keeps the servo PWM running so it holds its last angle. Switching the PWM off mid-pulse can throw the servo past its limit. Individual commands are rejected during autonomous run or a latched fault.

For C code using robot state and fault checks:

```c
Robot_SetServo(2, 95);             // Degrees; enables servo 2
Robot_SetServoPulseUs(2, 1556);    // Microseconds; enables servo 2
```

Both functions return NULL on success or an error string. The low-level driver also provides `servo(2, 95)` and `Servo_SetPulseUs(2, 1556)`; these return HAL status and require `Servo_Start(2)` to enable output after initialization. In C, `1556u` is an unsigned integer, not a unit: use the separate pulse-width function. `SERVO_ANGLE_US(angle)` in `servo.h` converts an angle to its pulse width. Angles are nominal; confirm the mechanism can accommodate the commanded travel.
### Demo sequence (`run` or PC11/SW2)

1. Home to **SW4 / PC15** with the DC motor off and servo 2 at **1556 us (95 degrees)**, the start of its arc. Zero is set where SW4 is first touched, after a 30 ms debounce.
   - If SW4 is **already pressed** when the run starts (for example after a completed run), the carriage first **backs off 1500 steps** toward SW1. SW4 must then read released; if it is still pressed 30 ms later, a limit fault latches instead of moving further. Otherwise the carriage homes back to first touch as normal.
2. Set position to zero and push outward toward **SW1**, with DC duty **-100%**.
3. At **SW1**, reverse the stepper and DC motor (**+100%**) and follow the servo schedule backwards on the return to SW4. There is no step-count limit: only the limit switches reverse the run.
4. Back at SW4 the run **ends**: the stepper and DC motor stop and the servo returns to the start of its arc (95 degrees, 1556 us). The CLI prints `RUN COMPLETE` with the home error. Send `run` or press the button for another cycle; a new run always homes again.

`stop` or a run-button press during a run stops the stepper and DC motor where they are and ends the run; the servo holds its angle. `pause` freezes the run the same way but keeps it, and `continue` resumes it. A pause during a switch debounce re-checks the switch for a fresh 30 ms on `continue`. A pause during the back-off or an overtravel finishes the remaining steps.

Run preserves the selected stepper speed (16–2000 pulses/s). Set it while stopped:

```text
stop
stepper(speed,333)
run
status
```

333 is an example, not a demo speed setting. The existing startup default remains 500 pulses/s until changed; speed is not saved across reset. Position comes from emitted STEP pulses, not elapsed time or encoder feedback. Microstepping/mechanical calibration must match the supplied schedule. `status` reports homing/table phase, position from SW4, and servo pulse width.

### Servo steering schedule

The servo schedule comes from the CAD model (`Steering_schedule_from_CAD.csv`, 21 Oct 2026). At each position it gives the pod angle that keeps the cutter tip on the bore centreline. The run's servo arc starts at **95 degrees (1556 us)**, where the CAD offset is zero, and never goes past **135 degrees (2000 us)**. Offsets use the nominal scale of 20 us per pod degree through the 1.8:1 gears. Degrees here use the same mapping as `servo(angle)`.

The CSV counts steps from the CAD zero, which is the rear reversal point 1500 steps past SW4 first touch (398.87 steps/mm). With no limit overtravel, the firmware zeroes at SW4 first touch, so its positions are the CSV steps minus 1500.

| CSV steps | Position from SW4 first touch | Pod angle | Servo 2 (us) |
|---:|---:|---:|---:|
| 1,500–66,715 | 0–65,215 | 0.00° | 1556 |
| 69,706 | 68,206 | 0.13° | 1559 |
| 73,695 | 72,195 | 0.37° | 1563 |
| 77,684 | 76,184 | 0.70° | 1570 |
| 81,672 | 80,172 | 1.11° | 1578 |
| 85,661 | 84,161 | 1.60° | 1588 |
| 89,650 | 88,150 | 2.18° | 1600 |
| 93,638 | 92,138 | 2.84° | 1613 |
| 97,627 | 96,127 | 3.59° | 1628 |
| 101,616 | 100,116 | 4.41° | 1644 |
| 105,604 | 104,104 | 5.33° | 1663 |
| 109,593 | 108,093 | 6.33° | 1683 |
| 113,582 | 112,082 | 7.41° | 1704 |
| 117,570 | 116,070 | 8.57° | 1727 |
| 121,559 | 120,059 | 9.82° | 1752 |
| 125,548 | 124,048 | 11.14° | 1779 |
| 129,537 | 128,037 | 12.54° | 1807 |
| 133,525 | 132,025 | 13.98° | 1836 |
| 137,514 | 136,014 | 15.42° | 1864 |
| 141,503 | 140,003 | 16.85° | 1893 |
| 144,853 | 143,353 (mid-bore) | 18.06° | 1917 |
| 145,491 | 143,991 | 18.29° | 1922 |
| 149,480 | 147,980 | 19.72° | 1950 |
| 153,469 | 151,969 | 21.16° | 1979 |
| 156,289 | 154,789 (arc reaches 135°) | 22.20° | 2000 |
| 157,457 | 155,957 | 22.59° → 22.20° | 2000 (capped) |
| 160,250 | 158,750 (SW1 first touch) | 23.60° → 22.20° | 2000 (capped) |
| 161,750 | 160,250 | 24.13° → 22.20° | 2000 (capped) |

Pulse widths are interpolated between adjacent rows and applied without a time-based ramp. The run follows the table all the way to SW1 and reverses there; there is no step-count limit. The table reaches the 135 degree end of the arc about 10 mm before SW1 (CSV 156,289), and the servo holds there to SW1 and beyond. That is 22.2 degrees of pod at the nominal scale, or about 20 degrees on the MG995 (measured at about 22 us per pod degree), inside the 23.5 degree keeper pocket. The return to SW4 ends the run. Independent `stepper(auto)` remains a stepper-only test and does not activate this servo/DC schedule.

#### Limit overtravel (dampened switches)

`LIMIT_OVERTRAVEL_STEPS` in `Core/Inc/robot_control.h` is **0**, so the stepper reverses at first touch of each switch. Only once dampened switch mounts are fitted, set it to **1500** (about 3.8 mm). Each debounced SW1/SW4 press then drives up to 1500 more pulses before reversing. The zero point becomes the reversal point behind SW4, which is the CSV's own zero, so the table needs no change; `status` positions then count from that point rather than from first touch. The run-start back-off grows to 1500 + 1500 steps so the carriage clears the overtravel zone before homing.

The host tests cover both settings, and the general parser and safety tests run in each. Test both before flashing an overtravel build: `-DLIMIT_OVERTRAVEL_STEPS=0U` and `-DLIMIT_OVERTRAVEL_STEPS=1500U`. On Linux, from the repository root:

```sh
gcc -std=c11 -DLIMIT_OVERTRAVEL_STEPS=1500U -Itests/host -IMETR4810_Team2_code/Core/Inc tests/host/test_control.c METR4810_Team2_code/Core/Src/{robot_control,serial,motor,stepper,servo,switch,status_led}.c -o test_control && ./test_control
```

### Checking the return path

The drill polarity for the demo is now **-100% outward toward SW1**, **+100% returning toward SW4**. Initial homing keeps the DC motor off. Manual `motor(duty)` commands retain their signed-duty meaning.

The return uses a fixed reference captured at the actual outward turnaround: `position = outward endpoint - pulses emitted on return`. An SW1 press anywhere therefore retraces the part of the schedule actually reached. Brief switch bounce pauses and resumes without changing the reference. Both directions use the same position-to-pulse table and the selected CLI speed.

Each confirmed demo limit produces a serial line such as:

```text
TURN: SW1; POSITION=158750; SERVO=2000 US; NEXT=SW4; DC=100%; HOME ERROR=0
```

`status` also shows `OUTWARD END`, `RETURN PULSES`, and `LAST HOME ERROR`. Home error is the outward pulse count minus the return pulse count when SW4 first triggers, less any limit overtravel (positive = fewer pulses on return). These are commanded pulses, not encoder measurements. Counts can reveal switch-position differences, but cannot directly detect mechanical backlash, servo lag, or missed motor steps. Do not shift only the return table: that would deliberately command a different path.

For a return mismatch, capture the outward TURN line and `status` shortly afterward while retracting. Compare the reported servo target to the observed mechanism before changing angle calibration or compensating for backlash.