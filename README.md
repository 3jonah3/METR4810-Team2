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
| `run` | Home to SW4, then repeat outward/return passes with the servo schedule and DC direction following travel. |
| `stop` | Stop all motion and disable servo PWM. |

### Servo

All servo commands use **servo 2**.

| Command | Action |
|---|---|
| `servo(90)` | Set angle; range **0–180 degrees**. |
| `servo(1500u)` | Set pulse width directly; range **500–2500 microseconds**. |
| `servo(status)` | Show nominal target angle and exact pulse width (not measured position). |
| `servo()` | Show help. |

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
- If both limit switches trigger together, correct the cause and reset the PCB to clear the fault.
- Closing PuTTY does **not** stop the robot.

## If UART does not work

Check **TX → RX**, **RX → TX**, common GND, the COM port and PuTTY settings. Make sure CubeIDE has not paused the code.

**PB9 flashes when receiving; PF1 flashes when transmitting.**

For more detail, see [SERIAL_CONTROL.md](METR4810_Team2_code/SERIAL_CONTROL.md).

## Modifying the code

Create a new branch before making changes. See [CONTRIBUTING.md](CONTRIBUTING.md).

Ask Jonah if you have any questions.

### Servo pulse widths in firmware

The servo mapping is 0 degrees = 500 us, 90 degrees = 1500 us, and 180 degrees = 2500 us at 50 Hz. Startup keeps PWM off. The CLI controls servo 2 (PB6); `servo(90)` selects degrees and `servo(1500u)` selects microseconds. Send one command per line. `stop` disables PWM; individual commands are rejected during autonomous run or a latched fault.

For C code using robot state and fault checks:

```c
Robot_SetServo(2, 90);             // Degrees; enables servo 2
Robot_SetServoPulseUs(2, 1500);    // Microseconds; enables servo 2
```

Both functions return NULL on success or an error string. The low-level driver also provides `servo(2, 90)` and `Servo_SetPulseUs(2, 1500)`; these return HAL status and require `Servo_Start(2)` to enable output after initialization. In C, `1500u` is an unsigned integer, not a unit: use the separate pulse-width function. Angles are nominal; confirm the mechanism can accommodate the commanded travel.
### Demo sequence (`run` or PC11/SW2)

1. Home to **SW4 / PC15** with the DC motor off and servo 2 at **1500 us (90 degrees)**. If SW4 is already pressed, debounce it without driving farther into the limit.
2. Set position to zero and push outward toward **SW1**, with DC duty **-100%**.
3. At SW1, reverse the stepper and DC motor (**+100%**) and follow the servo schedule backwards on the return to SW4.
4. At SW4, reset position to zero and repeat. `stop` or another run-button press stops everything. A new run always homes again.

Run preserves the selected stepper speed (16–2000 pulses/s). Set it while stopped:

```text
stop
stepper(speed,333)
run
status
```

333 is an example, not a demo speed setting. The existing startup default remains 500 pulses/s until changed; speed is not saved across reset. Position comes from emitted STEP pulses, not elapsed time or encoder feedback. Microstepping/mechanical calibration must match the supplied schedule. `status` reports homing/table phase, position from SW4, and servo pulse width.

| Position from SW4 (pulses) | Servo 2 (us) |
|---:|---:|
| 0–59,200 | 1500 |
| 70,000 | 1505 |
| 80,000 | 1520 |
| 90,000 | 1544 |
| 100,000 | 1578 |
| 110,000 | 1621 |
| 120,000 | 1673 |
| 130,000 | 1734 |
| 131,800 | 1745 |
| 140,000 | 1800 |
| 145,000 | 1833 |

Pulse widths are interpolated between adjacent rows and applied without a time-based ramp. The physical limit switches still trigger reversals; 145,000 is the table endpoint, not a forced reversal. Beyond it, the servo holds 1833 us until travel returns into the table. An earlier SW1 press reverses from the actual counted position. The SW4 return re-establishes zero every cycle. Independent `stepper(auto)` remains a stepper-only test and does not activate this servo/DC schedule.
### Checking the return path

The drill polarity for the demo is now **-100% outward toward SW1**, **+100% returning toward SW4**. Initial homing keeps the DC motor off. Manual `motor(duty)` commands retain their signed-duty meaning.

The return uses a fixed reference captured at the actual SW1 turnaround: `position = outward endpoint - pulses emitted on return`. An early SW1 press therefore retraces the part of the schedule reached, not a fresh path from 145,000. Brief switch bounce pauses and resumes without changing the reference. Both directions use the same position-to-pulse table and the selected CLI speed.

Each confirmed demo limit produces a serial line such as:

```text
TURN: SW1; POSITION=145000; SERVO=1833 US; NEXT=SW4; DC=100%; HOME ERROR=0
```

`status` also shows `OUTWARD END`, `RETURN PULSES`, and `LAST HOME ERROR`. Home error is the outward pulse count minus the return pulse count when SW4 triggers (positive = fewer pulses on return). These are commanded pulses, not encoder measurements. Counts can reveal switch-position differences, but cannot directly detect mechanical backlash, servo lag, or missed motor steps. An outward endpoint beyond 145,000 still has the original final-angle plateau; the TURN report makes that visible. Do not shift only the return table: that would deliberately command a different path.

For a return mismatch, capture the TURN line at SW1 and `status` shortly afterward while retracting. Compare the reported servo target to the observed mechanism before changing angle calibration or compensating for backlash.