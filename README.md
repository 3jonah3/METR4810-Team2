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

## 3. Flash the PCB

1. Connect ST-LINK to the PCB's **J24** header: **SWDIO → IO**, **SWCLK → CLK**, **NRST → RST**. The programming header is missing a GND pin, so connect the **black ST-LINK ground wire to GND on the spare GPIO header**, as shown below. Power the PCB.
2. In CubeIDE, select **METR4810_Team2_code Debug** and click **Debug** to flash.
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
| `run` | Run the DC motor at 100% and move the stepper between its limits. |
| `stop` | Stop all motion and disable servo PWM. |

### Servo

All servo commands use **servo 2**.

| Command | Action |
|---|---|
| `servo(90)` | Set angle; range **0–120 degrees**. |
| `servo(status)` | Show the last target angle. |
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
