# Contributing

## 1. Make a new branch

Always make a new branch before changing code. Start with your current work saved and committed.

```sh
git switch main
git pull origin main
git switch -c your-branch-name
```

Replace `your-branch-name` with a name like `fix-stepper`. The `-c` creates the branch and switches to it.

## 2. Put code in the right place

These files are in `METR4810_Team2_code/Core/Src/`:

| File | Purpose |
|---|---|
| `main.c` | Setup and main-loop calls. |
| `robot_control.c` | Run/run2, maintained switch control and stopping. |
| `serial.c` | CLI commands and help text. |
| `motor.c`, `servo.c`, `stepper.c`, `switch.c` | Hardware drivers. |

In generated files like `main.c`, only add code between `USER CODE BEGIN` and `USER CODE END`. Otherwise, CubeMX can overwrite it. Leave vendor files in `Drivers/` unchanged.

## 3. Write the autonomous routine

The **autonomous run is the code used for the demo**. The **UART CLI is generally used for testing** individual actuators and checking behavior.

Put the autonomous routine in `robot_control.c`:

- `start_auto()` starts the run from UART or the maintained switch (which selects run2).
- `Robot_Update()` runs the sequence. Put full-run actions under the `ROBOT_RUNNING` state.
- `Robot_Stop()` stops and cancels the sequence.

Avoid long delays or waiting loops so the stop command and switch keep working.

## 4. Test and share

Build in CubeIDE and test your changes, including starting and stopping. Update the README if commands or setup change.

```sh
git status
git add <changed-files>
git commit -m "Describe your changes"
git push -u origin your-branch-name
```

Replace `<changed-files>` with the files you changed. Open a pull request into `main` and briefly say what changed and what you tested.

For function contracts, formatting conventions and checks, see the
[firmware maintenance guide](docs/firmware-maintenance.md).
