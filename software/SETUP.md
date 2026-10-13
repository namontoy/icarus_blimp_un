# Setting up the Python tools (laptop)

The tools in this folder (`drone_gui.py`, `drone_dynamics.py`, `motor_diag.py`) were written on a second PC for the esp-drone firmware, at `esp-drone@560e17b`. Their protocol, design and known issues are in `docs/esp-drone-tools-report.md`; the raw session reports are in `docs/history/`.

## 1. Conda environment

```bash
conda create -n blimp-tools python=3.10 -y
conda env config vars set PYTHONNOUSERSITE=1 -n blimp-tools
conda activate blimp-tools
pip install -r software/requirements.txt
```

**Why `PYTHONNOUSERSITE=1`:** without it, the env also sees packages in `~/.local`. On 2026-10-13 a plain `pip install` in this env uninstalled `numpy 2.2.6` from `~/.local`, which broke other Python work (OpenCV, matplotlib), and it skipped installing `scipy` because it found one in `~/.local`. Set the variable *before* installing anything.

`pip check` may print a `launch-ros` warning. It comes from ROS 2 packages visible through `PYTHONPATH` and is unrelated to these tools.

## 2. Checks that need no hardware (passed on 2026-10-13)

```bash
python software/drone_dynamics.py        # prints: drone_dynamics self-check OK
python software/motor_diag.py --help     # loads cflib, prints the options
QT_QPA_PLATFORM=offscreen python -c "import sys; sys.path.insert(0, 'software'); import drone_gui; print('ok')"
```

## 3. Things to know before opening the GUI

- **Qt on Linux:** Qt 6.5+ may need `sudo apt install libxcb-cursor0`. The symptom is an error about the `xcb` platform plugin.
- **NetworkManager:** the GUI joins the drone's Wi-Fi with `nmcli`. The laptop **loses internet** while connected (one Wi-Fi interface).
- **cfclient:** close it first. Both use local UDP port 2399, so only one program can hold the link.
- **Password constant:** `WIFI_PASSWORD` in `drone_gui.py` must match the firmware's `CONFIG_WIFI_PASSWORD`.
- **Bench use only:** the esp-drone fork has tumble detection disabled (`SITAW_TU_ENABLED`, commit `560e17b`), and there is no firmware watchdog on `motorPowerSet`. If the link drops while a motor is enabled, the motor keeps running until reset.

## 4. Hardware checklist (not done yet: needs the drone)

Prepare: remove propellers, power the drone up **lying still** and wait for gyro calibration, close cfclient.

1. **Read-only link test:** `python software/motor_diag.py` (no `--spin`). Expect `selftestPassed=1`, `mpu6050=1`, `stop=0`, `armed=1`, `acc.z` about +1.0, `roll` about 0, `rtStab` about 1000. `rtStab = 0` means the stabilizer is waiting for calibration.
2. **GUI connect, disconnect, reconnect** from the laptop. This is a new host and the `nmcli` permission check (polkit) was only tested on PC2. Telemetry should keep flowing for more than 1 s, which proves the keep-alive of the pinned `cflib` fork works.
3. **One motor at low duty** (limit 30 %, no propeller): check the 8-bit readback under the slider, that STOP works (also Space and Esc), and that the motor stops when you disconnect cleanly.
4. **Not yet verified on any hardware:** ALL-slider mode, the top-view orientation against the real motor wiring (M3 alone should light up front-right), and the tendency arrows.
5. **Record results** here or in a new file in `docs/`, with the date and the firmware commit.
