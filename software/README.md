# Bench test tools

## For this repo's firmware: `blimp_link.py`

The S3 bench firmware (`firmware/projects/i2c_tools`) is a Wi-Fi access point; `blimp_link.py`
is its ground tool, standard library only, any OS. Join the blimp's SSID (`ICARUS-xxxxxx`),
then:

```
python software/blimp_link.py selftest                     # no hardware
python software/blimp_link.py status                       # telemetry
python software/blimp_link.py console                      # the firmware console, over the air
python software/blimp_link.py motor 0.2 0 0 0 --seconds 2  # FL RL FR RR, PROPS OFF
python software/blimp_link.py stop
```

`mock_blimp.py` serves the same protocol on localhost for development without hardware
(`--host 127.0.0.1` on the commands above). Protocol, safety timers and the bring-up
checklist: `docs/wireless-link.md`.

## esp-drone tools (not for this repo's firmware)

Everything below is about the Python tools imported from the esp-drone project.

Python tools for testing the ESP-Drone (ESP32-S3 mini quad) on the bench over its Wi-Fi link, without cfclient:

| File | What it is |
|---|---|
| `drone_gui.py` | PyQt6 GUI: connect/disconnect, drive the 4 motors, live MPU-6050 data, battery, top view of the quad |
| `drone_dynamics.py` | Pure-Python model used by the GUI to draw the open-loop XY / yaw tendency from the motor duties |
| `motor_diag.py` | Command-line read-out of key params/logs, with an optional 3 s spin of one motor |

> **Remove the propellers before moving any motor slider.** These tools write
> `motorPowerSet.*` directly, which bypasses the flight controller.

## Requirements

- **Linux with NetworkManager.** The GUI uses `nmcli` to join the drone's access point.
- **The cfclient virtualenv**, which already provides:
  - `cflib`, with the esp-drone UDP driver: port 2390, checksum byte and keep-alive;
  - PyQt6;
  - pyqtgraph.
- **Firmware** from this repo, with the drone access point at its defaults:
  - SSID `ESP-DRONE_<MAC>`, password `12345678`;
  - drone IP `192.168.43.42`.

Close cfclient before starting. Both use local UDP port 2399, so only one can be connected at a time.

## Running

```bash
python software/drone_gui.py
```

It works from any directory. It imports `drone_dynamics.py` from its own folder.

**Power the drone up lying flat and still.** The firmware calibrates the gyro before starting the stabilizer loop. Until it succeeds, sensor values read 0 and the motors ignore every command.

## Using the GUI

The GUI labels are in Spanish.

### Connection bar
- **Conectar / Desconectar**
  - Connecting joins the strongest `ESP-DRONE_*` network. If the PC is already on one, it skips this step.
  - It then waits for a `192.168.43.x` address and opens the CRTP link.
  - Disconnecting zeroes the motors, closes the link, and leaves the Wi-Fi network, but only if the GUI joined it itself. The Wi-Fi profile the GUI creates is set to not auto-connect.
- **Status:**
  - `Buscando Wi-Fi` → `Asociando` → `Abriendo CRTP` → `Conectado`;
  - otherwise, `Error: ...` in red.
- **Info:** `selftestPassed` and `mpu6050` from the firmware self-test.
- **Batería:** charge % bar, voltage and power state (`En bateria`, `Cargando`, `Cargada`, `Bateria baja`, `Apagando`).
  - Colours: red below 20 % or in `Bateria baja`, orange up to 49 %, green from 50 %.
  - The level comes from `pm.batteryLevel` in 10 % steps.

### Motores (PWM)
- **Habilitar motores:** turns `motorPowerSet.enable` on or off. While it is off, the sliders don't reach the motors.
- **Mode:**
  - **Individual:** the M1–M4 sliders drive each motor.
  - **Todos (M1-M4):** the **ALL** slider drives the four together, and the individual sliders follow it (greyed out).
  - Switching mode sets every motor to 0 first.
- **Limite max %:** caps every slider. It is 30 % by default.
- **Readback:** under each slider, `pwm: N/255` is the duty the drone reports it is applying. The motor driver uses 8-bit PWM.
- **STOP:** also bound to **Space** and **Esc**. It sets all motors to 0 and turns *Habilitar motores* off.
- **How commands are sent:** changes go out at most every 50 ms per motor, and the last value always arrives.

### Vista superior
Top view of the quad, front at the top:

| Motor | Position | Propeller |
|---|---|---|
| M1 | rear-left | CW |
| M2 | front-left | CCW |
| M3 | front-right | CW |
| M4 | rear-right | CCW |

- **Per motor:**
  - the **applied** duty % (from the PWM log);
  - the requested % (`pedido`);
  - a curved arrow showing the rotation. It is grey at 0, then green, orange and red as the duty rises.
- **At the centre**, drawn from `drone_dynamics.py`:
  - **Blue arrow:** where the bare airframe would tend to drift in the XY plane.
  - **Purple arc:** which way it would tend to yaw.
  - **Caption below:** these values as absolute % of the maximum possible imbalance.
- Arrow length is scaled to the largest imbalance reachable within *Limite max %*, so the arrows stay readable at bench duties.

### MPU-6050
- **Values:** acc [g], gyro [deg/s], roll/pitch/yaw [deg], |acc| (≈ 1 g at rest), armed / canfly.
- **Estabilizador:** the stabilizer loop rate. It is ≈ 1000 Hz when running.
- **Plots:** 10 s rolling plots of the accelerometer and gyroscope.

### Warnings and watchdogs
| Message | Meaning / what to do |
|---|---|
| Red banner `Estabilizador detenido (0 Hz)` | The stabilizer loop isn't running, so sensors read 0 and motors ignore commands. Usually the gyro calibration hasn't finished: leave the drone flat and still, or power-cycle it that way. |
| `Error: sin datos del drone` | Connected, but no log data for 2 s. The drone rebooted, powered off or went out of range. |
| `Error: el drone no responde (CRTP)` | Wi-Fi is up, but the CRTP link didn't finish opening within 15 s. |

## `drone_dynamics.py` model

**Model:**
- **Thrust per motor:** `T = (duty / 100)²`, since propeller thrust ∝ rpm².
- **XY:** the drone moves along `−Σ pᵢ·Tᵢ`, where `pᵢ` is the motor position. A stronger side tilts the drone away from it, so it drifts toward the weaker side.
- **Yaw:** `(T_M1 + T_M3) − (T_M2 + T_M4)`. A CW propeller pushes the body CCW, so + means CCW.

This is the **open-loop** tendency. In flight the stabilizer counteracts it, so it's a teaching aid, not a flight prediction.

Run the file directly for a self-check:
```bash
python software/drone_dynamics.py
```

## `motor_diag.py`

```bash
PY=python
$PY software/motor_diag.py                      # read-only: params + 5 log samples
$PY software/motor_diag.py --spin 3             # spin M3 at raw 10000 for 3 s
$PY software/motor_diag.py --spin 3 --duty 25   # spin M3 at 25 % for 3 s
```

`stabilizer.rtStab` in its output is the quickest way to tell whether the stabilizer loop is running.

## Known limitations

- **No firmware watchdog on `motorPowerSet`:** if the Wi-Fi drops while motors are enabled, they keep their last value until the drone is reset. Keep the propellers off.
- **Local UDP port:**
  - cflib's UDP driver always binds local port 2399.
  - The GUI adds a receive timeout to that driver, so the port is released on disconnect and reconnecting works.
  - Other scripts started while the GUI is connected will fail with `Address already in use`.
