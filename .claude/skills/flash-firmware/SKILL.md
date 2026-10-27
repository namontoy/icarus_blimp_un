---
name: flash-firmware
description: Build and flash the previous board's flight shell (firmware/projects/espfly) to an ESP-FLY board on Windows, optionally in motor-identification mode. Not for the live ESP32-S3-Zero firmware. Touches real hardware, so it runs only when a person explicitly asks for it.
disable-model-invocation: true
argument-hint: "[COM port] [motor-id]"
---

# Flash the firmware

This skill flashes the previous board's shell, `firmware/projects/espfly/` (XIAO ESP32-S3 on the
ESP-FLY PCB, ESP-IDF v5.3). The live ESP32-S3-Zero firmware, `firmware/projects/i2c_tools/`, builds
and flashes as CLAUDE.md describes (`scripts\idf.bat i2c_tools build` on Windows) and has no
motor-identification mode; if the request is about that board, say so and stop.

The request may name a COM port, and may ask for motor-identification mode.

This drives real hardware. Before any step that can spin a motor, confirm with the person.

## 1. Check what will be flashed

- Read `BLIMP_BOOT_MOTOR_ID_MODE` in `firmware/projects/espfly/main/board_config.h`. It must be `0` unless the
  person asked for motor-identification mode. Say which mode is about to be flashed.
- Run `git status`. Flashing uncommitted code is allowed, but mention it in the report so the
  flashed build can be traced.

## 2. Build

Run `tools\idf_build.bat` from cmd or PowerShell. It works around the four documented
Windows/ESP-IDF traps: PowerShell's stderr wrapping, the Windows Store Python, `export.bat`
needing `PROMPT`, and `MSYSTEM` inherited from Git Bash. Report the "binary size ... free" line.
If ESP-IDF is not installed, stop and point to `docs/08-firmware-setup.md`.

## 3. Find the port

Use the port from the arguments if one was given. Otherwise list serial ports in PowerShell:

```
Get-PnpDevice -Class Ports -PresentOnly | Select-Object FriendlyName, Status
```

The XIAO ESP32-S3 shows up as a USB serial or USB JTAG device on a COM port. If there are none or
several, ask the person which one.

## 4. Flash

From PowerShell, substituting the port:

```
cmd /c 'tools\idf_build.bat "-p COM5 flash"'
```

If it times out, close anything holding the port (an open monitor, Arduino IDE's serial monitor).
If it still fails, have the person hold BOOT, tap RESET, release BOOT, then retry.

## 5. Monitor (the person runs this)

`idf.py monitor` is interactive, so it has to run in the person's own terminal:

```
. .\tools\idf_env.ps1
cd firmware\projects\espfly
idf.py -p COM5 monitor
```

It exits with Ctrl+]. On the first boot, check which sensors acknowledged on the I2C bus (the
`imu`, `baro` and `mag` log tags) before any task starts.

## Motor-identification mode

Used to find out which physical motor is wired to which position. Never a flight build.

1. Propellers off, or the blimp restrained: confirm this with the person before flashing.
2. Set `BLIMP_BOOT_MOTOR_ID_MODE` to `1`, then build and flash as above.
3. After a 5 s pause, the firmware spins each motor alone at 20 % duty for 1.5 s, in the order FL,
   FR, RL, RR, forever. The person writes down which physical motor moves for each name.
4. Set the flag back to `0`, then rebuild and reflash a normal build. Never leave the repository
   with the flag at `1`.
5. If the observed mapping differs from the pin assignment in `board_config.h`, update
   `board_config.h` and record the change in `docs/10-known-issues.md` §6.
