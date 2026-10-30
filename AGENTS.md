# icarus_blimp_un

A Universidad Nacional de Colombia team's autonomous **blimp** for IAR-LARC 2026, raced at
Universidad Militar Nueva Granada (UMNG) in Bogotá, the host and not the team's university. It
must touch four tethered red balloons in order on two courses, with total thrust capped at 100 gf
(Rule 13). Since 2026-10-27 this repo holds everything: the flight controller, simulator and design
docs that were the DroneUN repo, and the bench firmware, ground station and reports of
icarus_blimp_un. README.md is the overview; docs/10-known-issues.md is the live status and
known-problems page.

This file is shared by every coding agent used on the project (Codex, Claude Code through
CLAUDE.md, and others). Keep it tool-neutral; tool-specific notes belong in that tool's own file.

## The vehicle

- It is a **blimp**: a spherical helium envelope carrying a gondola. Call it a blimp ("airship" is
  only the rulebook's umbrella word); never describe it as a drone or quadcopter.
- Controller board: Waveshare ESP32-S3-Zero (ESP32-S3FH4R2: 4 MB flash, 2 MB PSRAM), chosen
  2026-10-09. The ESP-FLY board (XIAO ESP32-S3 + MPU-6050, one-way MOSFET motor drive) was the
  previous board; its firmware shell is kept under `firmware/projects/espfly/` as a reference.
- Electronics, all on the bench and reading: two DRV8833 dual H-bridges, so every motor is
  reversible; stand-alone MPU-6050, QMC5883L (sold as HMC5883L), BME280 (sold as BMP280) and a
  VL53L1X on one I2C bus; a battery divider on ADC1; and AI-Thinker ESP32-CAM modules (OV2640,
  measured 58 x 45 degree lens) as the perception sensor. The lidar was ruled out as too heavy.
  Details: CLAUDE.md "Hardware facts that shape code" and docs/sensor-bringup-report.md.
- Four fixed brushed motors, named FL, FR, RL, RR in the driver, the link packets and `core/`.
  Surge pair FL, FR: push along +x, on the rear bar at +/-20 cm (surge, and yaw by running one
  faster). Heave pair RL, RR: push along +z, on the same bar at +/-5.5 cm (heave; their
  difference is a static roll trim at most). No servos, no sideways force. The DRV8833s allow
  reverse thrust, but `core/`'s mixer still assumes one-way motors; changing that is a core
  change, never a firmware workaround.
- Trimmed about 6 gf heavy, so motors off means a slow sink to the floor; that is the failsafe.
  Older docs that say it drifts upward are wrong.
- Thrust: one motor measured 14.7-16.8 gf on a lab supply, reverse about 74 % of forward.
  `core/src/params.c` still carries the 35 gf placeholder until a duty sweep through the DRV8833
  from the real cell replaces it. Never present simulated lap times as the real vehicle's.

## Layout

- `core/`: portable C11 flight controller. `blimp_step()` in `core/include/blimp/blimp.h` is the
  entire controller; tests are `core/tests/test_<module>.c`.
- `firmware/projects/i2c_tools/`: the live ESP-IDF project for the ESP32-S3-Zero: sensor and motor
  drivers, Wi-Fi/UDP link, console, and since 2026-10-27 the skeleton of the fixed-rate control
  task (`main/ctl_task.c`, middle stages empty) with the capture of its records
  (`main/capture.c`, in PSRAM, fetched over the link); plan and increments in
  `docs/control-loop-plan.md`, esp-drone reuse in `docs/reuse_log.md`. Once its stages are
  filled, it calls `blimp_step()`. Since 2026-10-30 it compiles `core/` as a component, and
  the `param` console command (`main/cmd_param.c`) reads and sets `blimp_params_t` live, in RAM.
- `firmware/projects/espfly/`: the previous board's shell. `main/main.c` is only an RTOS shell
  around `blimp_step()` (the esp_timer loop pattern); not part of day-to-day builds.
- `firmware/projects/optflow_cam/`: ESP32-CAM optical-flow experiment. `blink/`: toolchain test.
- `sim/`: MuJoCo closed loop that runs the same core, built as `build/libblimp.dll`, through
  ctypes (`sim/blimp_ffi.py`). `sim/scene/build_scene.py` generates the course from the rulebook.
- `software/`: ground tools: `blimp_link.py`, `mock_blimp.py`, `blimp_log.py` (decodes captures),
  the PID tuning plan's instruments `thrust_map.py` (the thrust stand), `step_test.py` (the step
  tests) and `step_response.py` (the fits), `ground_station/` (Flight, Control and Bench pages).
- `tools/`: build, verify, sweep and environment scripts, plus `lens_test/`. `scripts/idf.bat`: the
  ESP-IDF v5.5.5 wrapper for the S3-Zero projects on Windows.
- `docs/`: design and status (numbered files from DroneUN, dated reports from the bench);
  `docs/prompts/` is a local, gitignored folder of task prompts (not part of the repo; the plans
  they led to are `docs/control-loop-plan.md` and `docs/reuse_log.md`). `research/`: sources.
  `reference/`: download links
  (the rulebook PDF is not committed). `external/`: read-only third-party clones, gitignored.

## Commands

The host side is verified on Windows 11 and, since 2026-10-27, on Ubuntu 22.04: Python 3.12 and
clang on PATH. The controller library is `build/libblimp.dll` on Windows and `build/libblimp.so`
on Linux; the same seed gives different lap times on the two (the maths libraries round
differently and the closed loop amplifies it), so compare a platform's sweep with the same
platform's table, or compare across seeds. The S3-Zero firmware builds on Windows and Linux
with ESP-IDF v5.5.5 (see CLAUDE.md); the espfly shell needs v5.3.

```
python tools/setup_env.py                     # once: .venv with the pinned MuJoCo
python tools/verify.py                        # fast checks (~20 s), after any change
python tools/verify.py --full                 # adds both courses, with and without ToF (~1-2 min)
python tools/verify.py --firmware             # adds the espfly shell build (ESP-IDF v5.3)
python tools/verify.py --gui                  # adds the ground station's offscreen tests (~1 min)
python tools/build.py module <name>           # one module: pid params mixer estimator guidance blimp
python tools/build.py dll                     # rebuild build/libblimp.{dll,so} after any core/ change
python tools/sim_sweep.py                     # lap times across seeds 0-4, both courses, none + tof
.venv/Scripts/python sim/run_course.py --course 1 --max-time 220 --altitude-source tof --seed 3
.venv/bin/python sim/run_course.py ...        # the same on Linux
scripts\idf.bat i2c_tools build               # S3-Zero firmware on Windows; Linux: see CLAUDE.md
tools\idf_build.bat                           # espfly shell build; handles the ESP-IDF PATH traps
python tools/wifi_soak.py <label> --connection ICARUS-xxxxxx   # Linux: S3 link soak, counts drops
```

- Run anything under `sim/` with the `.venv` interpreter; the `tools/` scripts pick it up
  themselves.
- Always pass `--max-time 220` to `run_course.py`. The first 60 s are gyro-bias convergence, so
  the 150 s default usually stops before the third loop, which looks like a failure but isn't.
- One seed is one draw of the noise; judge flight changes across seeds (`tools/sim_sweep.py`).
- `run_course.py --envelope-diameter` overwrites the committed `sim/scene/course{1,2}.xml`. Use
  `tools/sim_sweep.py --envelope-diameter`, which writes its scenes to `build/sweep/`.
- The firmware scripts find Python 3.11 and ESP-IDF in the standard per-user locations; set the
  `PY311` and `IDF_PATH` environment variables if yours are elsewhere. `scripts\idf.bat` expects
  v5.5.5 at `%USERPROFILE%\esp\esp-idf-v5.5.5`; `tools\idf_build.bat` expects v5.3.

## Rules that must hold

- IMPORTANT: control, estimation and guidance logic lives only in `core/`. `sim/` synthesises
  sensors and applies actuator outputs. The firmware is the shell: sensor schedule, unit
  conversion, mode switch, failsafes, logging and motor output. Its estimate/control/mixer stages
  are placeholders until `blimp_step()` fills them; never grow them into a second controller. If
  the simulated blimp flies badly, fix `core/`; never compensate in `sim/` or in the firmware.
- `core/` is freestanding C11 plus `<math.h>`: no ESP-IDF, FreeRTOS, MuJoCo or stdio; no dynamic
  allocation, no mutable globals, no blocking; deterministic. Every tunable is a field of
  `blimp_params_t` with its default in `core/src/params.c`, never a file-scope constant.
- Never weaken or skip the Rule 13 sweep in `core/tests/test_mixer.c`; a passing `verify.py` is
  the evidence that total thrust stays at or under 100 gf.
- Changing a struct in `core/include/blimp/*.h` means updating its ctypes mirror and
  `_EXPECTED_LAYOUT` in `sim/blimp_ffi.py`; the import-time check against the DLL fails otherwise.
  A field added to `blimp_params_t` also needs its entry and accepted range in
  `firmware/projects/i2c_tools/main/param_table.c` (the S3's `param` command);
  `tools/fw_host_test.py`, run by `verify.py`, fails otherwise.
- `tools/build.py` compiles with `-std=c11 -Wall -Werror`. On Windows clang targets MSVC, so never
  add `-lm` there; on Linux `build.py` adds it itself (`LDLIBS`). Include the header that owns
  what you use (`<stddef.h>` for `NULL`): MSVC's headers pull in more than glibc's.
- Motor names and order: FL, RL, FR, RR is `motor_id_t` order in the S3-Zero driver and in the
  link packets; `core/` names them thrust_fl/fr/rl/rr. Never introduce M1..M4 or another order.
- In the S3-Zero firmware, persistent config goes in `sdkconfig.defaults*`, never in the gitignored
  `sdkconfig`; the existing console commands and the motor failsafe keep working.
- `BLIMP_BOOT_MOTOR_ID_MODE` in `firmware/projects/espfly/main/board_config.h` must be `0` in any
  build of that shell.
- `sim/scene/*.xml` files are generated; change `build_scene.py` and regenerate, never hand-edit.

## Which docs to trust

docs/10 and docs/07 §3 describe the current controller and simulator. The dated bench reports
(docs/sensor-bringup-report.md, docs/wireless-link.md, docs/ground-station-plan.md,
docs/esp32-cam-optical-flow-report.md) describe the current hardware. Much of docs/02-05, docs/07
outside §3 and docs/08 predates the four-motor design and the S3-Zero: they mention two tilting
rotors, servos, brushless motors, an upward-drifting failsafe, mochi-swarm-sim as the testbed, and
the ESP-FLY board. docs/sensor-fusion-plan.md (`blimp_ins`) is a proposal that was never
implemented; the estimator is `core/`'s. Check those against the code first.

The simulator is our own MuJoCo model. The rulebook's suggested mochi-swarm-sim is not used: it
models the two-rotor baseline blimp on a different course and runs only Python controllers.

## Skills

Multi-step workflows are written as Agent Skills (a `SKILL.md` per folder):

| Skill | Use it for |
|---|---|
| `core-change` | Changing the C flight controller in `core/` without breaking the simulator, the firmware or the Rule 13 cap |
| `sim-sweep` | Measuring lap times across seeds and refreshing the results tables |
| `flash-firmware` | Building and flashing the previous board's shell (`firmware/projects/espfly`); only on an explicit request, because it drives hardware. The S3-Zero firmware builds with `scripts/idf.bat` as CLAUDE.md describes |

Codex loads them from `.agents/skills/` (mention one with `$sim-sweep`, or pick from `/skills`);
Claude Code loads them from `.claude/skills/` (`/sim-sweep`). The `.claude/skills/` files are the
originals: edit those, then run `python tools/sync_skills.py` to regenerate the Codex copies
(`tools/verify.py` fails while they are out of date). Keep skill text tool-neutral: no
tool-specific placeholders or invocation syntax. If your agent does not load skills, open the
matching `SKILL.md` and follow it as a checklist.

## Conventions

- Comments explain why: the constraint, the rulebook rule number, what was measured. Match that.
- Cite rulebook rules by number ("Rule 13") and keep units in names and comments (`_m`, `_n`, gf).
- Keep paths repo-relative in code and docs; no machine-specific paths.
- When simulated results change, update the tables in README.md, docs/10 and docs/09 together,
  and state the settings used (seeds, `--max-time`, envelope).
- Bench reports are dated and keep "measured" apart from "reasoned"; raw session notes go in
  `docs/history/`.
