# DroneUN

A Universidad Nacional de Colombia team's autonomous **blimp** for IAR-LARC 2026, raced at
Universidad Militar Nueva Granada (UMNG) in Bogotá, the host and not the team's university. It
must touch four tethered red balloons in order on two courses, with total thrust capped at 100 gf
(Rule 13).
README.md is the overview; docs/10-known-issues.md is the live status and known-problems page.

This file is shared by every coding agent used on the project (Codex, Claude Code through
CLAUDE.md, and others). Keep it tool-neutral; tool-specific notes belong in that tool's own file.

## The vehicle

- It is a **blimp**: a spherical helium envelope carrying a gondola. Call it a blimp ("airship" is
  only the rulebook's umbrella word); never describe it as a drone or quadcopter.
- Controller board: ESP-FLY (XIAO ESP32-S3 + MPU-6050). ESP-FLY names the board, not the vehicle.
- Four fixed, one-way brushed motors. Front pair (FL, FR) push forward: surge, and yaw by running
  one faster. Rear pair (RL, RR) push up: heave. No servos, no reverse thrust, no sideways force.
- Trimmed about 6 gf heavy, so motors off means a slow sink to the floor; that is the failsafe.
  Older docs that say it drifts upward are wrong.
- Still undecided, so never assume them: the perception sensor (no camera; a 360° lidar is being
  evaluated), the motor drivers (H-bridges would make the motors reversible), and real motor
  thrust (35 gf per motor is a placeholder until a thrust stand measures it).

## Layout

- `core/`: portable C11 flight controller. `blimp_step()` in `core/include/blimp/blimp.h` is the
  entire controller; tests are `core/tests/test_<module>.c`.
- `firmware/`: ESP-IDF project for the ESP32-S3. `main/main.c` is only an RTOS shell around
  `blimp_step()`; pins and boot flags are in `main/board_config.h`.
- `sim/`: MuJoCo closed loop that runs the same core, built as `build/libblimp.dll`, through
  ctypes (`sim/blimp_ffi.py`). `sim/scene/build_scene.py` generates the course from the rulebook.
- `tools/`: build, verify, sweep and environment scripts. `docs/`: design and status.
  `research/`: sources. `reference/`: download links (the rulebook PDF is not committed).

## Commands

Windows 11 is the verified platform: Python 3.12, clang on PATH, and ESP-IDF v5.3 (firmware only).

```
python tools/setup_env.py                     # once: .venv with the pinned MuJoCo
python tools/verify.py                        # fast checks (~20 s), after any change
python tools/verify.py --full                 # adds both courses, with and without ToF (~1-2 min)
python tools/verify.py --firmware             # adds the ESP32-S3 build
python tools/build.py module <name>           # one module: pid params mixer estimator guidance blimp
python tools/build.py dll                     # rebuild build/libblimp.dll after any core/ change
python tools/sim_sweep.py                     # lap times across seeds 0-4, both courses, none + tof
.venv/Scripts/python sim/run_course.py --course 1 --max-time 220 --altitude-source tof --seed 3
tools\idf_build.bat                           # firmware build; handles the ESP-IDF PATH traps
```

- Run anything under `sim/` with the `.venv` interpreter; the `tools/` scripts pick it up
  themselves.
- Always pass `--max-time 220` to `run_course.py`. The first 60 s are gyro-bias convergence, so
  the 150 s default usually stops before the third loop, which looks like a failure but isn't.
- One seed is one draw of the noise; judge flight changes across seeds (`tools/sim_sweep.py`).
- `run_course.py --envelope-diameter` overwrites the committed `sim/scene/course{1,2}.xml`. Use
  `tools/sim_sweep.py --envelope-diameter`, which writes its scenes to `build/sweep/`.
- The firmware scripts find Python 3.11 and ESP-IDF in the standard per-user locations; set the
  `PY311` and `IDF_PATH` environment variables if yours are elsewhere.

## Rules that must hold

- IMPORTANT: control, estimation and guidance logic lives only in `core/`. `sim/` synthesises
  sensors and applies actuator outputs; `firmware/main/main.c` only calls `blimp_step()`. If the
  simulated blimp flies badly, fix `core/`; never compensate in `sim/`.
- `core/` is freestanding C11 plus `<math.h>`: no ESP-IDF, FreeRTOS, MuJoCo or stdio; no dynamic
  allocation, no mutable globals, no blocking; deterministic. Every tunable is a field of
  `blimp_params_t` with its default in `core/src/params.c`, never a file-scope constant.
- Never weaken or skip the Rule 13 sweep in `core/tests/test_mixer.c`; a passing `verify.py` is
  the evidence that total thrust stays at or under 100 gf.
- Changing a struct in `core/include/blimp/*.h` means updating its ctypes mirror and
  `_EXPECTED_LAYOUT` in `sim/blimp_ffi.py`; the import-time check against the DLL fails otherwise.
- `tools/build.py` compiles with `-std=c11 -Wall -Werror`. On Windows clang targets MSVC, so never
  add `-lm`.
- `BLIMP_BOOT_MOTOR_ID_MODE` in `firmware/main/board_config.h` must be `0` in any flight build.
- `sim/scene/*.xml` files are generated; change `build_scene.py` and regenerate, never hand-edit.

## Which docs to trust

docs/10 and docs/07 §3 describe the current blimp. Much of docs/02-05, docs/07 outside §3 and
docs/08 predates the four-motor design: they mention two tilting rotors, servos, brushless motors,
an upward-drifting failsafe, and mochi-swarm-sim as the testbed. Check those against the code first.

The simulator is our own MuJoCo model. The rulebook's suggested mochi-swarm-sim is not used: it
models the two-rotor baseline blimp on a different course and runs only Python controllers.

## Skills

Multi-step workflows are written as Agent Skills (a `SKILL.md` per folder):

| Skill | Use it for |
|---|---|
| `core-change` | Changing the C flight controller in `core/` without breaking the simulator, the firmware or the Rule 13 cap |
| `sim-sweep` | Measuring lap times across seeds and refreshing the results tables |
| `flash-firmware` | Building and flashing the ESP32-S3; only on an explicit request, because it drives hardware |

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
