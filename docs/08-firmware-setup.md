# Firmware setup (Windows)

> **UNCOMPILED.** Everything in `firmware/` and this document was written on a machine with
> **no ESP-IDF and no xtensa toolchain installed** — none of it has been built, flashed, or run.
> It follows documented ESP-IDF v5.x conventions as closely as could be verified without a
> compiler; every place a specific API or mechanism could not be fully verified is flagged
> `UNVERIFIED`/`CONFIDENCE FLAG` in the code and repeated in section 6 below. **The first person
> to actually run `idf.py build` should expect to fix a few things**, and should update those
> flagged comments (remove them, or correct them) once verified, so the flag doesn't linger as
> stale caution after the fact.

This document covers: installing ESP-IDF v5.x on Windows, opening a build shell, building,
flashing, monitoring, and running the host-side (non-firmware) unit tests. It also documents the
firmware's failsafe behaviour, since that is safety-relevant and easy to get backwards if
approached with quadcopter intuition.

---

## 1. Install ESP-IDF v5.x on Windows

Two supported paths. Either is fine; **pick one and be consistent** — do not mix an
installer-based IDF_PATH with a manually-cloned one on the same machine without being deliberate
about `%IDF_PATH%`.

### 1a. Official Windows installer (recommended, simplest)

1. Download the **ESP-IDF Windows Installer** from Espressif's own site
   (`https://dl.espressif.com/dl/esp-idf/`). Pick the newest **v5.x** offline or online installer
   (not v4.x — this project targets v5.x conventions throughout, see `firmware/CMakeLists.txt`'s
   own header comment).
2. Run it. Accept the defaults unless there's a specific reason not to. It installs:
   - The ESP-IDF source tree (default `C:\Espressif\frameworks\esp-idf-vX.Y`)
   - The xtensa/riscv toolchains, CMake, Ninja, Python, and a Python venv with the required
     packages, all under `C:\Espressif\`
   - Start Menu shortcuts for an **"ESP-IDF PowerShell"** and **"ESP-IDF Command Prompt"**, each
     of which launches a shell with the environment already exported (this replaces the manual
     `export.ps1` step in 1b below).
3. Verify: open **"ESP-IDF PowerShell"** from the Start Menu and run:
   ```powershell
   idf.py --version
   ```
   This should print something like `ESP-IDF v5.x.y`. If it does not, the installer did not
   complete correctly — re-run it before going further.

### 1b. Manual install (more control, more steps)

1. Install prerequisites: Git, Python 3.11+ (from python.org, not the Windows Store version —
   the Store version's sandboxing has caused real, documented problems with ESP-IDF's installer
   scripts), and confirm both are on `PATH`.
2. Clone ESP-IDF at a specific `v5.x` release tag (do not track a moving branch for a competition
   build — pin an exact tag so a rebuild six weeks from now uses the same toolchain):
   ```powershell
   cd C:\Espressif
   git clone -b v5.3 --recursive https://github.com/espressif/esp-idf.git
   ```
   (Substitute whichever v5.x release is current and stable when this is actually run — v5.3 is
   an example, not a mandate. `--recursive` matters: ESP-IDF vendors several submodules.)
3. Install the toolchain **for this project's target only**, to save time/disk:
   ```powershell
   cd C:\Espressif\esp-idf
   .\install.ps1 esp32s3
   ```

### Which ESP-IDF v5.x minor version, specifically

This project's firmware code was written against **general v5.x conventions** without pinning an
exact minor version, because no toolchain was available to verify version-specific API details
against. Two files flag this explicitly and need re-checking against whichever version is
actually installed:

- `firmware/main/drivers/motor.c` — uses the newer `driver/mcpwm_prelude.h` MCPWM driver
  (replacing the pre-v5.0 `driver/mcpwm.h`). Diff it against
  `$IDF_PATH\examples\peripherals\mcpwm\mcpwm_servo_control\main\mcpwm_servo_control_example_main.c`
  in whatever version gets installed before trusting it near a spinning propeller.
- `firmware/main/net/espnow_link.c` — assumes the ESP-NOW receive-callback signature that takes
  an `esp_now_recv_info_t *`, introduced partway through the v5.x line. If the installed version
  predates that change, `esp_now.h`'s `esp_now_recv_cb_t` typedef will show the older
  `(const uint8_t *mac_addr, ...)` signature instead, and `recv_cb()`'s signature needs to match.

Also flagged, lower-stakes: `firmware/main/drivers/imu.c`/`baro.c`/`mag.c` use the **legacy**
`driver/i2c.h` API rather than the newer `driver/i2c_master.h` bus/device API introduced in
v5.2 — a deliberate choice (the legacy API's signatures are stable across the whole v5.x line and
were more confidently recalled without a compiler to check against), not an oversight. Migrating
to the new driver is a reasonable improvement once real hardware is in hand, not a blocker.

---

## 2. Open a shell with the ESP-IDF environment exported

Every terminal session that runs `idf.py` needs the environment set up first. Two ways to get
there, matching the two install paths above:

- **Installer path:** open **"ESP-IDF PowerShell"** from the Start Menu. Done — this shortcut
  runs the export step for you every time it opens.
- **Manual-install path:** in an ordinary PowerShell window:
  ```powershell
  C:\Espressif\esp-idf\export.ps1
  ```
  This sets `IDF_PATH`, adds the toolchain and `idf.py` to `PATH` for **this shell session
  only** — it does not persist, and does not modify system environment variables. Run it again
  in every new terminal window before using `idf.py`.

  (PowerShell's execution policy may block running `.ps1` scripts by default. If so, either
  launch via `powershell -ExecutionPolicy Bypass -File C:\Espressif\esp-idf\export.ps1`, or set a
  less restrictive policy for the current user — `Set-ExecutionPolicy -Scope CurrentUser
  RemoteSigned` — once, rather than bypassing it every time.)

Confirm the export worked:
```powershell
idf.py --version
where.exe idf.py
```

---

## 3. First build

From a shell with the environment exported:

```powershell
cd firmware                      # from the repository root
idf.py set-target esp32s3
idf.py build
```

`idf.py set-target esp32s3` is the unambiguous, always-correct way to select the target — run it
explicitly the first time even though `firmware/CMakeLists.txt`'s `set(IDF_TARGET esp32s3)` and
`firmware/sdkconfig.defaults`'s `CONFIG_IDF_TARGET="esp32s3"` are both meant to make this
automatic (see that file's own comments on why both exist and why neither is fully verified).
`set-target` generates/overwrites `sdkconfig` from `sdkconfig.defaults` plus the target's own
defaults — safe to run again any time something about the target config seems wrong.

**What to check in the build output the first time**, specifically because this tree has never
been built before:

1. Does the component list (printed near the top of the configure step, or visible via
   `idf.py build -v` / in `build/CMakeCache.txt` as `BUILD_COMPONENTS`) include a component
   literally named **`core`**? If not, see `firmware/CMakeLists.txt`'s `EXTRA_COMPONENT_DIRS`
   comment for the documented fallback.
2. Does `core`'s sources compile clean? `core/CMakeLists.txt` deliberately does **not** pass
   `-Werror` (see its own comment on why), so a warning here will not fail the build but is worth
   reading — `tools/build.py`'s host build treats every one of these as `-Werror`, so a genuine
   warning here is a real bug the host build should also have caught, unless it's an
   xtensa-gcc-specific false positive.
3. Does `firmware/main/drivers/motor.c` compile at all? This is the file this document's
   section 1 flags as lowest-confidence. If it fails, diff it against the real
   `mcpwm_servo_control` example as instructed above.

---

## 4. Flashing

Connect the board over USB. Find its COM port (Device Manager, or `idf.py -p PORT flash` will
often auto-detect if only one relevant device is attached):

```powershell
idf.py -p COM5 flash
```

(Substitute the actual port. If flashing fails with a permissions/timeout error, close any other
program that might be holding the serial port open — a stale `idf.py monitor` session, a
terminal emulator, Arduino IDE's serial monitor, etc. — and try again; ESP32-S3's bootloader
generally auto-resets into download mode without needing a manual boot-button sequence, but if it
doesn't, hold **BOOT**, tap **RESET**, release **BOOT**, then retry.)

Flash and immediately open the serial monitor in one step:
```powershell
idf.py -p COM5 flash monitor
```

---

## 5. Monitoring

```powershell
idf.py -p COM5 monitor
```

Exit with `Ctrl+]`. `idf.py monitor` also decodes backtraces from a crash/panic into
function+line if the ELF file from the matching build is present (it is, right after a build) —
worth reading the very first boot log carefully rather than skimming past it: `main.c`'s
`app_main()` logs which sensors ACKed on the I2C bus (`imu`/`baro`/`mag` tags) before creating any
task, which is the fastest way to learn whether `board_config.h`'s placeholder addresses are even
in the right ballpark for the real board.

---

## 6. Running the host-side tests (not firmware)

`core/` is portable C11 and is unit-tested **on this machine**, independent of ESP-IDF, via
`tools/build.py`. This has nothing to do with sections 1–5 above — no ESP-IDF environment needs
to be exported for this:

```powershell
# from the repository root
python tools/build.py module pid
python tools/build.py module params
```

Use `module <name>` for whichever module is being worked on — **never `all`** while multiple
people/agents may be mid-write in the same tree (`tools/build.py`'s own doc comment). This is the
build that actually runs today, produces a `PASS`/`FAIL` line and a check count, and is the one
to run after any change to `core/`. The ESP-IDF build in sections 1–5 compiles the *same*
`core/src/*.c` files (via `core/CMakeLists.txt`) but cannot be run at all on a machine without
the toolchain — `tools/build.py` is the only build in this repository that is actually verified
green right now.

---

## 7. Failsafe behaviour

**A neutrally-buoyant airship does not fall when thrust is cut — it drifts and slowly rises.**
This is the opposite of a multirotor's failure mode, and the firmware is built to exploit it
rather than port a multirotor's "controlled descent" instinct (docs/03-software-architecture.md
section 1.4, docs/07-control-design.md section 4.4):

- **Trigger:** `blimp_notify_link()` (core/include/blimp/blimp.h) is called from `control_task`
  every time a new, CRC-valid command packet has arrived since the last control tick (see
  `main.c`'s link-snapshot handling). `blimp_step()` internally compares the time since the last
  such call against `blimp_params_t.link_timeout_s` (1.0 s default, `core/src/params.c`). Losing
  the ESP-NOW link for longer than that — not losing WiFi entirely, just this one heartbeat —
  is what trips the failsafe.
- **What happens on trigger:** guidance transitions to `BLIMP_GUID_FAILSAFE`
  (core/include/blimp/guidance.h) and the wrench demand this firmware then hands to the mixer
  goes to (or toward) zero — **the vehicle holds altitude and stops translating**, rather than
  attempting to descend. Concretely: no forward thrust, no yaw torque, a neutral or near-neutral
  heave command; the vehicle drifts with whatever residual velocity and ambient air currents it
  already had, slowly rising if anything (near-neutral buoyancy, trimmed to +0..+10 gf net lift
  in the worst case per Rule 11's tolerance band), and telemetry keeps streaming over ESP-NOW if
  the radio itself is still functional (only the *command* direction went stale — the *telemetry*
  direction is unidirectional and independent, see `net/protocol.h`).
- **What this firmware does NOT yet implement:** a battery-critical failsafe trigger and an
  attitude-sanity fault trigger (both named in docs/07 section 4.4 alongside link-loss) are not
  wired into `main.c` — `battery_read_voltage()` exists and logs a value at boot, but nothing
  currently acts on `BLIMP_BOARD_BATT_LOW_V` (board_config.h) during flight. This is a real gap,
  not an oversight to ignore: add a periodic check in `control_task` (or a dedicated low-priority
  task) that calls `blimp_notify_link()`'s sibling concept — there is no
  `blimp_notify_battery_critical()` in the current core/include/blimp/blimp.h contract, so this
  needs either a core API addition (out of scope for this firmware-only pass — flag it to
  whoever owns core/) or a firmware-side decision to force `arm_request = false` /command a
  motor all-stop directly via `blimp_motor_all_stop()` (motor.h) when voltage drops below
  threshold.
- **Pre-flight gate, not a mid-flight catch:** the buoyancy/actuator sanity checks
  (docs/07-control-design.md section 7, bring-up steps 1–2) are meant to **block arming** in the
  first place if the vehicle is mis-trimmed, rather than relying on an in-flight failsafe to catch
  it — this firmware does not yet implement an ARM gate at all (`blimp_input_t.arm_request` is
  passed straight through from the ground command; whatever gating logic core/'s own
  `blimp_step()`/guidance implementation applies to it is that module's responsibility, not
  main.c's).

---

## 8. Known gaps in this skeleton (do not rediscover these from scratch)

> **Updated 21 Sep 2026.** `drivers/servo.*` no longer exists (the airframe has no servo);
> `motor.c` is LEDC PWM into four MOSFET gates with a boot-time motor identification mode
> (`BLIMP_BOOT_MOTOR_ID_MODE` in `board_config.h`); the console is the XIAO's native USB
> Serial/JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`); mag/baro drivers are compiled out
> unless `BLIMP_BOARD_HAS_MAG`/`_BARO` are set; the command packet is protocol v3 with the
> search hint and the any-balloon altitude fix. The ToF and touch-detector gaps below still
> stand and are the two most important ones.

- **No ToF/rangefinder driver.** `drivers/` contains exactly `imu`, `baro`, `mag`, `motor`,
  `servo` — the task that produced this tree scoped the driver list to those five files. Altitude
  fusion (`core/include/blimp/estimator.h`) is designed to tolerate a missing range source
  (`blimp_imu_sample_t.range_valid` staying false), but accuracy will be materially worse without
  one — add a `tof.h`/`tof.c` pair alongside these five when a VL53L1X (or similar) is chosen
  (docs/02-hardware.md section 6, question C.9).
- **No onboard contact/touch detector.** `main.c`'s `control_task` always passes
  `contact_detected = false` into `blimp_step()` — see its own `TODO` comment. Guidance's
  `APPROACH_TIMEOUT`/`MISS` path (guidance.h) is the only thing currently able to end an approach;
  a real IMU-deceleration-spike detector (docs/05-race-strategy.md section 4,
  docs/07-control-design.md section 6.2) still needs to be written, most likely as logic inside
  `control_task` reading the same `blimp_imu_sample_t` already being assembled each tick.
- **No waypoint running-order upload over the wire.** `net/protocol.h`'s command packet carries
  only a perception observation and an arm request — not Rule 2's per-run waypoint sequence. That
  needs either a third, one-shot packet type, or a different channel (e.g. a value baked in before
  flashing, unacceptable per Rule 2's "specified prior to each run" requirement) — not designed
  here.
- **No OTA, no param/log live-tuning service.** Both are named as intended infrastructure in
  docs/03-software-architecture.md sections 1.5 and 8, but neither is implemented in this
  skeleton — gains currently can only change by editing `core/src/params.c` and reflashing.
- **No battery-critical or attitude-fault failsafe trigger wired up in firmware** — see section 7
  above.
