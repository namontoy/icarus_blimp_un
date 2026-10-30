# Building and flashing the ESP32-S3-Zero (`i2c_tools` firmware)

How to build this project's flight-controller firmware and load it onto a
Waveshare ESP32-S3-Zero, on Linux or Windows. Written 2026-10-22 against branch
`s3-wireless-link`. A clean `set-target` + `build` was checked that day
on Linux: no warnings, and a 905 KB image in the 2 MB app partition (56 % free).

What the firmware does, its commands and its wiring are in this folder's
[`README.md`](README.md).

## What you need

- A **Waveshare ESP32-S3-Zero** and a USB-C **data** cable. Some cables only
  carry power, and with one of those no serial port appears.
- **ESP-IDF v5.5.5**. Other versions are untested. On v5.3.x the `led_strip`
  component does not compile (`docs/wireless-link.md` §10).
- Internet access for the first build. It downloads the `espressif/led_strip`
  component.
- The code: `git clone git@github.com:namontoy/icarus_blimp_un.git`, then
  `git checkout s3-wireless-link`.

**Safety:** if the board is wired into the blimp, take the propellers off. At
boot the firmware takes control of the motor driver pins first, but motor
commands are typed at the console, and a typo can spin a thruster.

## Why this board needs no BOOT button

The S3-Zero's USB-C goes straight to the chip's built-in USB port. There is no
USB-to-UART bridge chip in between. `esptool` resets the chip into download mode
over that USB link, so a plain `idf.py flash` works and you don't touch the
buttons. This is different from the ESP32-CAM, which needs BOOT held through an
external programmer. Don't apply the CAM procedure here.

That direct USB link has two side effects:

- The console runs over USB too (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`, set in
  `sdkconfig.defaults.esp32s3`). If that setting is lost, the board boots but
  the monitor stays silent.
- **Opening the serial port resets the chip.** Each time you start the monitor
  you see a fresh boot. This is normal.

## 1. Install ESP-IDF v5.5.5 (once per computer)

**Linux**

```bash
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.5.5
cd esp-idf-v5.5.5
./install.sh esp32s3
```

If you prefer Espressif's EIM installer, it installs under `~/.espressif`
instead. Either way works. Step 2 shows how to activate each one.

Your user must be in the `dialout` group to open the serial port
(`sudo usermod -aG dialout $USER`, then log out and back in).

**Windows**

The repo's `scripts\idf.bat` expects ESP-IDF at `%USERPROFILE%\esp\esp-idf-v5.5.5`
and Python 3.11 from python.org at its default per-user path:

```bat
mkdir %USERPROFILE%\esp
cd %USERPROFILE%\esp
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.5.5
cd esp-idf-v5.5.5
install.bat esp32s3
```

If your paths differ, set `IDF_PATH` and `PY311` before you run the script.

## 2. Build

From the root of the repo.

**Linux**

First, **activate ESP-IDF in this terminal.** This puts `idf.py` and the
compiler on your `PATH`. You must do it again in every new terminal. If you
skip it, you get `idf.py: command not found`. Run the one line that matches how
you installed ESP-IDF:

```bash
source $HOME/.espressif/tools/activate_idf_v5.5.5.sh   # installed with EIM (the lab machines)
. $HOME/esp/esp-idf-v5.5.5/export.sh                   # installed with git clone + install.sh (step 1)
```

It prints the environment it set up. Check it with `idf.py --version`, which
should print `ESP-IDF v5.5.5`. Then build:

```bash
idf.py -C firmware/projects/i2c_tools set-target esp32s3   # only once, on a fresh checkout
idf.py -C firmware/projects/i2c_tools build
```

**Windows**

```bat
scripts\idf.bat i2c_tools set-target esp32s3
scripts\idf.bat i2c_tools build
```

`set-target` creates `sdkconfig` from `sdkconfig.defaults` and
`sdkconfig.defaults.esp32s3`. Those set 4 MB flash, the USB console, the
status LED on GPIO21 and I2C on SDA=4/SCL=5. Run it once. Running it again
rebuilds `sdkconfig` and throws away any `menuconfig` changes. `sdkconfig`
and `build/` are not in git, so every clone makes its own.

A successful build ends with
`i2c_tools.bin binary size ... (56%) free` and `Project build complete`.

## 3. Flash and open the console

Plug the board in. Find its port:

- **Linux:** `/dev/ttyACM0`. `ls /dev/ttyACM*` before and after plugging in.
  `lsusb` shows `303a:1001 Espressif USB JTAG/serial debug unit`.
- **Windows:** a new `COMx` under *Ports (COM & LPT)* in Device Manager. Windows
  10 and 11 need no driver.

**Linux** (in a terminal where ESP-IDF is activated, step 2)

```bash
idf.py -C firmware/projects/i2c_tools -p /dev/ttyACM0 flash monitor
```

**Windows** (replace `COM5` with your port)

```bat
scripts\idf.bat i2c_tools -p COM5 flash monitor
```

**To exit the monitor, press Ctrl+X.** That closes the monitor and frees the
serial port. Ctrl+T followed by `x` also works. **Ctrl+C does not exit**: the
monitor sends it to the chip on purpose. If you type `exit` or `quit` at the
`i2c-tools>` prompt, the firmware prints these instructions. It can't close the
monitor itself, because the monitor runs on your computer, not on the chip.

The stock monitor exits with Ctrl+], but on Spanish and Latin American keyboards
`]` needs AltGr, so that key combination usually does nothing. The file
`esp-idf-monitor.cfg` in this folder changes the exit key to Ctrl+X.
`idf.py` starts the monitor from this folder, so the setting applies here and
nowhere else.

To open the console again later without reflashing, run the same command with
only `monitor`. If a monitor is ever truly stuck, close it from another
terminal with `pkill -f idf_monitor`. That also frees `/dev/ttyACM0`.

## 4. Check that it runs

- The RGB LED on the board blinks (period set by `CONFIG_EXAMPLE_BLINK_PERIOD`,
  3 s by default).
- The console prints the "Steps to Use i2c-tools" banner, then the prompt
  `i2c-tools>`. The name comes from the Espressif example this firmware grew
  out of.
- `help` lists every command. `imu`, `mag`, `baro` and `tof` read the sensors
  (they need the sensors wired: a GY-88 or the stand-alone MPU-6050,
  QMC5883L and BMP280/BME280 breakouts, plus the VL53L1X; `mag` and `baro` find
  either set on their own). `motor --status` shows the thrusters.
  `wifi` and `link` show the wireless link.
- The board also starts a Wi-Fi access point named `ICARUS-xxxxxx`, password
  `icarusblimp` (both set under "Wireless link" in `idf.py menuconfig`). On
  2026-10-21 the access point had not yet been tested on hardware.
  `docs/wireless-link.md` §8 is the checklist for that.

## When something goes wrong

| Symptom | Cause and fix |
|---|---|
| `Build directory ... configured for project '<other path>'` | The `build/` folder came from another copy of the repo, or the repo was moved. Run `idf.py -C firmware/projects/i2c_tools fullclean`, then build again. |
| No `/dev/ttyACM*` or COM port appears | Charge-only cable, or a bad USB connector. One of our S3-Zero units has a faulty connector: it keeps reconnecting (`dmesg` shows `error -71`). Try another cable, then another board. |
| `Permission denied: '/dev/ttyACM0'` (Linux) | Add your user to `dialout` (step 1). |
| Flashing can't connect, or the port disappears right after boot | Put the chip into download mode by hand: hold **BOOT**, press and release **RESET**, release BOOT, then flash again. Press RESET afterwards to run the firmware. This is the standard ESP32-S3 recovery. We have not needed it on this board. |
| Board boots (LED blinks) but the monitor shows nothing | `sdkconfig` lost `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`. Delete `sdkconfig`, run `set-target esp32s3` again, rebuild. |
| Windows: `idf.bat` exits silently or says ESP-IDF not found | See `docs/wireless-link.md` §10. The script works around `export.bat` failing under Git Bash and a Windows Store Python on PATH. Check `IDF_PATH` and `PY311`. |
| Sensors stop responding the moment a motor starts | The motors and sensors share one 5 V supply, so suspect a brownout before you suspect code. See the header comment in `main/cmd_motor.c`. |
