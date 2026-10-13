# Wired link: two ESP32-CAMs → ESP32-S3 flight controller

> **Pin source of truth:** `docs/artifacts/Cableado del blimp (Emma).md` (the
> "Cableado", 9 Oct 2026). Every S3 and CAM pin it assigns is used here
> unchanged. Pins it does not assign — the downward camera, the PING))) backup
> altimeter and the MPU-6050 interrupt — are **proposed** in this plan and
> marked as such. Reconciled against the Cableado on 2026-10-13.

## Context

The blimp is a two-chip system that currently has no link between the chips.
The ESP32-S3 (`firmware/projects/i2c_tools/`) owns the IMU, the altimeter and the
two DRV8833s driving the four thrusters. The ESP32-CAM (`firmware/projects/optflow_cam/`) computes
downward optical flow and prints it as ASCII CSV to a **laptop**. The two have
never been wired together, and `docs/sensor-fusion-plan.md` does not mention a
camera at all — it recommends a PMW3901 chip instead and leaves the horizontal
filter as a stub. So this is new architecture, not an extension.

Two gaps in the camera firmware are blocked on the link existing, and both are
documented in the code itself:

- **No gyro de-rotation.** `optflow_main.c:23-31`: "whichever board owns the gyro
  can de-rotate properly." Pure yaw is currently indistinguishable from translation.
- **Height is an operator-typed constant**, set by the `h<mm>` console command.
  The S3 has the altimeter; the CAM is guessing.

Both need the S3 to push data *down*, so the link must be bidirectional.

The target is two cameras: one forward running a balloon detector **on the CAM
itself** (so only detections cross the link), and one downward for flow, plus
**on-demand single-frame grabs** for the §11 tuning work.

**How this relates to the Cableado.** The Cableado wires **one** ESP32-CAM, the
forward balloon camera, on S3 GP2/GP6. It recommends against a second camera
*for stereo*, and suggests a PMW3901 for ground speed. It does not consider a
downward ESP32-CAM for optical flow, which is what `optflow_cam` is. So CAM-FWD
uses the Cableado's pins exactly, and CAM-DOWN gets proposed pins on pads the
Cableado leaves free. CAM-DOWN is optional hardware: everything for CAM-FWD
works without it.

### The bandwidth question, answered

The premise that UART needs replacing for throughput does not survive the numbers.

| Traffic | Size | Rate | Load |
|---|---|---|---|
| Flow telemetry, binary | 31 B | 12.5 Hz | **388 B/s** |
| Height + gyro, downward | 27 B | 25 Hz | **675 B/s** |
| Balloon detections | ≤ 70 B | 12.5 Hz | ≤ 0.9 kB/s |
| **Steady state, per link** | | | **≈ 1.1 kB/s** |
| One QQVGA frame grab | 19.2 kB | on demand | ~210 ms burst at 921600 |

Steady state is **1.2% of a 921600-baud link** (0.5% of 2 Mbaud) and would fit
inside 115200 four times over. The current ASCII CSV is only ~110–145 B/frame ≈
1.5 kB/s — 13% of 115200. `SOC_UART_BITRATE_MAX` is **5,000,000** on both chips
(`soc/esp32/include/soc/soc_caps.h:359`, `esp32s3/…:425`). 921600 is a fractional
divider of the 80 MHz APB clock (86 + 13/16, 0.01% error), well inside UART
tolerance; 2 Mbaud is an exact divider of 40.

**So the reason to change the link is not speed.** It is: binary framing with a CRC
instead of parsed text, a downward channel that does not exist today, getting off
the CAM's programming port, and the frame-grab burst.

---

## Decision: two independent point-to-point UARTs, 921600 baud by default

The Cableado specifies 115200–921600 baud for the camera link. **921600 is the
default.** 2 Mbaud stays available as a runtime switch (`SET_BAUD`, auto-reverting
after 2 s of silence), to be enabled only after the scope check in
[Verification](#verification) passes with the Cableado's 470 Ω series resistors
in place. (470 Ω into ~50 pF of pin plus wire is τ ≈ 24 ns, about 2% of a
921600-baud bit and 5% of a 2 Mbaud bit, so neither should be marginal. That is
a calculation, not a measurement.)

Not SPI. I initially designed a shared SPI bus and it is the wrong choice here —
the deciding constraint is not bandwidth but **which pins are free on the CAM**.

With the camera and its flash LED attached, exactly five GPIOs are free on an
AI-Thinker ESP32-CAM: **2, 12, 13, 14, 15** (the microSD pads). Of those, four are
ESP32 strapping pins — 2 and 12 and 15, plus 0 and 5 already taken by the camera.
**Only GPIO13 (MTCK) and GPIO14 (MTMS) are not straps.** A two-wire UART fits
entirely inside the non-strapping set, and that is also the pair the Cableado
uses. SPI needs four pins and therefore *must*
consume straps on the one board in this system that is painful to reflash
(`optflow_main.c:406` `escape_hatch_delay()` exists precisely because GPIO0 is XCLK
and esptool can never force download mode once the camera starts).

What SPI would buy: a 19.2 kB grab in ~22 ms instead of ~210 ms, on an operation
that happens when a human asks for it. What it would cost:

- 2 extra CAM pins, both strapping. The Cableado also requires IO12 to stay free:
  if it is high at power-up, the camera does not boot.
- A shared MISO net. GPIO12 is the flash-voltage strap; one CAM driving the shared
  line while the other releases reset mis-straps the flash regulator. Moving MISO to
  GPIO2 trades that for a worse problem — GPIO2's strap fires *only* during
  download-mode entry, so the other CAM driving MISO high breaks the flash you were
  trying to do.
- Two ESP32-only silicon workarounds in the data path: the slave CS freeze
  (`spi_slave.c:231-233`) and the DMA-abort reset
  (`spi_slave_hal_iram.c:80-93`) — the latter fires whenever the master clocks a
  different length than the slave armed, which breaks the obvious
  "status word says if a payload follows" protocol.
- A shared-net MISO tri-state assumption that is not stated anywhere in the IDF tree.

Also rejected: **I2C with the CAMs as slaves.** The ESP32 I2C controller
**cannot clock-stretch as a slave** (`docs/en/api-reference/peripherals/i2c.rst:52`),
and the CAM is busy in 50 ms blocks — stretching is I2C's only backpressure. Worse,
the tempting version hangs them off the existing GPIO4/5 bus, where a wedged CAM
holding SDA low would take out the MPU-6050 and the VL53L1X altimeter that the
estimator depends on.

**Keep SPI as a documented fallback.** If continuous video is ever needed rather
than on-demand stills, it is the only answer. Leave CAM GPIO 2 and 12 unused.

---

## Pin assignments

### ESP32-CAM — identical wiring and identical binary on both boards

The UART pins follow the Cableado: **the camera receives on IO13 and transmits on
IO14.** (An earlier version of this plan had the directions swapped.)

| GPIO | Function | Source | Note |
|---|---|---|---|
| **13** | **UART1 RX ← S3 TX** (through 470 Ω) | Cableado | MTCK, not a strap. 10k pull-up to the CAM's own 3V3 |
| **14** | **UART1 TX → S3 RX** (through 470 Ω) | Cableado | MTMS, not a strap. May emit pulses or stray bytes at boot, before the UART is configured; the S3 must discard anything that is not a valid frame |
| **15** | **Role strap**: open = downward, wire to GND = forward | proposed | Input, internal pull-up |
| 1, 3 | UART0 console **and programming port** (U0T, U0R) | — | unchanged; the boot log goes out here, not on IO14 |
| 0 | XCLK | — | strap; guarded by `escape_hatch_delay()` |
| 4 | 1 W flash LED, held low | — | `quiesce_heat_sources()` |
| 2, 12 | **unused** — IO12 must stay free (Cableado); both reserved for the SPI fallback | Cableado | leave open |
| 5, 18, 19, 21, 22, 23, 25, 26, 27, 32, 34, 35, 36, 39 | camera | — | `camera_pins.h` |
| 16 | PSRAM CS — reclaimable, see "Side finding" | — | |

CAM-side rules from the Cableado:

- **The microSD slot stays empty.** The card uses IO13 and IO14.
- **Pass pins 13 and 14 explicitly** to `uart_set_pin()`. The ESP32's default UART
  pins clash with the ESP32-CAM's PSRAM and flash.
- **Ground from the GND pin next to 5V**, never the last pad of the 3V3 row. On
  these clones that pad is labelled GND/R and may be wired to reset.

GPIO15 is the right pin for the role strap because it is the only leftover that is
safe to drive either way: its strap function (MTDO) merely suppresses the ROM boot
log on U0TXD, which costs nothing since `boot_report()` prints the reset reason from
software. Open = downward. One binary, one flash procedure, and the role survives
`erase_flash` because it is a wire. Back it with an NVS override for bench work.

### ESP32-S3-Zero — complete map

The Cableado assigns almost every edge pad, so this table lists the whole board,
not just the additions. The pinout comes from the Cableado's Figure 2 (Waveshare
wiki): edge pads, small top-side pads 14/15/16, and one column of 8 back pads
(45, 42, 41, 40, 39, 38, 18, 17).

| GPIO | Function | Source |
|---|---|---|
| 1 | ADC: motor battery, after S1, through a 100k + 100k divider, 100 nF to GND | Cableado |
| **2** | **UART1 TX → 470 Ω → CAM-FWD IO13** | Cableado |
| 3 | Free. Optional ADC for the logic battery, with its own 100k + 100k divider after S2. A strap pin, but ignored with factory eFuses | Cableado (optional) |
| 4 / 5 | I²C SDA / SCL: GY-88 and VL53L1X | Cableado |
| **6** | **UART1 RX ← 470 Ω ← CAM-FWD IO14** | Cableado |
| 13, 12 / 11, 10 | DRV8833 #1 IN1, IN2 (FL) / IN3, IN4 (RL) | Cableado |
| 9, 8 / 7, 14 | DRV8833 #2 IN1, IN2 (FR) / IN3, IN4 (RR). 14 is a small top pad | Cableado |
| 15 | VL53L1X INT (the sensor's GPIO1 pad), optional. Small top pad | Cableado |
| 16 | VL53L1X XSHUT, open-drain. Small top pad | Cableado |
| **17** | **UART2 TX → 470 Ω → CAM-DOWN IO13** | **proposed** (back pad) |
| **18** | **UART2 RX ← 470 Ω ← CAM-DOWN IO14** | **proposed** (back pad) |
| **38** | **PING))) 28015 SIG** (backup altimeter), through the 1.0k / 1.8k divider in `cmd_ping.c` | **proposed** (back pad) |
| **39** | **MPU-6050 INT** (GY-88 `G_INT`), optional; `sensor-fusion-plan.md` §5.1 asks for it | **proposed** (back pad) |
| 40, 41, 42 | spare | back pads |
| 19 / 20 | native USB | — |
| 21 | WS2812 RGB LED | — |
| 43 / 44 | TX / RX pads: **left free** (Cableado). TX prints at every boot and both start with pull-ups | Cableado |
| 0 | BOOT button | — |
| 45 | strap (VDD_SPI voltage): do not use | — |

Why these proposals:

- **GP17/18 for CAM-DOWN.** They are not straps, they are UART1's IO-MUX pins on
  the S3 (`soc/esp32s3/include/soc/uart_pins.h`), and wiring the downward camera
  there gives it the same IO13/IO14 wiring and the same 470 Ω resistors as CAM-FWD.
  UART2 routes to them through the GPIO matrix, which is fine at these baud rates.
- **GP38 for the PING))).** `cmd_ping.c` defaulted to GPIO6, which the Cableado
  now gives to the camera UART, so its default must change to 38. It runs off the
  5 V step-up rail (~30–35 mA) and needs the divider because the S3 is not 5 V
  tolerant.
- **GP39 for the MPU-6050 INT.** It is the next free back pad. The GY-88's `G_INT`
  is a 3.3 V output; the Cableado leaves it unconnected, which is still fine for
  polling.
- **HMC5883L `M_DRDY`** stays unconnected, as in the Cableado: `cmd_mag.c` reads
  DRDY over I²C.

> **Overlap with the bench sketches.** The Arduino sketches in
> `docs/artifacts/Height and Ground-Speed Sensors (Emma).md` put bench-only
> ultrasonic sensors on 17/18 (HC-SR04) and 38/39 (US-100). That is fine on a
> separate bench board. On the flight board those pads are now the CAM-DOWN UART,
> the PING))) and the MPU INT, so change the sketches' `#define`s before running
> them there.

> **Old S3-Zero revision** (buttons labelled BOOT/RESET, two columns of back pads,
> LED on GP10): per the Cableado, DRV8833 #1 IN4 moves from GP10 to **GP15** and the
> ToF INT stays unconnected. Nothing in this plan's proposals changes.

> **If the Cableado's "recover thrust" variant is adopted** (DRV8833 in parallel
> for each forward motor, a MOSFET for each vertical one): GP10 and GP14 become
> free, and GP11/GP7 drive the MOSFETs. The link pins are unaffected.

The S3's console is USB Serial/JTAG (`CONFIG_ESP_CONSOLE_UART_NUM=-1`), so all
three UART controllers are free: UART1 for CAM-FWD, as the Cableado says, and
UART2 for CAM-DOWN.

> **Always call `uart_set_pin()` explicitly**, for both ports. UART2 has no IO-MUX
> pins at all, and nothing routes UART1 to GP2/GP6 unless you do. (An earlier
> version of this plan warned that UART1 defaults to GPIO 9/10. That is true of the
> original ESP32, not the S3, where UART1's IO-MUX pins are 17/18.)

### Mandatory passives

- **470 Ω in series in each of the four signal lines**, as in the Cableado
  (S3 TX → CAM IO13 and CAM IO14 → S3 RX, per camera). They limit the current if
  both ends drive the line against each other while booting.
- **10k pull-up on each receiving pin, to that board's own 3V3.** This is a plan
  addition the Cableado does not mention. While a board is in reset its TX pad is
  an input and the line floats; a floating RX reads as a permanent start bit and the
  receiving UART raises break/framing interrupts *at the baud rate*. On the flight
  controller that is an interrupt storm. Pulling up to the *receiver's* rail, not
  the sender's, keeps the resistor from back-feeding an unpowered board.
  `uart_set_pin()` pulls up CTS only, never RX — so also call `gpio_pullup_en()` on
  each RX pin right after it.
- **C2, 470 µF (≥ 10 V), plus 100 nF, at each CAM's 5V pin**, as the Cableado
  specifies for its one camera. `esp_camera_init()` is the largest current step
  either CAM ever takes; stagger the two by a few hundred ms using the role strap.
- **One common ground point**, as in the Cableado, with a **dedicated return
  conductor per CAM** run from its 5V-side GND pin to that point, alongside its
  signal pair — not a daisy chain. At these baud rates, ground offset from
  camera-readout transients shows up as framing errors that read like software
  bugs. Twist each camera's 5 V pair (Cableado).

### Power and placement cost of the second camera

The Cableado's power budget counts **one** camera. CAM-DOWN adds roughly
100–180 mA typical and 300–500 mA peaks at 5 V to the logic step-up, so peak
demand (S3 + sensors + two cameras) could reach ≈ 1.0–1.4 A at 5 V. The Cableado
sizes the step-up for **≥ 0.8 A out at 3.0 V in**; the Pololu S9V11F3S5 gives
≈ 0.9 A there. These are datasheet estimates, not measurements. Before fitting
CAM-DOWN:

- measure the logic current with CAM-FWD alone, then re-check the step-up and F2
  (≈ 3 A) with the second camera added;
- stagger the two `esp_camera_init()` calls, as above;
- count 7–10 g more mass (≈ 1.3 L of helium per gram in Bogotá, per the Cableado);
- keep CAM-DOWN and its 5 V pair away from the magnetometer. The Cableado already
  flags CAM-FWD at ≈ 1.3 cm from the GY-88 as a compass disturbance.

---

## Protocol

Binary, length-prefixed, CRC-16/CCITT-FALSE, little-endian.

```
+------+------+------+--------+--------+---------+--------+--------+
| 0xA5 | 0x5A | type | len_lo | len_hi | payload | crc_lo | crc_hi |
+------+------+------+--------+--------+---------+--------+--------+
```
CRC covers `type` through `payload`. 7 bytes of overhead. The two-byte header is
also what lets the S3 ignore the stray bytes the Cableado warns IO14 may emit
while a camera boots.

> **Serialise field by field. Never `memcpy` `flow_result_t` onto the wire.** Its
> real layout (`flow_match.h:62-74`) is `dx, dy, n_valid, n_total, n_flat, n_edge,
> n_ambig, spread` — 32 bytes, with **`spread` last**, not third. Getting this wrong
> silently swaps the confidence number with a block count, and the header comment is
> explicit that `spread` "is the honest confidence number - use it, not n_valid."

**S3 → CAM**

- `0x01 STATE` — 20 B at 25 Hz: `u32 t_us`, `i32 height_mm`, `f32 gyro_x/y/z` (rad/s,
  body frame). This single packet closes both documented gaps.
- `0x02 CMD` — `u8 cmd; i32 arg`. Mirrors the existing console verbs (`h f r e z s ?`)
  so nothing is lost, plus `GRAB_FRAME`, `SET_BAUD` (auto-reverting on 2 s silence),
  `SET_RATE`, `SET_CHUNK`, and `REBOOT_FOR_FLASH`.

**CAM → S3**

- `0x81 FLOW` — 24 B at 12.5 Hz: `u32 t_us`, `u16 dt`, `i16 dx_q8`, `i16 dy_q8`
  (**raw pixel flow, kept alongside the derived velocity** per the contract at
  `optflow_main.c:23-31`), `i16 vx_mm_s`, `i16 vy_mm_s`, `u16 spread_q8`, the five
  block counters as `u8`, `u8 flags`, `u16 boot_count`.
- `0x82 DETECTIONS` — forward CAM. Carries the fields the Cableado lists for the
  balloon packet: `u16 frame`, `u32 t_us` (frame timestamp), `u8 n`, then n × 7 B:
  `i16 az_cdeg` and `i16 el_cdeg` (horizontal and vertical angle to the balloon,
  hundredths of a degree), `u16 diam_px_q4` (apparent diameter, px × 16, which gives
  distance as Z = K / w after calibration), `u8 confidence`. The header and checksum
  are the frame's own. n ≤ 8 keeps it ≤ 70 B on the wire.
- `0x83 FRAME_META` / `0x84 FRAME_CHUNK` — **240-byte chunks, 80 per frame, by
  default** (`SET_CHUNK` changes it). At 921600 baud each chunk is ~2.7 ms, so a
  `FLOW` packet can be interleaved between any two and telemetry is never delayed
  more than ~3 ms during a grab; at 2 Mbaud, 480-byte chunks give the same delay.
  Source the pixels from **`s_prev`** (`optflow_main.c:~432`), the firmware's own
  private 19,200-byte internal-DRAM copy — not from a live `camera_fb_t`, which
  would starve the `fb_count=2` camera driver.
- `0x85 LOG` — ASCII passthrough so CAM diagnostics surface on the S3's USB console.
- `0x87 BOOT` — `u32 boot_count; u8 reset_reason; u32 build_id; u8 role`, sent once
  at startup. **The S3 must treat a CAM going silent as expected** and re-push height
  and config when `boot_count` changes; otherwise a CAM that resets mid-flight
  silently reverts to its compiled-in default height and the estimator eats a bad
  scale factor.

---

## Implementation phases, ordered to minimise CAM reflashes

Every CAM-side change costs a reflash, so the CAM code must be right *before* it is
flashed, and anything that might need tuning later must be a runtime knob.

**S3 rule for every phase:** do not configure a link UART while the cameras are
unpowered. The Cableado's bring-up says not to start UART1 on a USB-only bench,
because an idle-high TX line powers the camera through its data pin. The S3 link
task starts on an explicit console command during bring-up, and the pins stay
inputs until then. (Once GP3's optional logic-battery divider exists, it can gate
this automatically.)

**Phase 0 — zero CAM reflashes. Prove the physical layer.**
Wire S3 UART2 (GP17 TX, GP18 RX, through the 470 Ω resistors) to CAM-DOWN's
existing UART0 console (U0R, U0T). Parse the ASCII CSV the firmware
already emits at 115200; push height with the `h<mm>` command it already accepts
(`optflow_main.c:258-290`). This closes the loop with **no CAM firmware change at
all** and validates grounding, levels, connectors and the S3-side plumbing. Temporary
only — UART0 is the programming port and the permanent link must not sit on it.

**Phase 1 — zero CAM reflashes. Build and debug the whole protocol off-target.**
1. Write the codec as one target-agnostic pair, `link_proto.c/.h`, with no IDF
   dependencies — framing, CRC, pack/unpack only.
2. Compile and fuzz it on the **host**; `optflow_cam/host/` already exists for this.
3. Compile it for the **S3** and run **both ends on the S3**, with both cameras
   disconnected: UART1 TX (GP2) to UART2 RX (GP18), one jumper. The S3 reflashes
   over USB in seconds. Exercise framing, CRC rejection,
   chunked frame transfer with interleaved telemetry, baud change, resync after
   injected garbage, and `BOOT`-driven re-priming, all without touching a CAM.
4. Build the S3 link task, its 50 Hz consumer, and the `LOG` relay.

**Phase 2 — one CAM reflash each. Must contain everything at once.**
Same binary to both boards: `link_proto` on UART1, RX = GPIO13 and TX = GPIO14,
both passed explicitly, with `gpio_pullup_en()`
on RX; role from the GPIO15 strap with NVS override; `STATE` consumption (height +
gyro de-rotation, raw flow still reported alongside); the full `CMD` set so baud,
rates, chunk size, search radius and height are **runtime** knobs and never a
reflash; `LOG` and `BOOT`; `FRAME_META`/`FRAME_CHUNK` from `s_prev`; NVS persistence
(on this board the power cycle *is* the reset — report line 339).

Include `REBOOT_FOR_FLASH` — `esp_restart()` with the escape window stretched. After
this lands, reflashing a CAM becomes *"send the command, then `idf.py flash`"*: no
BOOT button, no replug, no 5-second race, and no disturbance to the other camera.
**That is the real payoff of building this link**, and it is worth more than the
~190 ms of grab time SPI would have saved. Flashing still goes through the CAM's
U0T/U0R with an external adapter, following the Cableado's rules: never join the
adapter's 5 V to the step-up's, and keep S2 open or the CAM's 5V pin disconnected
from the step-up while flashing in place.

Bring up CAM-DOWN alone. Wire CAM-FWD only once CAM-DOWN is clean. (CAM-DOWN goes
first because its firmware exists; CAM-FWD's pins are the Cableado's and do not
depend on this order.)

**Phase 3 — zero CAM reflashes. S3-side integration.**
Height from the altimeter (VL53L1X, with the PING))) on GP38 as backup); gyro
decimated from the estimator's 200 Hz to 25 Hz for
`STATE`; flow into the horizontal filter that `sensor-fusion-plan.md:335-338` leaves
as a stub; link-loss and `BOOT` re-priming; a `framegrab` console command that dumps
a PGM over USB. All iteration happens on the USB-flashable board.

**Phase 4 — one reflash, forward CAM only.** The balloon detector. `0x82 DETECTIONS`
already exists from Phase 2, so the link does not change — only the producer.

`sensor-fusion-plan.md` needs amending either way: it currently recommends a PMW3901
and has no concept of a second MCU or any transport.

---

## Side finding, independent of the link

`CONFIG_SPIRAM=y` is set on the CAM, and `sdkconfig.defaults:24-27` says verbatim it
is "enabled for compatibility, not because we rely on it" — all pixel buffers are
forced to internal DRAM. But `CONFIG_SPIRAM_CACHE_WORKAROUND=y` applies
`-mfix-esp32-psram-cache-issue` **project-wide** via
`esp_psram/project_include.cmake:6`, inserting `memw` after vulnerable
loads/stores across the whole image — including `flow_match()`, which costs 49–55 ms
of every 80 ms frame. Setting `CONFIG_SPIRAM=n` plausibly buys free matcher headroom
and reclaims GPIO16.

Treat it as a **measurement, not a recommendation**: it changes a working, carefully
tuned firmware, so record `proc_us` before and after and keep the change only if the
delta is real. It also removes a live trap — the ESP32's GP-SPI DMA cannot reach
external RAM, so a plain `malloc(19200)` for a frame buffer would land in PSRAM
(`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384`) and be rejected by any DMA path.

---

## Verification

| Stage | Check |
|---|---|
| Phase 0 | S3 console shows live flow rows parsed from CAM-DOWN over GP17/18; `h<mm>` round-trips |
| Phase 1, host | Fuzz the codec: truncated frames, bit flips, garbage prefixes — every CRC failure resyncs within one frame, no crash |
| Phase 1, S3 loopback | GP2 → GP18 jumper, cameras disconnected. Sustained 25 Hz `STATE` + 12.5 Hz `FLOW` for 1 h with zero CRC errors; an 80-chunk grab completes with telemetry never delayed > 5 ms |
| Phase 2 | Both CAMs report correct `role` in `BOOT`; raw `dx/dy` matches the UART0 CSV for the same frame (proves serialisation, catches the `spread` field-order trap); rotate the airframe on the bench with the props off and confirm `vx/vy` collapses toward zero while raw `dx/dy` does not |
| Phase 2 | `REBOOT_FOR_FLASH` then `idf.py flash` succeeds with no button and no replug, and the other CAM keeps streaming throughout |
| Phase 3 | `framegrab` writes a PGM that opens and shows the real scene |
| Power | Scope all four signal nets through a CAM power cycle: pull-ups hold them high, no break storm at the far end, and any boot bytes from IO14 are discarded without a CRC-error burst |
| Baud | Scope the edges at the receiving pins with the 470 Ω fitted, at 921600 and then 2 Mbaud. Only enable 2 Mbaud if the edges are clean |
| Current | Logic-cell current with CAM-FWD alone, then with both cameras, during both `esp_camera_init()` calls |

## Before cutting any wire

1. `espefuse.py summary` on both CAMs (`XPD_SDIO_*`) and `esptool.py flash_id` —
   settles whether the module is 1.8 V or 3.3 V flash, which decides GPIO12's strap
   polarity and whether it could ever be reclaimed. **Unverified either way right now.**
2. Meter GPIO13/14/15 to 3V3 and to GND on an unpowered CAM, looking for factory
   pulls on the SD pads. The AI-Thinker is not in the IDF's list of boards with known
   SD pull-ups.
3. **Identify the S3-Zero revision** as the Cableado describes: "B" and "R" by the
   buttons and a single column of 8 back pads (45, 42, 41, 40, 39, 38, 18, 17) is the
   current one, which this map assumes. Confirm the back pads 17, 18, 38 and 39 are
   present on your unit before planning wires to them; they are the only pins this
   plan adds.
4. **Power during development**, per the Cableado: the logic cell feeds the step-up,
   which feeds each CAM's 5V pin and, through D1, the S3's 5V pin. Do not connect the
   S3's USB with S2 closed. On a USB-only bench, leave the link UARTs unstarted (see
   the S3 rule above), because the cameras are unpowered.
5. Set `cmd_ping.c`'s default pin to GP38 before restoring it to the build: GPIO6 is
   now the camera UART.

### Claims I could not verify

- GPIO12 strap polarity on this specific module (item 1 above).
- Whether `esp_restart()` releases GPIO0 from LEDC, which `REBOOT_FOR_FLASH` depends
  on. Very likely — a digital system reset resets LEDC and the GPIO matrix — but it
  must be confirmed on hardware once, in Phase 2.
- The S3-Zero back-pad set: taken from the Cableado's Figure 2, which cites the
  Waveshare wiki; not checked against the physical board (item 3).
- The step-up's headroom for a second camera (see "Power and placement cost").
