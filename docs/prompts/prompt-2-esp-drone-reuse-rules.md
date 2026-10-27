# Task: bring selected patterns from esp-drone into the blimp control loop

## Source and target
- esp-drone: https://github.com/espressif/esp-drone, Espressif's port of Bitcraze's crazyflie-firmware
  (tag 2021.01), GPL-3.0. Clone it read-only into `external/esp-drone` (add that path to
  `.gitignore`; it is never committed) and write the commit hash you read into `docs/reuse_log.md`.
- Target: the control loop built in the previous task (`firmware/projects/i2c_tools`). Nothing in
  this task changes drivers, the link transport, the stage boundaries, or `core/`.
- Licence: esp-drone is GPL-3.0. Code copied verbatim carries that licence into this firmware; a
  pattern re-expressed in our own code does not. Prefer re-expression. For every item say which it
  is; a verbatim copy keeps its original header. (`docs/03-software-architecture.md` section 2 and
  `docs/04-risk-and-troubleshooting.md` risk 18 discuss this.)

## Reuse rules
1. Reuse plant-independent patterns:
   - Fixed-rate task with rate dividers for slower sensors (esp-drone's `RATE_DO_EXECUTE` idea).
     Trivial: re-implement.
   - PID structure from `pid.c` (`pidUpdate`): P on the error, optional second-order low-pass on the
     D term, symmetric integral clamp (`iLimit`, 0 = off), output clamp (`outputLimit`, 0 = off).
     Note that esp-drone differentiates the ERROR (`error - prevError`), not the measurement. Keep the
     structure but compute the derivative on the measurement, so a setpoint step does not kick the
     output, and keep the D filter. If the loop ends up calling `blimp_step()`, the PID is
     `core/`'s `blimp_pid_t` and this item is dropped.
   - Commander with setpoint timeout -> failsafe: our link timeout and motor failsafe already exist;
     take only the priority and latch semantics, not the code.
   - Log/param idea: variables registered by name, readable and tunable at runtime. esp-drone does it
     with linker-section macros and serves it over CRTP TOC packets. Re-express it as one flat table
     {name, type, pointer, min, max} and two new packet types on our link (PARAM_GET/SET and a log
     block request); no macros, no TOC machinery.
2. Do NOT import:
   - the roll/pitch rate -> attitude cascade (`controller_pid.c`, `attitude_pid_controller.c`)
   - the quadrotor mixer (`power_distribution_stock.c`)
   - Kalman process models that assume thrust along body z (`kalman_core.c`, `estimator_kalman.c`)
   - the position / flow-deck controller
   - `sitaw.c` free-fall and tumble detection, the arming and self-test logic, and the 8-bit
     `motorsConv16ToBits` PWM path
   - any sensor or bus driver: esp-drone targets ESP-IDF v4.4/v5.0 and the legacy I2C driver; our
     drivers use `i2c_master` on v5.5.5
   - the 500 Hz-1 kHz loop rates
   Reason: the blimp is passively stable in roll and pitch, actively controls only heave, yaw and
   surge, has buoyancy and large added mass, and runs at 50 Hz.
3. Never copy gains, filter cut-offs or estimator noise constants from esp-drone. Where the structure
   matches, start from the values in `core/src/params.c` (tuned in simulation with the measured
   thrust) and say so; otherwise initialise to zero or to conservative placeholders marked
   `TODO-IDENTIFY`. They will be confirmed by our own tests.
4. Frames and signs. The convention is fixed, do not pick another: body x forward, y left, z up, yaw
   positive counter-clockwise seen from above, SI units. This is Crazyflie's frame and `core/`'s
   (`core/include/blimp/types.h`). The mounting rotation of each breakout on the gondola is a
   parameter, not a hard-coded sign flip. Before any gain is set, run the sign tests: nose up ->
   pitch positive; left side up -> roll positive; turn the gondola counter-clockwise -> yaw rate
   positive; lift it -> altitude increases. A frame mismatch is the most common porting bug.
5. Keep our existing working drivers and communication link. Replace one with an esp-drone
   equivalent only if it is clearly better; justify the change and wait for approval. Do not adopt
   esp-drone's CRTP/UDP stack.
6. Minimal footprint: bring only what the current increment needs. No dead code, unused parameters
   or unused tasks.
7. Traceability: every reused or adapted module starts with a short comment giving the origin file in
   esp-drone (path and commit), the quadrotor assumption it contained, and how it was removed or
   changed. Keep the same information in `docs/reuse_log.md`, with the verbatim/re-expressed status
   from the licence note above.

## Working method
Same as the previous task: read, summarise, propose, wait for approval; one buildable increment at a
time; explain each decision briefly.
