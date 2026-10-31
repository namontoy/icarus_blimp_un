# Bench test tools

## For this repo's firmware: `blimp_link.py`

The S3 bench firmware (`firmware/projects/i2c_tools`) is a Wi-Fi access point; `blimp_link.py`
is its ground tool, standard library only, any OS. Join the blimp's SSID (`ICARUS-xxxxxx`),
then:

```
python software/blimp_link.py selftest                     # no hardware
python software/blimp_link.py status                       # telemetry
python software/blimp_link.py console                      # the firmware console, over the air
python software/blimp_link.py console "param alt."         # the controller's gains; "param alt.kp 0.4" sets one
python software/param_check.py --reboot                    # the 'param' bench checks, then a power cycle
python software/blimp_link.py motor 0.2 0 0 0 --seconds 2  # FL RL FR RR, PROPS OFF
python software/blimp_link.py stop
python software/link_check.py --download 60 --timers      # latency, loss, the blimp's wait, during a download too
```

`link_check.py` (2026-10-29) measures the link with the ground station closed: the time from a
`MOTOR_SET` with a new value to its echo in the telemetry, the loss as the blimp counts our
packets, the longest the blimp waited for one of ours (`link_age_ms`; it coasts at 500), the
telemetry and sensor rates and gaps, the same figures during a capture download, and the two
firmware timeouts (motors coast 0.5 s after the last `MOTOR_SET` even with heartbeats on, since
the link timeout counts motor packets only from 2026-10-29; the link falls 0.5 s after the last
packet).

`mock_blimp.py` serves the same protocol on localhost for development without hardware
(`--host 127.0.0.1` on the commands above). Protocol, safety timers and the bring-up
checklist: `docs/wireless-link.md`.

## The PID tuning plan's instruments

Three standard-library tools for `docs/pid-tuning-plan.md`; each has a `--selftest` or a dry run,
and all of them work against `mock_blimp.py` (`--host 127.0.0.1`):

```
python software/thrust_map.py run --motor FL --reverse --go       # P0: the thrust stand, one motor, PROPS ON
python software/thrust_map.py fit captures/thrust-*.csv           # F(u) per motor, Rule 13 sum, thrust-map.json
python software/step_test.py yaw --mirror                         # Y1/Y2: prints the schedule; --go runs it
python software/step_test.py heave --hold 3 --gap 8 --go          # H1/H2
python software/step_response.py captures/step-yaw-<date>.bin --axis yaw --all \
    --thrust-map captures/thrust-map.json --half-span 0.10        # the fits again, in core/ units
```

`step_test.py` refuses to start while another source (the ground station) holds the motors and
stops them on Ctrl-C and on any error; the captures and their `.json` notes go under `captures/`.

The closed-loop tests (C1–C4) run on the firmware's bench hold (`hold`, branch `control-inc3`):

```
python software/hold_test.py --steps 0 30 0 -30 0 --dwell 8      # prints the plan; --go runs it
python software/hold_test.py --steps 0 30 0 --go --max-duty 30 --deadband 10 \
    --param front_half_span_m=0.12 motor_max_thrust_n=0.0098 front_deadband_n=0.0005 \
            yaw.out_max=0.00118 yaw.out_min=-0.00118 yaw.kp=0.0006 yaw.kd=0.0006   # the rope rig
```

`hold_test.py` keeps the link alive that the hold needs, steps the heading, kills the hold from
the ground when the gyro passes `--max-rate`, and always ends with `hold off`; it saves the
capture with overshoot, settling and the final error per step in its `.json`.

## The ground station: `ground_station/`

The GUI for the bench and the arena: motors, sensors, the camera and the console in one
window (or a browser page), started with `python -m software.ground_station` (`--demo` needs
no hardware). Dependencies: `pip install -r software/ground_station/requirements.txt`. Its own
README covers installing on Windows and Linux, the differences between the two (the window
engine and how the Wi-Fi button joins the blimp), and how to use it; the design is
`docs/ground-station-plan.md`.

## The esp-drone tools (removed 2026-10-30)

`drone_gui.py`, `drone_dynamics.py`, `motor_diag.py` and their `SETUP.md` were written for the
esp-drone quadcopter the team used before the blimp's own firmware existed. They speak CRTP over
UDP to that firmware, not to anything in this repo, and the ground station replaced them, so they
left the tree on 2026-10-30. They are in git history (`git show dab8786:software/drone_gui.py`);
`docs/esp-drone-tools-report.md` and `docs/history/` describe them.
