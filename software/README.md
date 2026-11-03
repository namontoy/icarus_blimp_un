# Bench test tools

## For this repo's firmware: `blimp_link.py`

The S3 bench firmware (`firmware/projects/i2c_tools`) is a Wi-Fi access point; `blimp_link.py`
is its ground tool, standard library only, any OS. Join the blimp's SSID (`ICARUS-xxxxxx`),
then:

```
python software/blimp_link.py selftest                     # no hardware
python software/blimp_link.py status                       # telemetry
python software/blimp_link.py camera --seconds 10          # the forward camera's rows, relayed by the S3 (2026-11-01)
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

`blimp_link.py camera` watches the `CAMERA` packets (`LINK_PKT_CAMERA`, 0x85): at most `--rate`
lines a second (default 3) with the verdict, azimuth, elevation, range, `frame_seq`, rows/s, the S3's
rejected count, the UART and streaming state and the bearing `core/` would steer on (`bearing_sign`
x az, + left); `--seconds 0` runs until Ctrl-C. It says so if no `CAMERA` packet arrives in 3 s
(a firmware from before the relay, or no peer). `mock_blimp.py --camera rows|silent|off` fakes the
camera: a drifting red balloon at 12.5 rows a second (default), a camera that sends nothing
(`silent`: health-only packets), or a firmware without the relay (`off`). Built 2026-11-01; run
against the mock only, not yet against the blimp.

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

The closed-loop tests (C1–C4) run on the firmware's bench hold (`hold`, branch `pid-tuning`):

```
python software/hold_test.py --steps 0 30 0 -30 0 --dwell 8      # prints the plan; --go runs it
python software/hold_test.py --steps 0 30 0 --go --max-duty 30 --deadband 10 --surge 0.5 \
    --param front_half_span_m=0.12 motor_max_thrust_n=0.0098 front_deadband_n=0.0001 \
            yaw.out_max=0.00118 yaw.out_min=-0.00118 yaw.kp=0.0006 yaw.kd=0.0006   # the rope rig, B3
```

`hold_test.py` keeps the link alive that the hold needs, steps the heading, kills the hold from
the ground when the yaw rate passes `--max-rate` (roll and pitch have their own
`--max-tilt-rate` since 2026-11-01: the gondola's harmless nodding tripped a single limit), and
always ends with `hold off`; it saves the capture with overshoot, settling and the final error
per step in its `.json`. `--surge` holds a constant forward push for the whole run, so both
front motors keep turning and a yaw correction speeds one up and slows the other: on the rope
rig that removed the limit cycle that one motor at a time produced
(`docs/history/rig-2026-10-30.md`). A run that holds a height ends with a landing that `core/`
flies itself (`hold altitude <m> <m/s>`, core 0.4.0): 0.1 m/s down to 0.30 m, 3 cm/s on toward
the floor, the motors off at a ToF of 0.09 m, about 2 cm above where the gondola stands.

The square and its calibration runs (2026-11-01) fly with `square_test.py`: a climb, then legs
of a timed push, coast and brake (the brake a negative `hold surge`, which needs
`param front_reverse_ratio` above 0) with turns on the spot between them, then the landing. The
legs are open loop, since nothing on the blimp measures where it is; tape on the floor and a
video measure them, laid out in the "Icarus Flight Room" page. `--legs 0` is the landing test,
`--legs 1` the push, brake and leg runs:

```
python software/square_test.py --legs 1 --push 2 4 --coast 8                     # prints the plan; --go flies it
python software/square_test.py --legs 4 --turn 90 --push 2 4 --coast 2 --brake 2 3 --go \
    --param front_reverse_ratio=0.74 ...                                         # the square
```

The balloon follow (F0–F2 in the tuning plan) uses the same tool, after a check with the motors
off:

```
python software/cam_watch.py                                       # the camera through the S3, live
python software/hold_test.py --follow --dwell 60 --go --surge 0.5 ...   # turn toward the balloon
```

`cam_watch.py` polls the firmware's `cam` (console only, so the ground station may stay open) and
prints the row rate and, for a balloon in view, the bearing `core/` would steer on and on which
side of the blimp it is: the sign check before anything turns. `--follow` replaces the heading
steps with `hold follow` for `--dwell` seconds and reports the camera's row rate, how often the
balloon was in view and the yaw error while it was.

The balloon flights build on it: `target_test.py` (2026-11-02) climbs, finds one balloon and
moves toward and away from it; `course_test.py` (2026-11-03) flies a course of balloons in
order, the race in miniature: for each, it looks where the course puts it, follows it,
approaches in straight pulses, touches it, moves away and turns to the next, then lands.
`--close-px`, the patch at which it goes in to touch, comes from a motors-off camera check with
the course's own balloons (`docs/flight-tests.md`, "The course room"):

```
python software/course_test.py --selftest                                  # the sequence against a stand-in
python software/course_test.py --close-px 1500 2200 --trim -0.6 --param $(cat software/bench_params.txt)
                                                                           # prints the plan; --go flies it
```

## The ground station: `ground_station/`

The GUI for the bench and the arena: motors, sensors, the forward camera's rows (relayed by the S3) and the console in one
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
