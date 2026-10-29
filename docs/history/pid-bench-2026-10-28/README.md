# PID bench, 2026-10-28: the thrust sweep from the scale

Raw session data for `docs/pid-tuning-plan.md` (P0). The team measured the thrust of the rear
pair on a scale before the tools existed, duty from 0 to 75 % in steps of 3 to 5 %, and sent the
workbook `Empuje motores.xlsx` (one sheet, Spanish headers). This folder holds it as the CSV
`thrust_map.py fit` reads and the fit it produced. What the workbook says, what it does not,
and what was asked back are below; the fit's printout is `thrust-map.txt`, the curves
`thrust-map.json`.

## What was measured

`thrust-sweep-scale.csv`: four series, each 21 points, duty 0.02 … 0.75, grams on the scale
(0.1 g resolution). "forward" is a positive command on the link, "reverse" a negative one;
the workbook called them "valores positivos" and "valores negativos".

| Series | Workbook columns | At 0.29 duty | At 0.75 duty |
|---|---|---|---|
| RL forward | B, E rows 29–49 | 0.5 g | 3.2 g |
| RL reverse | H, J | 1.0 g | 5.1 g |
| RR reverse | N, P | 2.1 g | 8.5 g |
| RR forward | S, U | 1.2 g | 5.0 g |

Not used for the fit, kept for the record:

- Rows 2–18, "propela grande" (big propeller) forward and reverse, "propela pequeña" (small
  propeller) in its correct and inverse rotation, duty 2.7 … 40.5 % in steps of 2.7 %: at
  29.7 % the big propeller gave 3.2 g forward, the small one 1.6 g; reverse (inverse rotation)
  about 65 % of that at the same duty. No zeros at low duty in this block, against a dead band
  up to 11–14 % in the RL/RR block: a different supply, scale or rig, not recorded.
- Rows 52–72, an unlabelled 21-point series whose last six points are RL forward's; left out.

## What the numbers say

- **The negative command gives more thrust than the positive, on both motors**: RR 8.5 g
  against 5.0 g at 0.75, RL 5.1 against 3.2 (ratios 0.59 and 0.63, where AGENTS.md's one
  earlier figure for the inefficient direction was 0.74). Either the propellers are mounted for
  the negative command, or the positive points were read with the propwash blowing onto the
  scale (which pushes the platform down by about the thrust). Asked back: how the motor faced
  the scale for each sign, and which sign pushes the mounted vehicle UP.
- **The two motors differ by 40 % in the same direction** (RR reverse 8.5 g, RL reverse 5.1 g at
  0.75). Not a propeller-size question if both carry the same propeller; a motor or wiring
  question.
- **Magnitudes**: at the amplitudes the plan wrote for the heave step (0.15 to 0.30 duty) the pair
  gives 0.3 to 3 gf in total, under the 6 gf trim, so the vehicle would not stop sinking.
  The heave steps were moved to 0.40 to 0.70 duty (plan §4, H1), where the pair gives about
  4 to 12 gf in its stronger direction.
- **FL and FR were not measured.** The yaw fit does not need them (K comes out per unit duty);
  the conversion to I_z and D_yaw does, and `step_response.py` can be re-run on the saved
  capture once they are, with the stand or with the same scale method. Until then the fit JSON
  carries RL and RR only, and the tool applies their mean curve to FL and FR and says so.
- The dead band: readings stay at 0 g up to 0.11 duty on three of the four series. Either the
  motors do not turn below about 10 % at this supply, or the thrust is under the scale's
  resolution; the affine fit's u0 in `thrust-map.txt` is the number.

## Not known from the workbook

The supply (the cell through the step-up, or a bench supply), the voltage during the sweep,
the firmware build (the deadband default changed to 0 at `6a91878`; before that a command c was
the duty db + (1 − db)·|c|), the scale, the rig and the propeller fitted. The values are used
as duty = command.
