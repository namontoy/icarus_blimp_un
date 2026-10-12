# Recorded flow traces

Hardware runs backing §8.14 and §8.15 of
[`docs/esp32-cam-optical-flow-report.md`](../../../../docs/esp32-cam-optical-flow-report.md).
Keep them: the report's numbers are only checkable against these files.

## The runs

All five are the same mechanical reference — a height-adjustable table moved
between two detents, **73 cm → 100 cm → 73 cm**, so **270 ± 10 mm** out and the
same back, at a taped lens-to-target distance of **1370 mm**. One table speed
(the only one it has), about 24 mm/s, which is ~0.22 px/frame at 8.726 mm/px.

| file | estimator | camera roll | motion lands on | frames |
|---|---|---|---|---|
| `oab5.csv` | vfit | upright | image y | 3148 |
| `oab6.csv` | parabola | upright | image y | 839 |
| `oab7.csv` | vfit | 90° | image x | 895 |
| `oab8.csv` | parabola | 90° | image x | 3121 |
| `oab9.csv` | vfit | 43.5° | **both axes at once** | 1219 |

`oab9-predictions.txt` is what was predicted from `oab5`–`oab8` *before* `oab9`
was recorded, kept so the conclusions in §8.15 can't be read as hindsight.

## Two traps

**1. Every one of these logged with `h1500` in force — 9.554 mm/px — not the
true 1370 mm.** The recorder scripts sent `h1500` at startup and that line was
carried from script to script. **Multiply every `vx`, `vy`, `pos_x`, `pos_y` by
0.9133** before comparing against the 270 mm reference.

Do not trust that correction factor on faith either. The constant actually in
force is recoverable from the data, since `vx = -dx · mmpx / dt`:

```
mmpx = |vx| / |dx| × dt_ms/1000          # fields 6, 4 and 16
```

Take the median over frames where `|dx| > 0.15` so quantisation doesn't
dominate. All five files return 9.558.

**2. `LC_ALL=C`.** The shell locale on this machine prints decimal commas, which
silently corrupts any `awk` parse of these files.

## Field layout

Seventeen comma-separated fields. Field 1 is added by the host recorder; fields
2–17 are the device's own line, emitted by `optflow_main.c:538`.

| # | name | notes |
|---|---|---|
| 1 | `host_ts` | host `time.time()` at the moment the line was read — locates a spoken "start"/"done" in the trace, independent of device time |
| 2 | `t_ms` | device capture timestamp, ms |
| 3 | `ok` | 1 = match accepted, 0 = honest failure |
| 4 | `dx` | image flow, px/frame |
| 5 | `dy` | image flow, px/frame |
| 6 | `vx` | body velocity, mm/s — **negated** flow (ground left ⇒ airframe right) |
| 7 | `vy` | body velocity, mm/s |
| 8 | `pos_x` | integrated position, mm, since the last `z` |
| 9 | `pos_y` | integrated position, mm |
| 10 | `spread` | dispersion of the 25 per-block estimates |
| 11 | `n_valid` | blocks that produced a usable match, of 25 |
| 12 | `n_total` | 25 |
| 13 | `n_flat` | blocks rejected: below `min_var` |
| 14 | `n_edge` | blocks rejected: minimum on the search boundary |
| 15 | `n_ambig` | blocks rejected: ambiguous minimum |
| 16 | `dt_ms` | frame interval, ms (80.0 at 12.5 FPS) |
| 17 | `proc_us` | matching time, µs |

The three rejection counters are **not** contiguous with `n_valid`/`n_total` in
the way the names suggest reading them: `n_flat`, `n_edge` and `n_ambig` are
fields 13, 14 and 15, with `dt_ms` landing *after* `n_ambig` at 16. Check
`optflow_main.c:538` before trusting any field past 12.

## Reading a run

Integrated position is `Σ(−dx · mm_per_px)`, so `dt` cancels and frame-timing
error cannot bias it. With both an out and a back leg, gain and a constant bias
solve together per axis — two equations, two unknowns:

```
out = +g·L + b·T      back = −g·L + b·T
g·L = (|out| + |back|)/2        b·T = (out + back)/2
```

Using the *mean* of the two legs is what makes a gain figure bias-free; a
single leg conflates the two, which is why §8.9 exists.
