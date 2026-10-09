---
name: sim-sweep
description: Measure the blimp's simulated performance across sensor-noise seeds and altitude-sensor setups with tools/sim_sweep.py, compare it with the recorded baseline, and refresh the results tables in README.md, docs/10 and docs/09. Use after changes to core/ or sim/ that can affect flight, when asked for lap times or how well the blimp flies, or to compare envelope sizes.
argument-hint: "[--courses 1,2] [--altitude-sources none,tof] [--seeds 0-4] [--envelope-diameter D]"
---

# Measuring simulated performance

If the request names courses, altitude sources, seeds or an envelope diameter, pass them to the
script as options (`python tools/sim_sweep.py --help` lists them).

1. Rebuild the controller with `python tools/build.py dll`. The sweep refuses to run without a
   DLL but cannot tell whether it is stale.
2. Run `python tools/sim_sweep.py -v` with those options. The default is both courses, no
   altitude sensor and ToF, seeds 0-4, 220 s per run: 20 runs, a few minutes with 4 in parallel.
   Add `--markdown` for a table that can be pasted into a document.
3. Compare with the recorded baseline, the seed-spread table in `docs/10-known-issues.md`, which
   uses the same defaults. Name what moved: loops completed, the best-lap range, and runs with
   failsafe. A shift that stays inside the seed spread is not evidence of an improvement.
4. Report the numbers together with the settings: seeds, `--max-time`, envelope, and whether the
   code had uncommitted changes.

## Reading the results

- **loops**: completed out of 3 per run. Check the per-run times (`-v`) before calling an
  incomplete third loop a failure; it can be the 220 s clock running out.
- **failsafe runs**: runs that spent any time in the FAILSAFE state (ceiling breach, link loss or
  estimator fault).
- Thrust and gondola mass are placeholders, so compare runs with each other. Do not present
  absolute lap times as predictions for the real blimp.

## Updating the recorded results

Only when asked, or when committing a change that moved the numbers. Keep all three consistent and
state the settings next to each table:

- `README.md`, "Simulated lap times": seeds 0-4, 220 s.
- `docs/10-known-issues.md`: the status block (seed 0 rows) and the seed-spread table.
- `docs/09-simulation.md`: the status table (seed 0, 220 s).

Get the seed 0 rows with `python tools/sim_sweep.py --seeds 0 -v`.
