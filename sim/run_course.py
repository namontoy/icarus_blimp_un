"""run_course.py — CLI driver for the closed-loop simulation (sim_main.py).

Generates (or reuses) the requested course scene via the REAL
sim/scene/build_scene.py, runs sim_main.ClosedLoopSim against the REAL
compiled control core (build/libblimp.dll, via sim/blimp_ffi.py), and prints
a result table: one row per loop attempt, plus a summary.

This file contains no simulation or control logic of its own — see
sim_main.py for the closed loop and blimp_ffi.py for the C ABI bindings.

Examples:
    python run_course.py --course 1
    python run_course.py --course 2 --loops 3 --seed 7
    python run_course.py --course 1 --envelope-diameter 0.69 --altitude-source tof
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_main  # noqa: E402
import blimp_ffi as ffi  # noqa: E402


def build_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", type=int, choices=[1, 2], default=1,
                     help="1 = rectangle perimeter (Rule 19), 2 = figure-eight (Rule 20)")
    ap.add_argument("--loops", type=int, default=3,
                     help="loop attempts this harness waits for before stopping "
                          "(Rule 5 allows up to 3; see docs/09 for the current "
                          "internal-loop-count caveat)")
    ap.add_argument("--envelope-diameter", type=float, default=None,
                     help="m; omit to use build_scene.py's own default (0.78 m)")
    ap.add_argument("--altitude-source", choices=list(sim_main.ALTITUDE_SOURCES),
                     default="none",
                     help="synthetic altitude sensor: none (the real PCB, altitude "
                          "held from the balloon's elevation), baro, tof or both")
    ap.add_argument("--seed", type=int, default=0, help="sensor-noise RNG seed")
    ap.add_argument("--max-time", type=float, default=150.0,
                     help="sim seconds, total budget across all loop attempts")
    ap.add_argument("--dll", type=str, default=None, help="override build/libblimp.dll path")
    ap.add_argument("--xml", type=str, default=None,
                     help="override the scene XML path entirely (skips scene generation)")
    ap.add_argument("--force-regen", action="store_true",
                     help="regenerate the scene even if course{N}.xml already exists")
    ap.add_argument("--quiet", action="store_true", help="suppress the periodic tick log")
    ap.add_argument("--print-period", type=float, default=2.0)
    return ap


def _fmt_time(t: Optional[float]) -> str:
    return f"{t:8.2f}s" if t is not None else "     --  "


def print_table(result: "sim_main.RunResult") -> None:
    print()
    title = f" RESULT: course {result.course} ({result.course_name}) "
    print(title.center(72, "="))
    print(f" order (Rule 2, runtime-loaded from the scene): {result.order}")
    print(f" envelope_diameter={result.envelope_diameter or 'default (0.78 m)'}  "
          f"altitude_source={result.altitude_source}  seed={result.seed}  "
          f"control_hz={result.control_hz}")
    print("-" * 72)
    header = f" {'loop':>4} | {'status':<11} | {'time':>9} | {'waypoints touched':<20}"
    print(header)
    print("-" * 72)
    for loop in result.loops:
        status = "COMPLETED" if loop.completed else "incomplete"
        n = len(loop.touches)
        print(f" {loop.loop_index:>4} | {status:<11} | {_fmt_time(loop.total_time_s)} | "
              f"{n}/{len(result.order)}")
    print("-" * 72)
    if result.best_time_s is not None:
        n_completed = sum(1 for l in result.loops if l.completed)
        print(f" BEST TIME: {result.best_time_s:.2f}s  ({n_completed}/{result.requested_loops} "
              f"loop attempts completed)")
    else:
        print(f" BEST TIME: none — no loop completed within {result.max_sim_time_s:.1f}s "
              f"of simulated time")
    print(f" total simulated time reached: {result.sim_time_reached_s:.2f}s")
    print("=" * 72)


def main(argv: Optional[List[str]] = None) -> int:
    ns = build_argparser().parse_args(argv if argv is not None else sys.argv[1:])

    if ns.xml:
        xml_path = Path(ns.xml)
    else:
        xml_path = sim_main.resolve_scene_xml(
            ns.course, ns.envelope_diameter, force=ns.force_regen,
        )

    try:
        sim = sim_main.ClosedLoopSim(
            xml_path=xml_path, course=ns.course, envelope_diameter=ns.envelope_diameter,
            altitude_source=ns.altitude_source, seed=ns.seed,
            dll_path=Path(ns.dll) if ns.dll else None,
            print_period_s=ns.print_period, verbose=not ns.quiet,
        )
    except ffi.BlimpFFIError as e:
        print(f"FAIL: {e}")
        return 1

    result = sim.run(max_sim_time_s=ns.max_time, requested_loops=ns.loops)
    sim_main.print_result(result)
    print_table(result)

    if result.warnings and any("NaN/Inf" in w for w in result.warnings):
        print("FAIL: simulation produced NaN/Inf.")
        return 1
    if not result.completed:
        print("FAIL: course not completed (0 loops finished) within the time budget.")
        return 1
    print("PASS: at least one loop completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
