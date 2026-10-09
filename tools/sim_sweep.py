#!/usr/bin/env python3
"""sim_sweep.py -- run the closed-loop simulation across noise seeds and
altitude-sensor setups, and print one summary table.

    python tools/sim_sweep.py                          both courses, none + tof, seeds 0-4
    python tools/sim_sweep.py --courses 1 --altitude-sources tof --seeds 0-9
    python tools/sim_sweep.py --envelope-diameter 0.69
    python tools/sim_sweep.py --markdown               table ready for README.md / docs/10

WHY THIS EXISTS. One run is one draw of the sensor noise. On course 1 with no
altitude sensor, seeds 0-4 gave best laps from 20.0 s to 48.3 s, so a single
run cannot show whether a change helped. Judge changes against the spread.

WHY 220 s. The first 60 s of every run are gyro-bias convergence
(BLIMP_EST_CONVERGE_S in core/src/estimator.c). run_course.py's 150 s default
therefore usually ends a run before the third loop, which looks like a failure
but is only the clock running out.

WHAT IT DOES NOT DO. It contains no simulation logic: it runs the real harness
(sim/run_course.py) with the project venv, in parallel, and parses what that
prints. It does not rebuild the controller - run `python tools/build.py dll`
first, or every run tests the previous build of core/.

Scenes are generated into build/sweep/ and passed with --xml. Letting
run_course.py generate them would overwrite the committed
sim/scene/course{1,2}.xml whenever --envelope-diameter is used, and parallel
runs would race on the same file.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv"
SWEEP_DIR = REPO_ROOT / "build" / "sweep"
DLL = REPO_ROOT / "build" / "libblimp.dll"

ROW_RE = re.compile(r"^\s*(\d+)\s*\|\s*(COMPLETED|incomplete)\s*\|\s*([\d.]+)?s?\s*(?:--)?\s*\|", re.M)
BEST_RE = re.compile(r"BEST TIME:\s*([\d.]+)s")
# The "Time spent per guidance state" block; not the final "guidance_state: FAILSAFE" line.
FAILSAFE_RE = re.compile(r"^\s+FAILSAFE\s+([\d.]+)s\s*$", re.M)


def venv_python() -> Path:
    """The venv interpreter (absolute: a relative one fails on Windows with a
    bare 'cannot find the file specified'), or this interpreter as a fallback."""
    candidate = VENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if candidate.exists():
        return candidate.resolve()
    print(f"NOTE: no .venv found, using {sys.executable}. "
          "Create the project environment with: python tools/setup_env.py")
    return Path(sys.executable)


def parse_list(text: str) -> list[int]:
    """'0-4' -> [0, 1, 2, 3, 4]; '0,3,7' -> [0, 3, 7]."""
    out: list[int] = []
    for part in text.split(","):
        if "-" in part:
            lo, hi = part.split("-", 1)
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(part))
    return out


@dataclass
class Run:
    course: int
    alt: str
    seed: int
    loop_times: list[float | None]
    best: float | None
    failsafe_s: float
    error: str | None = None


def generate_scene(py: Path, course: int, diameter: float | None) -> Path:
    name = f"course{course}" + (f"_d{diameter:g}" if diameter is not None else "") + ".xml"
    out = SWEEP_DIR / name
    cmd = [str(py), "sim/scene/build_scene.py", "--course", str(course), "--out", str(out)]
    if diameter is not None:
        cmd += ["--envelope-diameter", str(diameter)]
    res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True)
    if res.returncode != 0:
        sys.exit(f"scene generation failed for course {course}:\n{res.stderr}")
    return out


def run_one(py: Path, xml: Path, course: int, alt: str, seed: int,
            max_time: float, diameter: float | None) -> Run:
    cmd = [str(py), "sim/run_course.py", "--course", str(course), "--altitude-source", alt,
           "--seed", str(seed), "--max-time", str(max_time), "--xml", str(xml), "--quiet"]
    if diameter is not None:
        cmd += ["--envelope-diameter", str(diameter)]
    res = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True, text=True,
                         encoding="utf-8", errors="replace")
    out = res.stdout
    if "RESULT:" not in out:
        tail = (res.stderr or out).strip().splitlines()[-5:]
        return Run(course, alt, seed, [], None, 0.0, error=" | ".join(tail) or f"exit {res.returncode}")
    loops = [float(m.group(3)) if m.group(2) == "COMPLETED" else None
             for m in ROW_RE.finditer(out)]
    best = BEST_RE.search(out)
    fs = FAILSAFE_RE.search(out)
    return Run(course, alt, seed, loops, float(best.group(1)) if best else None,
               float(fs.group(1)) if fs else 0.0)


def summarise(runs: list[Run], loops_per_run: int) -> dict:
    ok = [r for r in runs if r.error is None]
    bests = [r.best for r in ok if r.best is not None]
    return {
        "runs": len(runs),
        "errors": len(runs) - len(ok),
        "loops_done": sum(1 for r in ok for t in r.loop_times if t is not None),
        "loops_possible": loops_per_run * len(ok),
        "best_lo": min(bests) if bests else None,
        "best_hi": max(bests) if bests else None,
        "failsafe_runs": sum(1 for r in ok if r.failsafe_s > 0),
    }


def fmt_range(lo: float | None, hi: float | None) -> str:
    if lo is None:
        return "none"
    return f"{lo:.1f} s" if f"{lo:.1f}" == f"{hi:.1f}" else f"{lo:.1f}-{hi:.1f} s"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--courses", default="1,2", help="e.g. 1 or 1,2 (default 1,2)")
    ap.add_argument("--altitude-sources", default="none,tof",
                    help="comma list of none, baro, tof, both (default none,tof)")
    ap.add_argument("--seeds", default="0-4", help="e.g. 0-4 or 0,3,7 (default 0-4)")
    ap.add_argument("--max-time", type=float, default=220.0,
                    help="simulated seconds per run (default 220; see WHY 220 s)")
    ap.add_argument("--envelope-diameter", type=float, default=None,
                    help="m; omit for build_scene.py's default (0.78 m)")
    ap.add_argument("--jobs", type=int, default=min(4, os.cpu_count() or 1),
                    help="runs in parallel (default: up to 4)")
    ap.add_argument("--markdown", action="store_true", help="print the summary as a markdown table")
    ap.add_argument("--verbose", "-v", action="store_true", help="also print every run")
    ns = ap.parse_args(argv if argv is not None else sys.argv[1:])

    courses = parse_list(ns.courses)
    alts = [a.strip() for a in ns.altitude_sources.split(",") if a.strip()]
    seeds = parse_list(ns.seeds)

    if not DLL.exists():
        print(f"FAIL: {DLL.relative_to(REPO_ROOT)} not found. Build it first: python tools/build.py dll")
        return 1

    py = venv_python()
    SWEEP_DIR.mkdir(parents=True, exist_ok=True)
    scenes = {c: generate_scene(py, c, ns.envelope_diameter) for c in courses}

    jobs = [(c, a, s) for c in courses for a in alts for s in seeds]
    print(f"{len(jobs)} runs: courses {courses}, altitude {alts}, seeds {seeds}, "
          f"{ns.max_time:g} s each, {ns.jobs} in parallel ...", flush=True)
    with ThreadPoolExecutor(max_workers=ns.jobs) as pool:
        runs = list(pool.map(
            lambda j: run_one(py, scenes[j[0]], j[0], j[1], j[2], ns.max_time, ns.envelope_diameter),
            jobs))

    if ns.verbose:
        for r in runs:
            laps = " ".join("x" if t is None else f"{t:.2f}" for t in r.loop_times) or "-"
            note = f"ERROR {r.error}" if r.error else f"failsafe {r.failsafe_s:.1f} s"
            print(f"  course {r.course}  {r.alt:<5} seed {r.seed:<3} loops: {laps:<22} {note}")

    envelope = f"{ns.envelope_diameter:g} m" if ns.envelope_diameter else "0.78 m (default)"
    print(f"\nenvelope {envelope}, seeds {ns.seeds}, up to 3 loops within {ns.max_time:g} s per run")
    rows = []
    for c in courses:
        for a in alts:
            s = summarise([r for r in runs if r.course == c and r.alt == a], 3)
            rows.append((c, a, s))

    if ns.markdown:
        print("\n| Course | Altitude sensor | Loops completed | Best laps | Runs with failsafe |")
        print("|---|---|---|---|---|")
        for c, a, s in rows:
            print(f"| {c} | {a} | {s['loops_done']} of {s['loops_possible']} | "
                  f"{fmt_range(s['best_lo'], s['best_hi'])} | {s['failsafe_runs']} of {s['runs'] - s['errors']} |")
    else:
        print(f"\n{'course':<7}{'altitude':<10}{'loops':<12}{'best laps':<16}{'failsafe runs':<14}")
        for c, a, s in rows:
            print(f"{c:<7}{a:<10}{str(s['loops_done']) + '/' + str(s['loops_possible']):<12}"
                  f"{fmt_range(s['best_lo'], s['best_hi']):<16}"
                  f"{str(s['failsafe_runs']) + '/' + str(s['runs'] - s['errors']):<14}")

    errors = sum(s["errors"] for _, _, s in rows)
    if errors:
        for r in runs:
            if r.error:
                print(f"ERROR course {r.course} {r.alt} seed {r.seed}: {r.error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
