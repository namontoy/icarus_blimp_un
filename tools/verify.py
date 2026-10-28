#!/usr/bin/env python3
"""verify.py -- one command that checks the whole project is healthy.

    python tools/verify.py              fast suite   (~15 s)  run this often
    python tools/verify.py --full       full suite   (~2 min) run before flying
    python tools/verify.py --firmware   also build the ESP32-S3 firmware
    python tools/verify.py --all        everything

Exit code 0 means every check passed. Non-zero means at least one failed, and
the failing step's own output is printed. Nothing here reimplements a check:
this script only sequences the tools that already exist.

WHY THIS EXISTS. Verifying this project by hand means remembering six commands
across three toolchains, run in the right order, from the right interpreter.
The order genuinely matters: the simulation loads the C core through
build/libblimp.dll, so a stale DLL silently tests OLD control code against a
NEW scene, and both halves look green while proving nothing. Here the DLL is
always rebuilt before anything loads it.

THIS IS A SAFETY CHECK, NOT JUST HYGIENE. Rule 13 caps total thrust at 100
grams-force, and compliance is enforced by a unit test that sweeps 114,244
actuator demands (core/tests/test_mixer.c), including NaN, infinity and
corrupted parameters. If that test stops running, nothing else in the project
will notice that the vehicle can be commanded past a rule limit. A green run
of this script is the statement that the thrust budget still holds.

WHAT EACH SUITE COVERS:

  fast (default)
    1. C control core unit tests        11,276 checks, includes the Rule 13 sweep
    2. build/libblimp.dll               so the simulation tests CURRENT core code
    3. FFI struct layout + DLL smoke    catches C/Python struct drift, which has
                                        previously caused a silent segfault
    4. scene generation, both courses
    5. scene physics checks             buoyancy, balloon heights, course geometry
    6. one closed-loop lap, course 1    proves the vehicle still flies at all
    7. agent skills in sync             Codex copies in .agents/skills match the
                                        .claude/skills originals (tools/sync_skills.py)

  full (adds)
    8. both courses x 3 loops, with and without a simulated altitude sensor

  firmware (adds)
    9. idf.py build for the ESP32-S3    via tools/idf_build.bat, which handles
                                        the four documented Windows/ESP-IDF traps
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv"

GREEN, RED, YELLOW, DIM, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"
if os.name == "nt" and not os.environ.get("WT_SESSION") and not os.environ.get("TERM"):
    # Old conhost without VT processing renders escapes literally; drop colour.
    GREEN = RED = YELLOW = DIM = OFF = ""


def venv_python() -> Path | None:
    """Absolute path to the venv interpreter, or None if there is no venv.

    ABSOLUTE deliberately: passing a relative interpreter path to subprocess on
    Windows fails with a bare 'cannot find the file specified' that says nothing
    about which file, which cost time to diagnose once already.
    """
    candidate = VENV_DIR / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    return candidate.resolve() if candidate.exists() else None


class Runner:
    def __init__(self, verbose: bool) -> None:
        self.verbose = verbose
        self.results: list[tuple[str, bool, float, str]] = []

    def step(self, label: str, cmd: list[str], *, shell: bool = False) -> bool:
        print(f"  {label:<46s}", end="", flush=True)
        t0 = time.time()
        proc = subprocess.run(
            " ".join(cmd) if shell else cmd,
            cwd=REPO_ROOT, capture_output=True, text=True, shell=shell,
        )
        elapsed = time.time() - t0
        ok = proc.returncode == 0
        output = (proc.stdout or "") + (proc.stderr or "")
        print(f"{GREEN}PASS{OFF}" if ok else f"{RED}FAIL{OFF}", f"{DIM}{elapsed:5.1f}s{OFF}")
        if not ok or self.verbose:
            tail = output.strip().splitlines()
            shown = tail if self.verbose else tail[-25:]
            for line in shown:
                print(f"      {DIM}{line}{OFF}")
        self.results.append((label, ok, elapsed, output))
        return ok


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--full", action="store_true", help="add the long simulation matrix")
    ap.add_argument("--firmware", action="store_true", help="also build the ESP32-S3 firmware")
    ap.add_argument("--all", action="store_true", help="--full and --firmware")
    ap.add_argument("--verbose", "-v", action="store_true", help="show output of passing steps too")
    ap.add_argument("--keep-going", action="store_true",
                    help="run every step even after one fails (default: stop at the first failure)")
    ns = ap.parse_args(argv if argv is not None else sys.argv[1:])
    if ns.all:
        ns.full = ns.firmware = True

    vp = venv_python()
    if vp is None:
        print(f"{YELLOW}NOTE{OFF}: no .venv found, falling back to {sys.executable}")
        print(f"      Create the project environment with:  python tools/setup_env.py")
        print()
        vp = Path(sys.executable)

    host = sys.executable            # builds C; needs no third-party packages
    sim = str(vp)                    # runs the simulation; needs mujoco + numpy

    print(f"DroneUN verification")
    print(f"  repo         {REPO_ROOT}")
    print(f"  host python  {host}")
    print(f"  sim python   {sim}")
    suite = "all" if (ns.full and ns.firmware) else "full" if ns.full else "firmware+fast" if ns.firmware else "fast"
    print(f"  suite        {suite}")
    print()

    r = Runner(ns.verbose)
    ok = True

    def gate() -> bool:
        return ok or ns.keep_going

    print(f"{DIM}control core (C){OFF}")
    if gate():
        ok &= r.step("unit tests, incl. the Rule 13 thrust sweep",
                     [host, "tools/build.py", "all"])
    if gate():
        ok &= r.step("build libblimp for the simulation",
                     [host, "tools/build.py", "dll"])

    print(f"{DIM}simulation{OFF}")
    if gate():
        ok &= r.step("FFI struct layout + DLL smoke test",
                     [sim, "sim/blimp_ffi.py"])
    if gate():
        ok &= r.step("generate scene, course 1",
                     [sim, "sim/scene/build_scene.py", "--course", "1"])
    if gate():
        ok &= r.step("generate scene, course 2",
                     [sim, "sim/scene/build_scene.py", "--course", "2"])
    if gate():
        ok &= r.step("scene physics checks",
                     [sim, "sim/scene/verify_scene.py"])
    if gate():
        ok &= r.step("closed loop, course 1, one lap",
                     [sim, "sim/run_course.py", "--course", "1", "--loops", "1",
                      "--max-time", "120", "--quiet"])

    print(f"{DIM}agent files{OFF}")
    if gate():
        ok &= r.step("skills in sync (.agents from .claude)",
                     [host, "tools/sync_skills.py", "--check"])

    if ns.full:
        print(f"{DIM}simulation matrix{OFF}")
        matrix = [
            ("course 1, 3 loops, no altitude sensor", ["--course", "1", "--altitude-source", "none"]),
            ("course 2, 3 loops, no altitude sensor", ["--course", "2", "--altitude-source", "none"]),
            ("course 1, 3 loops, with ToF",           ["--course", "1", "--altitude-source", "tof"]),
            ("course 2, 3 loops, with ToF",           ["--course", "2", "--altitude-source", "tof"]),
        ]
        for label, args in matrix:
            if not gate():
                break
            ok &= r.step(label,
                         [sim, "sim/run_course.py", *args, "--loops", "3",
                          "--max-time", "220", "--quiet"])

    if ns.firmware:
        print(f"{DIM}firmware (ESP32-S3){OFF}")
        if gate():
            if os.name != "nt":
                print(f"  {YELLOW}SKIP{OFF}  firmware build is Windows-only here "
                      f"(tools/idf_build.bat); on Linux run idf.py build in firmware/projects/espfly/")
            else:
                # Invoked through cmd.exe deliberately: idf_build.bat handles four
                # documented environment traps (inherited MSYSTEM, an undefined
                # PROMPT, the Store-Python/venv mismatch, and stderr capture) that
                # calling idf.py directly from this process would re-introduce.
                ok &= r.step("idf.py build",
                             ["cmd", "/c", "tools\\idf_build.bat", "build"], shell=False)

    print()
    passed = sum(1 for _, good, _, _ in r.results if good)
    total = len(r.results)
    elapsed = sum(t for _, _, t, _ in r.results)
    if ok:
        print(f"{GREEN}ALL CHECKS PASSED{OFF}  ({passed}/{total} steps, {elapsed:.1f}s)")
        if not ns.full:
            print(f"{DIM}  run with --full before flying, and --all before a competition day{OFF}")
        return 0

    print(f"{RED}FAILED{OFF}  ({passed}/{total} steps passed, {elapsed:.1f}s)")
    for label, good, _, _ in r.results:
        if not good:
            print(f"  {RED}x{OFF} {label}")
    if not ns.keep_going:
        print(f"{DIM}  stopped at the first failure; use --keep-going to run the rest{OFF}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
