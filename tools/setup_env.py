#!/usr/bin/env python3
"""setup_env.py -- create the project's Python environment, from scratch, in one command.

    python tools/setup_env.py

Creates .venv/ in the repo root and installs the exact pinned versions from
requirements.txt into it. Safe to re-run: an existing, healthy .venv is reused
and its packages re-checked against requirements.txt.

WHY A VENV AND NOT A GLOBAL INSTALL. On 2026-09-21 MuJoCo was pip-installed
into a developer's personal Python, which also carried TensorFlow, pandas and
scikit-learn for unrelated work. That is two problems in one: the competition
robot and somebody's day job can silently break each other, and nobody else on
the team can reproduce the environment the simulation results came from. Both
matter here, because the simulated lap times are only comparable if everyone
runs the same pinned physics engine.

WHAT THIS SCRIPT DOES NOT TOUCH:
  - the C control core (core/). It is plain C11 built by tools/build.py with
    clang; it has no Python dependency at all.
  - the firmware (firmware/). ESP-IDF manages its OWN Python 3.11 environment
    under ~/.espressif/python_env/, created by the ESP-IDF installer. Use
    tools/idf_build.bat for that; do not mix the two.

This script deliberately uses ONLY the Python standard library, so it runs on a
machine where nothing has been installed yet.
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import venv
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
VENV_DIR = REPO_ROOT / ".venv"
REQUIREMENTS = REPO_ROOT / "requirements.txt"

# Minimum Python for the simulation. MuJoCo 3.13 wheels start at 3.9; the
# project is developed and verified on 3.12.
MIN_PYTHON = (3, 9)
VERIFIED_PYTHON = (3, 12)


def venv_python(venv_dir: Path) -> Path:
    """Path to the interpreter inside a venv, on either platform."""
    if os.name == "nt":
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"


def activate_hint(venv_dir: Path) -> str:
    """The shell-appropriate line for activating the venv, for printing only.

    Activation is a CONVENIENCE, never a requirement: every command in this
    project can be run by calling the venv's interpreter by path instead, which
    is what tools/verify.py does. Saying so here avoids the classic failure of a
    teammate forgetting to activate and silently using the wrong Python.
    """
    rel = venv_dir.relative_to(REPO_ROOT) if venv_dir.is_relative_to(REPO_ROOT) else venv_dir
    if os.name == "nt":
        return (
            f"  PowerShell   .\\{rel}\\Scripts\\Activate.ps1\n"
            f"  cmd.exe      {rel}\\Scripts\\activate.bat\n"
            f"  Git Bash     source {rel}/Scripts/activate"
        )
    return f"  bash/zsh     source {rel}/bin/activate"


def check_host_python() -> None:
    if sys.version_info < MIN_PYTHON:
        sys.exit(
            f"FAIL: Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]}+ is required to build the "
            f"simulation environment; this interpreter is "
            f"{sys.version_info.major}.{sys.version_info.minor}."
        )
    if sys.version_info[:2] != VERIFIED_PYTHON:
        print(
            f"NOTE: this project is verified on Python "
            f"{VERIFIED_PYTHON[0]}.{VERIFIED_PYTHON[1]}; you are on "
            f"{sys.version_info.major}.{sys.version_info.minor}. That is usually fine, "
            f"but if MuJoCo wheels fail to install, try "
            f"{VERIFIED_PYTHON[0]}.{VERIFIED_PYTHON[1]}."
        )


def run(cmd: list[str], what: str) -> None:
    print(f"  -> {what}")
    result = subprocess.run(cmd, cwd=REPO_ROOT)
    if result.returncode != 0:
        sys.exit(f"FAIL: {what} (exit {result.returncode})\n  command: {' '.join(cmd)}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recreate", action="store_true",
                    help="delete an existing .venv and build it again from scratch")
    ap.add_argument("--quiet", action="store_true", help="less pip output")
    ns = ap.parse_args(argv if argv is not None else sys.argv[1:])

    print(f"DroneUN environment setup")
    print(f"  repo        {REPO_ROOT}")
    print(f"  host python {sys.version.split()[0]} ({platform.system()})")
    print()

    check_host_python()

    if not REQUIREMENTS.exists():
        sys.exit(f"FAIL: {REQUIREMENTS} is missing - cannot pin the environment.")

    py = venv_python(VENV_DIR)

    if ns.recreate and VENV_DIR.exists():
        print(f"  removing existing {VENV_DIR.name}/ (--recreate)")
        shutil.rmtree(VENV_DIR)

    if py.exists():
        print(f"  reusing existing {VENV_DIR.name}/")
    else:
        print(f"  creating {VENV_DIR.name}/")
        # with_pip=True is the default but stated explicitly: some Linux distros
        # ship python3-venv without ensurepip, and the failure is otherwise cryptic.
        try:
            venv.EnvBuilder(with_pip=True, clear=False).create(VENV_DIR)
        except Exception as exc:
            sys.exit(
                f"FAIL: could not create the virtual environment: {exc}\n"
                f"  On Debian/Ubuntu you may need:  sudo apt install python3-venv"
            )
        if not py.exists():
            sys.exit(f"FAIL: venv created but no interpreter at {py}")

    quiet = ["--quiet"] if ns.quiet else []
    run([str(py), "-m", "pip", "install", "--upgrade", "pip", *quiet],
        "upgrading pip inside the venv")
    run([str(py), "-m", "pip", "install", "-r", str(REQUIREMENTS), *quiet],
        "installing pinned requirements")

    # Prove the environment actually works rather than assuming pip's exit code
    # is the whole story. Importing mujoco exercises its native library load,
    # which is the part that actually fails on a bad install.
    print("  -> verifying imports")
    check = subprocess.run(
        [str(py), "-c",
         "import mujoco, numpy; print(f'     mujoco {mujoco.__version__}, numpy {numpy.__version__}')"],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    if check.returncode != 0:
        sys.exit(f"FAIL: the installed packages do not import:\n{check.stderr}")
    print(check.stdout.rstrip())

    print()
    print("Environment ready.")
    print()
    print("Run things with the venv interpreter directly (no activation needed):")
    if os.name == "nt":
        print(r"  .venv\Scripts\python.exe sim\run_course.py --course 1")
    else:
        print("  .venv/bin/python sim/run_course.py --course 1")
    print()
    print("Or activate it for the session:")
    print(activate_hint(VENV_DIR))
    print()
    print("Then verify the whole project (C core, DLL, scene, simulation):")
    print("  python tools/verify.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
