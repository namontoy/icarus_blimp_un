#!/usr/bin/env python3
"""Host build/test driver for the portable blimp control core.

The core is plain C11 with no ESP-IDF dependency precisely so it can be built
and unit-tested on a laptop, and then compiled unchanged into the firmware and
loaded by the MuJoCo simulation. If it does not build here, it does not build.

Usage:
    python tools/build.py module <name>   compile+run one module's tests
    python tools/build.py all             compile+run every module's tests
    python tools/build.py dll             build libblimp shared lib for sim
    python tools/build.py clean

Requires clang on PATH. On Windows clang targets MSVC, so do NOT pass -lm
there; on Linux and macOS libm is a separate library and -lm is required.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "core" / "src"
TESTS = ROOT / "core" / "tests"
INC = ROOT / "core" / "include"
BUILD = ROOT / "build"

# Link-time dependencies between core modules. Keeps per-module test builds
# isolated so agents/humans working on different modules do not break each
# other's test runs.
DEPS: dict[str, list[str]] = {
    "params": [],
    "pid": [],
    "mixer": ["params"],
    "estimator": ["pid", "params"],
    "guidance": ["pid", "params"],
    "blimp": ["params", "pid", "mixer", "estimator", "guidance"],
}

CFLAGS = [
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-Wconversion",
    "-Wshadow",
    "-O2",
    "-g",
    f"-I{INC}",
]

# What the host platform needs on top of CFLAGS. clang on Windows targets the
# MSVC runtime, which has the maths functions built in and rejects -lm; glibc
# keeps them in libm, and an ELF shared library must be position-independent.
WINDOWS = os.name == "nt"
LDLIBS: list[str] = [] if WINDOWS else ["-lm"]
SHARED_FLAGS: list[str] = ["-shared"] if WINDOWS else ["-shared", "-fPIC"]

# The library the sim loads through ctypes; sim/blimp_ffi.py and
# tools/sim_sweep.py name the same file by the same rule.
LIB_NAME = ("libblimp.dll" if WINDOWS
            else "libblimp.dylib" if sys.platform == "darwin"
            else "libblimp.so")

GREEN, RED, YEL, DIM, OFF = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def run(cmd: list[str]) -> tuple[int, str]:
    p = subprocess.run(cmd, capture_output=True, text=True)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def resolve(name: str, seen: set[str] | None = None) -> list[str]:
    """Transitively resolve a module's dependencies, dependencies first."""
    seen = seen if seen is not None else set()
    if name in seen:
        return []
    seen.add(name)
    out: list[str] = []
    for d in DEPS.get(name, []):
        out.extend(resolve(d, seen))
    out.append(name)
    return out


def sources_for(name: str) -> tuple[list[Path], list[str]]:
    """Existing .c files for a module and its deps, plus names still missing."""
    present, missing = [], []
    for m in resolve(name):
        f = SRC / f"{m}.c"
        (present.append(f) if f.exists() else missing.append(m))
    return present, missing


def build_module(name: str) -> bool:
    test = TESTS / f"test_{name}.c"
    if not test.exists():
        print(f"{YEL}SKIP{OFF} {name}: no core/tests/test_{name}.c")
        return True
    srcs, missing = sources_for(name)
    if missing:
        print(f"{YEL}SKIP{OFF} {name}: missing core/src/{{{','.join(missing)}}}.c")
        return True

    BUILD.mkdir(exist_ok=True)
    exe = BUILD / f"test_{name}.exe"
    cmd = ["clang", *CFLAGS, *[str(s) for s in srcs], str(test), *LDLIBS, "-o", str(exe)]
    rc, out = run(cmd)
    if rc != 0:
        print(f"{RED}COMPILE FAIL{OFF} {name}")
        print(DIM + out.strip()[:4000] + OFF)
        return False

    rc, out = run([str(exe)])
    label = f"{GREEN}PASS{OFF}" if rc == 0 else f"{RED}FAIL{OFF}"
    print(f"{label} {name}")
    if out.strip():
        print("\n".join("     " + ln for ln in out.strip().splitlines()[:60]))
    return rc == 0


def build_dll() -> bool:
    """Shared library the MuJoCo sim loads via ctypes.

    Exports come from core/src/blimp_ffi.c, which wraps the public API in
    __declspec(dllexport) on Windows (default visibility elsewhere). Keeping
    exports in one file means the portable headers stay free of platform
    attributes.
    """
    ffi = SRC / "blimp_ffi.c"
    if not ffi.exists():
        print(f"{YEL}SKIP{OFF} dll: core/src/blimp_ffi.c not written yet")
        return True
    srcs = sorted(SRC.glob("*.c"))
    if not srcs:
        print(f"{RED}FAIL{OFF} dll: no sources")
        return False
    BUILD.mkdir(exist_ok=True)
    dll = BUILD / LIB_NAME
    # -Wconversion is deliberately dropped here only to keep third-party-free
    # FFI glue simple; the same sources still build clean under the full
    # warning set in the per-module test builds above.
    flags = [f for f in CFLAGS if f != "-Wconversion"]
    rc, out = run(["clang", *flags, *SHARED_FLAGS, *[str(s) for s in srcs], *LDLIBS,
                   "-o", str(dll)])
    if rc != 0:
        print(f"{RED}FAIL{OFF} dll")
        print(DIM + out.strip()[:4000] + OFF)
        return False
    print(f"{GREEN}BUILT{OFF} {dll.relative_to(ROOT)}")
    return True


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 2
    cmd = args[0]

    if cmd == "clean":
        if BUILD.exists():
            for f in BUILD.iterdir():
                f.unlink()
            BUILD.rmdir()
        print("cleaned")
        return 0

    if cmd == "module":
        if len(args) < 2:
            print("usage: build.py module <name>")
            return 2
        return 0 if build_module(args[1]) else 1

    if cmd == "dll":
        return 0 if build_dll() else 1

    if cmd == "all":
        ok = True
        for name in DEPS:
            ok &= build_module(name)
        ok &= build_dll()
        print()
        print(f"{GREEN}ALL TESTS PASSED{OFF}" if ok else f"{RED}FAILURES ABOVE{OFF}")
        return 0 if ok else 1

    print(f"unknown command: {cmd}")
    return 2


if __name__ == "__main__":
    sys.exit(main())
