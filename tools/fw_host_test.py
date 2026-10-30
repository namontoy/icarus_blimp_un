#!/usr/bin/env python3
"""Host tests for the S3 firmware's portable files: compiled with clang and
tools/build.py's flags, run on this computer, no ESP-IDF.

Today one: the 'param' command's table (firmware/projects/i2c_tools/main/
param_table.c) against core/src/params.c, checked by
firmware/projects/i2c_tools/host/test_param_table.c. tools/verify.py runs this.

    python tools/fw_host_test.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build import BUILD, CFLAGS, LDLIBS, ROOT, run  # noqa: E402

FW = ROOT / "firmware" / "projects" / "i2c_tools"

TESTS = {
    "param_table": [ROOT / "core" / "src" / "params.c", FW / "main" / "param_table.c"],
}


def main() -> int:
    BUILD.mkdir(exist_ok=True)
    ok = True
    for name, srcs in TESTS.items():
        test = FW / "host" / f"test_{name}.c"
        exe = BUILD / f"fw_test_{name}.exe"
        cmd = ["clang", *CFLAGS, f"-I{FW / 'main'}", f"-I{ROOT / 'core' / 'tests'}",
               *[str(s) for s in srcs], str(test), *LDLIBS, "-o", str(exe)]
        rc, out = run(cmd)
        if rc != 0:
            print(f"COMPILE FAIL {name}\n{out.strip()[:4000]}")
            ok = False
            continue
        rc, out = run([str(exe)])
        print(f"{'PASS' if rc == 0 else 'FAIL'} {name}")
        print("\n".join("     " + ln for ln in out.strip().splitlines()[:60]))
        ok &= rc == 0
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
