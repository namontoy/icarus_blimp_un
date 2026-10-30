---
# Generated from .claude/skills/core-change/SKILL.md by tools/sync_skills.py - edit that file, not this one.
name: core-change
description: Checklist for changing the blimp's C flight controller in core/ (estimator, guidance, PID loops, mixer, parameters, shared structs) without breaking the simulator, the firmware or the Rule 13 thrust cap. Use when editing core/src or core/include/blimp, adding a tunable, or changing a struct.
---

# Changing the flight controller (`core/`)

`blimp_step()` is the whole controller. The same source compiles into the firmware and into
`build/libblimp.dll` for the simulator, so every change here changes both.

## Before editing

- Read the module's header in `core/include/blimp/`. The headers carry the contract: units, valid
  ranges, and which inputs are levels rather than edges.
- Read the "Contract notes" table in `docs/10-known-issues.md` §5; it lists traps already hit.

## While editing

- Freestanding C11 plus `<math.h>` only: no allocation, no mutable globals, no blocking, and the
  same inputs must always give the same outputs.
- Non-finite or out-of-range inputs must still produce a safe output (all-zero thrust), as the
  existing modules do.
- **New tunable:** add a field to `blimp_params_t` (`core/include/blimp/params.h`), its default
  and the reasoning behind it in `core/src/params.c`, the same field at the same position in
  `BlimpParams` in `sim/blimp_ffi.py`, and an entry with its accepted range in
  `firmware/projects/i2c_tools/main/param_table.c`, so the S3's `param` command can set it
  (`tools/fw_host_test.py`, part of `tools/verify.py`, fails on a field with no entry).
- **Changed struct** that crosses the simulator boundary (listed in `_C_ORDER` in
  `sim/blimp_ffi.py`): update its ctypes mirror field for field in declaration order, and update
  `_EXPECTED_LAYOUT`. A new struct crossing the boundary also needs a case in
  `blimp_ffi_struct_size()` in `core/src/blimp_ffi.c` and an entry in `_C_ORDER`.

## Test

1. Add or extend checks in `core/tests/test_<module>.c`, using `CHECK`, `CHECK_NEAR`,
   `CHECK_FINITE` and `TEST_GROUP` from `core/tests/test_util.h`.
2. Run `python tools/build.py module <module>` until it passes. Warnings are errors: fix them,
   never silence them.
3. For any mixer or parameter change, run `python tools/build.py module mixer`. The Rule 13 sweep
   must pass without being edited.
4. Run `python tools/build.py dll`, then `python tools/verify.py --full`.
5. For a header or parameter change, also run `python tools/verify.py --firmware`, because the
   firmware compiles the same headers. If ESP-IDF is not installed, say so; don't skip silently.
6. If the change can affect how the blimp flies, follow the sim-sweep skill
   (`python tools/sim_sweep.py`) and compare with the recorded table in docs/10.

## Finish

- Report what changed, the test results with their check counts, and the sweep comparison if one
  was run.
- If behaviour or a known issue changed, update `docs/10-known-issues.md`.
