"""blimp_ffi.py — ctypes bindings to build/libblimp.dll.

THE POINT OF THIS FILE (see core/include/blimp/blimp.h's own header comment):
blimp_step() is the ENTIRE flight controller. This module loads the exact
compiled bytes that also run on the ESP32 and calls them directly, so nothing
tuned against this binding drifts from what the vehicle actually does. This
file must NEVER reimplement any control, estimation, guidance or mixing
logic — it only marshals bytes across the C ABI boundary.

Every ctypes.Structure below mirrors a core/include/blimp/*.h struct
FIELD-FOR-FIELD, in DECLARATION ORDER, so its C layout (size, per-field
offset, padding) matches exactly what clang actually emits for the real
struct — not a hand recomputation of C's alignment rules. That layout was
verified directly, not assumed: a throwaway probe
(scratch-only, never under core/) was compiled against the REAL headers
with `#include "blimp/blimp.h"` and printed sizeof()/offsetof() for every
struct below via clang 19.1.1 (x86_64-pc-windows-msvc, the same compiler
tools/build.py uses for the dll). Those printed numbers are reproduced as
the _EXPECTED_LAYOUT table at the bottom of this file, and _selftest()
(run automatically at import time) recomputes ctypes.sizeof()/offsetof()
for every struct here and hard-fails on any mismatch — so a future header
edit that desyncs this mirror is caught immediately, not discovered as a
silent memory-corrupting FFI call.

NOTE on the enum field blimp_guidance_t.state: C leaves an enum's underlying
type implementation-defined, but clang (like every mainstream compiler,
absent a packed-enum pragma neither guidance.h nor its includes use) picks
`int` whenever every enumerator fits, which is true here (11 sequential
values). Confirmed empirically: the probe's `sizeof(blimp_guid_state_t)`
printed 4. Mirrored here as ctypes.c_int, not left to guesswork.

Usage:
    from blimp_ffi import BlimpController, BlimpImuSample, BlimpTargetObs, BlimpInput
    ctl = BlimpController()          # loads build/libblimp.dll, allocates blimp_t
    ctl.init()
    act = ctl.step(some_blimp_input_t)
    state = ctl.get_state()
"""
from __future__ import annotations

import ctypes as C
import os
import sys
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Locating the DLL
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
# Named by the same rule as LIB_NAME in tools/build.py, which builds it.
DEFAULT_DLL_PATH = REPO_ROOT / "build" / (
    "libblimp.dll" if os.name == "nt"
    else "libblimp.dylib" if sys.platform == "darwin"
    else "libblimp.so")


class BlimpFFIError(RuntimeError):
    """Raised when the compiled control core can't be loaded or used."""


# ---------------------------------------------------------------------------
# Wire structs — MUST mirror core/include/blimp/*.h exactly, field order and
# all. Every one of these crosses the blimp_ffi.c boundary (see that file's
# "EXACT EXPORTED FFI SIGNATURES" header comment). No bitfields, no unions,
# no varargs anywhere in the real headers, so plain ctypes.Structure with
# ctypes' OWN default (natural C) alignment is sufficient — none of these
# use `_pack_`, and none should ever need to.
# ---------------------------------------------------------------------------


class BlimpPidParams(C.Structure):
    """Mirrors blimp/params.h: blimp_pid_params_t. 8 floats, 32 bytes, no
    internal padding — every field is naturally 4-byte aligned already."""

    _fields_ = [
        ("kp", C.c_float),
        ("ki", C.c_float),
        ("kd", C.c_float),
        ("i_min", C.c_float),
        ("i_max", C.c_float),
        ("out_min", C.c_float),
        ("out_max", C.c_float),
        ("d_lpf_hz", C.c_float),
    ]


class BlimpParams(C.Structure):
    """Mirrors blimp/params.h: blimp_params_t.

    NOTE: `use_mag` (a single bool) sits between two float fields
    (mag_declination_rad, gyro_bias_tau_s). The compiler inserts 3 bytes of
    padding after it so gyro_bias_tau_s stays 4-byte aligned — verified by
    the layout probe (use_mag@136, gyro_bias_tau_s@140). ctypes reproduces
    this automatically under its default alignment; nothing special needed
    here, but don't reorder these fields without re-running the probe."""

    _fields_ = [
        ("front_half_span_m", C.c_float),
        ("cam_dz_m", C.c_float),
        ("max_total_thrust_n", C.c_float),
        ("motor_max_thrust_n", C.c_float),
        ("heave_budget_n", C.c_float),
        ("hover_thrust_n", C.c_float),
        ("alt", BlimpPidParams),
        ("yaw", BlimpPidParams),
        ("surge", BlimpPidParams),
        ("pitch_damp_kd", C.c_float),
        ("comp_alpha_attitude", C.c_float),
        ("comp_alpha_alt", C.c_float),
        ("mag_declination_rad", C.c_float),
        ("use_mag", C.c_bool),
        ("gyro_bias_tau_s", C.c_float),
        ("wp_touch_radius_m", C.c_float),
        ("wp_approach_speed_ms", C.c_float),
        ("wp_align_tol_rad", C.c_float),
        ("touch_confirm_s", C.c_float),
        ("search_yaw_rate", C.c_float),
        ("miss_timeout_s", C.c_float),
        ("prepare_settle_s", C.c_float),
        ("link_timeout_s", C.c_float),
        ("max_altitude_m", C.c_float),
        ("front_deadband_n", C.c_float),    # core 0.2.0: the mixer's deadband, appended
    ]


class BlimpState(C.Structure):
    """Mirrors blimp/types.h: blimp_state_t. Tail-padded to 56 bytes (the two
    trailing bools end the struct at byte 54; alignment-4 rounds to 56)."""

    _fields_ = [
        ("roll", C.c_float),
        ("pitch", C.c_float),
        ("yaw", C.c_float),
        ("gyro", C.c_float * 3),
        ("accel", C.c_float * 3),
        ("altitude", C.c_float),
        ("vz", C.c_float),
        ("yaw_rate", C.c_float),
        ("t_us", C.c_uint32),
        ("attitude_valid", C.c_bool),
        ("altitude_valid", C.c_bool),
    ]


class BlimpImuSample(C.Structure):
    """Mirrors blimp/types.h: blimp_imu_sample_t. Tail-padded to 56 bytes."""

    _fields_ = [
        ("gyro", C.c_float * 3),
        ("accel", C.c_float * 3),
        ("mag", C.c_float * 3),
        ("baro_alt", C.c_float),
        ("range_alt", C.c_float),
        ("t_us", C.c_uint32),
        ("gyro_valid", C.c_bool),
        ("accel_valid", C.c_bool),
        ("mag_valid", C.c_bool),
        ("baro_valid", C.c_bool),
        ("range_valid", C.c_bool),
    ]


class BlimpWrench(C.Structure):
    """Mirrors blimp/types.h: blimp_wrench_t.

    CONTRACT CHANGE (see task brief): this struct now has FOUR components,
    fx/fy/fz/tz — fy is the lateral trim thruster channel added to correct
    cross-track error that a bare bicopter structurally cannot null out
    (docs/05 sec 3.6). 4 floats, 16 bytes, no padding."""

    _fields_ = [
        ("fx", C.c_float),
        ("fy", C.c_float),
        ("fz", C.c_float),
        ("tz", C.c_float),
    ]


class BlimpActuators(C.Structure):
    """Mirrors blimp/types.h: blimp_actuators_t - four fixed unidirectional
    motors (front pair pushes +x, rear pair pushes +z) plus three clamp
    flags. Tail-padded to 20 bytes."""

    _fields_ = [
        ("thrust_fl", C.c_float),
        ("thrust_fr", C.c_float),
        ("thrust_rl", C.c_float),
        ("thrust_rr", C.c_float),
        ("clamped_total", C.c_bool),
        ("clamped_yaw", C.c_bool),
        ("clamped_heave", C.c_bool),
    ]


class BlimpWaypoint(C.Structure):
    """Mirrors blimp/types.h: blimp_waypoint_t. NOT part of any blimp_ffi.c
    signature — the core never receives waypoint (x, y, z) directly, only
    bearing/elevation/range via blimp_target_obs_t (see guidance.h's own
    header comment on why: deliberately not a fused position). Provided here
    only as a convenience value type for sim-side course bookkeeping (e.g.
    run_course.py's waypoint table), so it never needs to cross the FFI
    boundary and its layout correctness is not safety-critical."""

    _fields_ = [
        ("x", C.c_float),
        ("y", C.c_float),
        ("z", C.c_float),
    ]


class BlimpTargetObs(C.Structure):
    """Mirrors blimp/guidance.h: blimp_target_obs_t.

    NOTE the 3-byte gap: `visible` (a bool at offset 16) is followed by
    `t_us` (a uint32_t, needing 4-byte alignment), so the compiler inserts 3
    bytes of INTERNAL padding before t_us at offset 20 — this one is not just
    tail padding. Verified by the layout probe (visible@16, t_us@20)."""

    _fields_ = [
        ("bearing_rad", C.c_float),
        ("elevation_rad", C.c_float),
        ("range_m", C.c_float),
        ("confidence", C.c_float),
        ("hint_yaw_rad", C.c_float),
        ("alt_fix_m", C.c_float),
        ("visible", C.c_bool),
        ("hint_valid", C.c_bool),
        ("alt_fix_valid", C.c_bool),
        ("t_us", C.c_uint32),
    ]


# Display-only mirror of guidance.h's blimp_guid_state_t enum order. NOT used
# for any control decision in this file or sim_main.py — the real state
# machine, gating and transitions all live in the compiled C core; this dict
# only turns the int blimp_guidance_t.state back into a human-readable name
# for print()/logging, since blimp_guidance_state_name() itself is a plain C
# function (not exported through blimp_ffi.c) and so is not callable from
# Python. If guidance.h's enum order ever changes, update this table to
# match; _selftest() cannot catch that (it isn't part of the wire layout).
GUID_STATE_NAMES = {
    0: "IDLE",
    1: "PREPARE",
    2: "SEARCH",
    3: "ALIGN",
    4: "APPROACH",
    5: "TOUCH",
    6: "CONFIRM",
    7: "COAST",
    8: "LOOP_DONE",
    9: "FINISHED",
    10: "FAILSAFE",
}


class BlimpGuidance(C.Structure):
    """Mirrors blimp/guidance.h: blimp_guidance_t. Tail-padded to 48 bytes."""

    _fields_ = [
        ("state", C.c_int),  # blimp_guid_state_t; see GUID_STATE_NAMES above
        ("wp_index", C.c_int),
        ("wp_count", C.c_int),
        ("loop_index", C.c_int),
        ("loop_count", C.c_int),
        ("state_entry_us", C.c_uint32),
        ("run_start_us", C.c_uint32),
        ("last_seen_us", C.c_uint32),
        ("confirm_timer_s", C.c_float),
        ("search_dir", C.c_float),
        ("miss_count", C.c_int),
        ("timing_active", C.c_bool),
        # Added to blimp_guidance_t AFTER this file was first written. Rule 5
        # scores the BEST of up to three loops, so the machine has to retain
        # it; elapsed_s() alone cannot, because run_start_us is reused.
        ("last_loop_time_s", C.c_float),
        ("best_loop_time_s", C.c_float),
        ("hint_reached", C.c_bool),
        ("blind_climb_s", C.c_float),
        ("alt_err_known", C.c_bool),
        ("last_alt_err", C.c_float),
    ]


class BlimpHold(C.Structure):
    """Mirrors blimp/blimp.h: blimp_hold_t — the bench hold's setpoints, in
    place of guidance's (docs/pid-tuning-plan.md C1-C4). Four bools (the
    fourth, target_held, in what was padding until core 0.3.0), three
    floats: 16 bytes. All-zero is a normal run."""

    _fields_ = [
        ("active", C.c_bool),
        ("heading_held", C.c_bool),
        ("altitude_held", C.c_bool),
        ("target_held", C.c_bool),
        ("heading_rad", C.c_float),
        ("altitude_m", C.c_float),
        ("surge_n", C.c_float),
    ]


class BlimpInput(C.Structure):
    """Mirrors blimp/blimp.h: blimp_input_t — what one blimp_step() tick
    consumes. Nested structs (imu, target) contribute their OWN tail padding
    before contact_detected/arm_request/t_us start (imu@0, target@56,
    contact_detected@88, arm_request@89, t_us@92); the bench hold follows at
    96 (core 0.2.0)."""

    _fields_ = [
        ("imu", BlimpImuSample),
        ("target", BlimpTargetObs),
        ("contact_detected", C.c_bool),
        ("arm_request", C.c_bool),
        ("t_us", C.c_uint32),
        ("hold", BlimpHold),
    ]


# ---------------------------------------------------------------------------
# Frozen layout contract, captured from the real compiler (see module
# docstring). _selftest() checks every one of these at import time.
# ---------------------------------------------------------------------------
_EXPECTED_LAYOUT = {
    "BlimpPidParams": (32, {"kp": 0, "ki": 4, "kd": 8, "i_min": 12, "i_max": 16, "out_min": 20, "out_max": 24, "d_lpf_hz": 28}),
    "BlimpParams": (184, {"front_half_span_m": 0, "cam_dz_m": 4, "max_total_thrust_n": 8, "motor_max_thrust_n": 12, "heave_budget_n": 16, "hover_thrust_n": 20, "alt": 24, "yaw": 56, "surge": 88, "pitch_damp_kd": 120, "comp_alpha_attitude": 124, "comp_alpha_alt": 128, "mag_declination_rad": 132, "use_mag": 136, "gyro_bias_tau_s": 140, "wp_touch_radius_m": 144, "wp_approach_speed_ms": 148, "wp_align_tol_rad": 152, "touch_confirm_s": 156, "search_yaw_rate": 160, "miss_timeout_s": 164, "prepare_settle_s": 168, "link_timeout_s": 172, "max_altitude_m": 176, "front_deadband_n": 180}),
    "BlimpState": (56, {"roll": 0, "pitch": 4, "yaw": 8, "gyro": 12, "accel": 24, "altitude": 36, "vz": 40, "yaw_rate": 44, "t_us": 48, "attitude_valid": 52, "altitude_valid": 53}),
    "BlimpImuSample": (56, {"gyro": 0, "accel": 12, "mag": 24, "baro_alt": 36, "range_alt": 40, "t_us": 44, "gyro_valid": 48, "accel_valid": 49, "mag_valid": 50, "baro_valid": 51, "range_valid": 52}),
    "BlimpWrench": (16, {"fx": 0, "fy": 4, "fz": 8, "tz": 12}),
    "BlimpActuators": (20, {"thrust_fl": 0, "thrust_fr": 4, "thrust_rl": 8, "thrust_rr": 12, "clamped_total": 16, "clamped_yaw": 17, "clamped_heave": 18}),
    "BlimpWaypoint": (12, {"x": 0, "y": 4, "z": 8}),
    "BlimpTargetObs": (32, {"bearing_rad": 0, "elevation_rad": 4, "range_m": 8, "confidence": 12, "hint_yaw_rad": 16, "alt_fix_m": 20, "visible": 24, "hint_valid": 25, "alt_fix_valid": 26, "t_us": 28}),
    "BlimpGuidance": (72, {"state": 0, "wp_index": 4, "wp_count": 8, "loop_index": 12, "loop_count": 16, "state_entry_us": 20, "run_start_us": 24, "last_seen_us": 28, "confirm_timer_s": 32, "search_dir": 36, "miss_count": 40, "timing_active": 44, "last_loop_time_s": 48, "best_loop_time_s": 52, "hint_reached": 56, "blind_climb_s": 60, "alt_err_known": 64, "last_alt_err": 68}),
    "BlimpHold": (16, {"active": 0, "heading_held": 1, "altitude_held": 2, "target_held": 3, "heading_rad": 4, "altitude_m": 8, "surge_n": 12}),
    "BlimpInput": (112, {"imu": 0, "target": 56, "contact_detected": 88, "arm_request": 89, "t_us": 92, "hold": 96}),
}


def _selftest() -> None:
    """Recomputes ctypes.sizeof()/offsetof() for every mirrored struct above
    and compares against _EXPECTED_LAYOUT (captured from the real clang
    compile — see module docstring). Raises BlimpFFIError immediately on any
    mismatch, before a single byte is ever handed to the DLL. This is the
    guard against a future header edit silently desyncing this file."""
    by_name = {
        "BlimpPidParams": BlimpPidParams, "BlimpParams": BlimpParams,
        "BlimpState": BlimpState, "BlimpImuSample": BlimpImuSample,
        "BlimpWrench": BlimpWrench, "BlimpActuators": BlimpActuators,
        "BlimpWaypoint": BlimpWaypoint, "BlimpTargetObs": BlimpTargetObs,
        "BlimpGuidance": BlimpGuidance, "BlimpInput": BlimpInput,
        "BlimpHold": BlimpHold,
    }
    problems = []
    for name, (exp_size, exp_offsets) in _EXPECTED_LAYOUT.items():
        cls = by_name[name]
        got_size = C.sizeof(cls)
        if got_size != exp_size:
            problems.append(f"{name}: ctypes size={got_size}, expected {exp_size}")
        for field, exp_off in exp_offsets.items():
            got_off = getattr(cls, field).offset
            if got_off != exp_off:
                problems.append(
                    f"{name}.{field}: ctypes offset={got_off}, expected {exp_off}"
                )
    # ---- AUTHORITATIVE CROSS-LANGUAGE CHECK --------------------------------
    # Everything above compares this file against a table captured from the
    # headers when it was written - i.e. Python against Python. That can never
    # detect a C-side change, and indeed it kept passing after
    # blimp_guidance_t grew two fields (48 -> 56 bytes) while the sim quietly
    # read guidance state at the wrong offsets.
    #
    # This asks the COMPILED LIBRARY what it actually laid out. If a header
    # changes and this file is not updated, the import fails loudly instead of
    # producing a plausible-looking but wrong simulation.
    try:
        _lib_for_layout = _load_library()
        _lib_for_layout.blimp_ffi_struct_size.argtypes = [C.c_int]
        _lib_for_layout.blimp_ffi_struct_size.restype = C.c_size_t
    except Exception:            # noqa: BLE001 - DLL not built yet is not fatal here
        _lib_for_layout = None

    if _lib_for_layout is not None:
        _C_ORDER = [
            ("BlimpPidParams", 0), ("BlimpParams", 1), ("BlimpState", 2),
            ("BlimpImuSample", 3), ("BlimpWrench", 4), ("BlimpActuators", 5),
            ("BlimpWaypoint", 6), ("BlimpTargetObs", 7), ("BlimpGuidance", 8),
            ("BlimpInput", 9), ("BlimpHold", 11),
        ]
        for name, cid in _C_ORDER:
            c_size = int(_lib_for_layout.blimp_ffi_struct_size(cid))
            py_size = C.sizeof(by_name[name])
            if c_size and c_size != py_size:
                problems.append(
                    f"{name}: ctypes size={py_size} but the COMPILED C library "
                    f"says {c_size} - a header changed and this mirror is stale"
                )

    if problems:
        raise BlimpFFIError(
            "blimp_ffi.py's ctypes struct mirrors have drifted from the "
            "compiled C layout this file's docstring captured:\n  "
            + "\n  ".join(problems)
            + "\nRe-run the layout probe described in this module's "
              "docstring against the current headers and update both the "
              "_fields_ lists and _EXPECTED_LAYOUT above."
        )


_selftest()


# ---------------------------------------------------------------------------
# The wrapper
# ---------------------------------------------------------------------------
class BlimpController:
    """Owns one blimp_t instance (as an opaque byte buffer) and the loaded
    libblimp.dll, and exposes the exact blimp_ffi.c surface as plain Python
    methods. THIS CLASS CONTAINS NO CONTROL LOGIC OF ITS OWN — every method
    is a direct, thin call into the real compiled C core."""

    def __init__(self, dll_path: Optional[Path] = None):
        path = Path(dll_path) if dll_path is not None else DEFAULT_DLL_PATH
        if not path.exists():
            raise BlimpFFIError(
                f"{path.name} not found at {path}.\n"
                "Build it first:\n"
                "    cd " + str(REPO_ROOT) + " && python tools/build.py dll\n"
                "(That step compiles core/src/*.c, including core/src/blimp_ffi.c, "
                "into this DLL -- it must succeed before any simulation can run "
                "the REAL control core.)"
            )
        try:
            self._lib = C.CDLL(str(path))
        except OSError as e:
            raise BlimpFFIError(f"failed to load {path}: {e}") from e

        lib = self._lib
        lib.blimp_ffi_sizeof.argtypes = []
        lib.blimp_ffi_sizeof.restype = C.c_size_t

        lib.blimp_ffi_init.argtypes = [C.c_void_p]
        lib.blimp_ffi_init.restype = None

        lib.blimp_ffi_step.argtypes = [C.c_void_p, C.POINTER(BlimpInput),
                                        C.POINTER(BlimpActuators)]
        lib.blimp_ffi_step.restype = None

        lib.blimp_ffi_notify_link.argtypes = [C.c_void_p, C.c_uint32]
        lib.blimp_ffi_notify_link.restype = None

        lib.blimp_ffi_version.argtypes = []
        lib.blimp_ffi_version.restype = C.c_char_p

        lib.blimp_ffi_set_params.argtypes = [C.c_void_p, C.POINTER(BlimpParams)]
        lib.blimp_ffi_set_params.restype = C.c_int

        lib.blimp_ffi_get_params.argtypes = [C.c_void_p, C.POINTER(BlimpParams)]
        lib.blimp_ffi_get_params.restype = None

        lib.blimp_ffi_get_state.argtypes = [C.c_void_p, C.POINTER(BlimpState)]
        lib.blimp_ffi_get_state.restype = None

        lib.blimp_ffi_get_guidance.argtypes = [C.c_void_p, C.POINTER(BlimpGuidance)]
        lib.blimp_ffi_get_guidance.restype = None

        lib.blimp_ffi_get_demand.argtypes = [C.c_void_p, C.POINTER(BlimpWrench)]
        lib.blimp_ffi_get_demand.restype = None

        lib.blimp_ffi_get_actuators.argtypes = [C.c_void_p, C.POINTER(BlimpActuators)]
        lib.blimp_ffi_get_actuators.restype = None

        lib.blimp_ffi_get_armed.argtypes = [C.c_void_p]
        lib.blimp_ffi_get_armed.restype = C.c_int

        n = lib.blimp_ffi_sizeof()
        if n <= 0:
            raise BlimpFFIError(f"blimp_ffi_sizeof() returned {n}, refusing to allocate")
        self._buf = C.create_string_buffer(n)
        self._initialised = False

    # -- lifecycle ----------------------------------------------------------
    def init(self) -> None:
        """Wraps blimp_init(). Call once before any step()."""
        self._lib.blimp_ffi_init(self._buf)
        self._initialised = True

    def version(self) -> str:
        v = self._lib.blimp_ffi_version()
        return v.decode("ascii", errors="replace") if v else ""

    # -- the control tick -----------------------------------------------------
    def step(self, inp: BlimpInput) -> BlimpActuators:
        """Wraps blimp_step(). `inp` is a fully-populated BlimpInput; returns
        the actuator command the real core computed for it. Always returns a
        valid (if all-zero-thrust, on internal fault) BlimpActuators, per
        blimp_step()'s own contract."""
        if not self._initialised:
            raise BlimpFFIError("step() called before init()")
        out = BlimpActuators()
        self._lib.blimp_ffi_step(self._buf, C.byref(inp), C.byref(out))
        return out

    def notify_link(self, t_us: int) -> None:
        """Wraps blimp_notify_link(). Call once per valid 'ground packet' —
        in sim, once per tick that the guidance/ground-link is considered
        healthy, so the failsafe watchdog in blimp_step() sees a live link."""
        self._lib.blimp_ffi_notify_link(self._buf, C.c_uint32(t_us))

    # -- params ---------------------------------------------------------------
    def set_params(self, params: BlimpParams) -> bool:
        """Returns True if params was already rulebook-valid as given, False
        if blimp_params_validate() had to clamp something (read back with
        get_params() to see what actually took effect)."""
        rc = self._lib.blimp_ffi_set_params(self._buf, C.byref(params))
        return rc != 0

    def get_params(self) -> BlimpParams:
        out = BlimpParams()
        self._lib.blimp_ffi_get_params(self._buf, C.byref(out))
        return out

    # -- telemetry / introspection ---------------------------------------------
    def get_state(self) -> BlimpState:
        out = BlimpState()
        self._lib.blimp_ffi_get_state(self._buf, C.byref(out))
        return out

    def get_guidance(self) -> BlimpGuidance:
        out = BlimpGuidance()
        self._lib.blimp_ffi_get_guidance(self._buf, C.byref(out))
        return out

    def get_demand(self) -> BlimpWrench:
        out = BlimpWrench()
        self._lib.blimp_ffi_get_demand(self._buf, C.byref(out))
        return out

    def get_actuators(self) -> BlimpActuators:
        out = BlimpActuators()
        self._lib.blimp_ffi_get_actuators(self._buf, C.byref(out))
        return out

    def get_armed(self) -> bool:
        return self._lib.blimp_ffi_get_armed(self._buf) != 0


def make_imu_sample(t_us: int, gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 0.0),
                     mag=(0.0, 0.0, 0.0), baro_alt: float = 0.0,
                     range_alt: float = 0.0, gyro_valid: bool = True,
                     accel_valid: bool = True, mag_valid: bool = True,
                     baro_valid: bool = True, range_valid: bool = True) -> BlimpImuSample:
    """Convenience constructor — purely a struct-filling helper, no physics
    or noise modelling of any kind (that lives in sim_main.py, deliberately,
    since it must be easy to see and change without touching the FFI file)."""
    s = BlimpImuSample()
    s.gyro[0], s.gyro[1], s.gyro[2] = gyro
    s.accel[0], s.accel[1], s.accel[2] = accel
    s.mag[0], s.mag[1], s.mag[2] = mag
    s.baro_alt = baro_alt
    s.range_alt = range_alt
    s.t_us = t_us & 0xFFFFFFFF
    s.gyro_valid = gyro_valid
    s.accel_valid = accel_valid
    s.mag_valid = mag_valid
    s.baro_valid = baro_valid
    s.range_valid = range_valid
    return s


def make_target_obs(t_us: int, bearing_rad: float = 0.0, elevation_rad: float = 0.0,
                     range_m: float = 0.0, confidence: float = 0.0,
                     visible: bool = False, hint_yaw_rad: float = 0.0,
                     hint_valid: bool = False, alt_fix_m: float = 0.0,
                     alt_fix_valid: bool = False) -> BlimpTargetObs:
    o = BlimpTargetObs()
    o.bearing_rad = bearing_rad
    o.elevation_rad = elevation_rad
    o.range_m = range_m
    o.confidence = confidence
    o.hint_yaw_rad = hint_yaw_rad
    o.alt_fix_m = alt_fix_m
    o.visible = visible
    o.hint_valid = hint_valid
    o.alt_fix_valid = alt_fix_valid
    o.t_us = t_us & 0xFFFFFFFF
    return o


def make_input(imu: BlimpImuSample, target: BlimpTargetObs, t_us: int,
               contact_detected: bool = False, arm_request: bool = True) -> BlimpInput:
    inp = BlimpInput()
    inp.imu = imu
    inp.target = target
    inp.contact_detected = contact_detected
    inp.arm_request = arm_request
    inp.t_us = t_us & 0xFFFFFFFF
    return inp


if __name__ == "__main__":
    # Struct-layout self-test already ran at import time (above). This block
    # additionally tries to load the real DLL and run the smallest possible
    # smoke sequence, purely as a convenience for `python sim/blimp_ffi.py`.
    print("blimp_ffi.py: struct layout self-test PASSED "
          f"({len(_EXPECTED_LAYOUT)} structs checked against the real compiler)")
    try:
        ctl = BlimpController()
    except BlimpFFIError as e:
        print(f"\nDLL smoke test SKIPPED: {e}")
        raise SystemExit(1)
    ctl.init()
    print(f"loaded {DEFAULT_DLL_PATH}, blimp_version() = {ctl.version()!r}")
    imu = make_imu_sample(t_us=1000)
    target = make_target_obs(t_us=1000)
    inp = make_input(imu, target, t_us=1000)
    act = ctl.step(inp)
    print(f"first step(): fl={act.thrust_fl:.4f} fr={act.thrust_fr:.4f} "
          f"rl={act.thrust_rl:.4f} rr={act.thrust_rr:.4f} armed={ctl.get_armed()}")
    print("DLL smoke test PASSED")
