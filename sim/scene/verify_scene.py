#!/usr/bin/env python3
"""verify_scene.py -- headless physics sanity checks for the generated IAR-LARC scene.

No viewer, no blocking calls. Loads the MJCF that build_scene.py produced,
steps it for a few seconds, and asserts:

  1. The model loads at all.
  2. Each waypoint balloon settles with its centre at balloon_height +/- tol.
  3. The four waypoints really form a wp_rect_length x wp_rect_width rectangle.
  4. The vehicle's buoyancy matches its declared trim: the scene records
     `trim_heavy_gf` (<custom><text>). If it is 0 the vehicle must drift
     less than `--drift-tol` m vertically over 10 s with zero actuation; if
     it is > 0 the vehicle must SINK (motors off = gentle descent to the
     floor, the intended failsafe) and come to rest on the floor, not
     through it.
  5. Nothing in qpos/qvel/qacc goes NaN or Inf at any point during the runs.

Usage:
    python verify_scene.py                  # verifies sim/scene/course1.xml (built with defaults)
    python verify_scene.py --xml course2.xml --tol 0.08
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

try:
    import mujoco
except ImportError as e:  # pragma: no cover
    print(f"FAIL: mujoco is not importable: {e}")
    raise SystemExit(1)


def assert_finite(data: "mujoco.MjData", where: str) -> None:
    for name, arr in (("qpos", data.qpos), ("qvel", data.qvel), ("qacc", data.qacc)):
        if not np.all(np.isfinite(arr)):
            raise AssertionError(f"NaN/Inf detected in {name} at {where}")


def find_body(model: "mujoco.MjModel", name: str) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise AssertionError(f"body '{name}' not found in model")
    return bid


def read_custom_float(model: "mujoco.MjModel", name: str, default: float) -> float:
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXT, name)
    if tid < 0:
        return default
    adr, size = int(model.text_adr[tid]), int(model.text_size[tid])
    raw = bytes(model.text_data[adr:adr + size]).decode("ascii", "ignore").rstrip("\x00")
    try:
        return float(raw)
    except ValueError:
        return default


def find_geom(model: "mujoco.MjModel", name: str) -> int:
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if gid < 0:
        raise AssertionError(f"geom '{name}' not found in model")
    return gid


def run(xml_path: str, height_tol: float, rect_tol: float, drift_tol: float,
        wp_length: float, wp_width: float, balloon_height: float,
        settle_time: float, total_time: float) -> None:
    print(f"Loading {xml_path} ...")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    print(f"  PASS: model loaded ({model.nbody} bodies, {model.ngeom} geoms, "
          f"{model.njnt} joints, {model.nu} actuators)")

    # Use the balloon SPHERE GEOM's world position, not the balloon body's
    # origin -- the body origin is the tether attachment point (the
    # balloon's bottom), offset by balloon_radius below the sphere centre.
    balloon_geom_ids = [find_geom(model, f"balloon_{i}_geom") for i in range(1, 5)]
    vehicle_id = find_body(model, "vehicle")

    # Zero all controls explicitly -- "zero actuation" for the buoyancy check,
    # and balloons/tethers have no actuators at all so this only affects the
    # vehicle's (inactive) rotors/tilt/trim actuators.
    data.ctrl[:] = 0.0

    mujoco.mj_forward(model, data)
    assert_finite(data, "t=0 (initial forward)")
    vehicle_z0 = float(data.xpos[vehicle_id, 2])

    n_settle = int(round(settle_time / model.opt.timestep))
    n_total = int(round(total_time / model.opt.timestep))
    if n_total < n_settle:
        n_total = n_settle

    for step in range(n_total):
        mujoco.mj_step(model, data)
        # Cheap, frequent NaN guard -- catches an exploding sim as early as
        # possible rather than only at the end.
        if step % 50 == 0 or step == n_total - 1:
            assert_finite(data, f"t={data.time:.3f}s (step {step})")

        if step == n_settle - 1:
            balloon_xyz = {i + 1: np.array(data.geom_xpos[gid]) for i, gid in enumerate(balloon_geom_ids)}

    print(f"  PASS: simulated {total_time:.1f} s ({n_total} steps) with no NaN/Inf")

    # --- Check 2: balloon centre height ---------------------------------
    for wp, xyz in balloon_xyz.items():
        z = xyz[2]
        ok = abs(z - balloon_height) <= height_tol
        status = "PASS" if ok else "FAIL"
        print(f"  {status}: waypoint {wp} balloon centre z={z:.4f} m "
              f"(target {balloon_height} +/- {height_tol} m)")
        assert ok, f"waypoint {wp} balloon centre height out of tolerance: {z:.4f} m"

    # --- Check 3: rectangle dimensions ------------------------------------
    xs = [balloon_xyz[i][0] for i in (1, 2, 3, 4)]
    ys = [balloon_xyz[i][1] for i in (1, 2, 3, 4)]
    measured_length = max(xs) - min(xs)
    measured_width = max(ys) - min(ys)
    len_ok = abs(measured_length - wp_length) <= rect_tol
    wid_ok = abs(measured_width - wp_width) <= rect_tol
    print(f"  {'PASS' if len_ok else 'FAIL'}: waypoint rectangle length (x-extent) = "
          f"{measured_length:.4f} m (target {wp_length} +/- {rect_tol} m)")
    print(f"  {'PASS' if wid_ok else 'FAIL'}: waypoint rectangle width (y-extent) = "
          f"{measured_width:.4f} m (target {wp_width} +/- {rect_tol} m)")
    assert len_ok, f"waypoint rectangle length wrong: {measured_length:.4f} m"
    assert wid_ok, f"waypoint rectangle width wrong: {measured_width:.4f} m"

    # Sanity: exactly 4 distinct corners (not degenerate/collinear)
    corners = sorted(set((round(x, 2), round(y, 2)) for x, y in zip(xs, ys)))
    assert len(corners) == 4, f"expected 4 distinct waypoint corners, got {corners}"
    print(f"  PASS: 4 distinct waypoint corners at {corners}")

    # --- Check 4: vehicle buoyancy matches the declared trim, zero actuation
    vehicle_zT = float(data.xpos[vehicle_id, 2])
    drift = vehicle_zT - vehicle_z0
    trim_gf = read_custom_float(model, "trim_heavy_gf", 0.0)
    env_gid = find_geom(model, "envelope")
    r_env = float(model.geom_size[env_gid, 0])
    if trim_gf <= 0.0:
        drift_ok = abs(drift) <= drift_tol
        print(f"  {'PASS' if drift_ok else 'FAIL'}: vehicle vertical drift over {total_time:.1f} s "
              f"= {drift * 100.0:+.2f} cm (z0={vehicle_z0:.4f} m -> zT={vehicle_zT:.4f} m, "
              f"tolerance +/-{drift_tol * 100:.1f} cm)")
        assert drift_ok, f"vehicle is not neutrally buoyant: drifted {drift*100:.2f} cm"
    else:
        sank = drift < -drift_tol
        on_floor = vehicle_zT >= r_env - 0.05
        print(f"  {'PASS' if sank else 'FAIL'}: vehicle trimmed {trim_gf:.1f} gf heavy sank "
              f"{-drift:.3f} m in {total_time:.1f} s with motors off "
              f"(z0={vehicle_z0:.3f} -> zT={vehicle_zT:.3f} m)")
        assert sank, f"vehicle declared {trim_gf} gf heavy but did not sink (drift {drift:+.3f} m)"
        print(f"  {'PASS' if on_floor else 'FAIL'}: vehicle rests on the floor "
              f"(centre z={vehicle_zT:.3f} m, envelope radius {r_env:.3f} m)")
        assert on_floor, "vehicle fell through the floor"

    print("\nALL CHECKS PASSED")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--xml", type=str, default=os.path.join(here, "course1.xml"))
    ap.add_argument("--tol", type=float, default=0.05, help="balloon height tolerance, m")
    ap.add_argument("--rect-tol", type=float, default=0.05, help="rectangle dimension tolerance, m")
    ap.add_argument("--drift-tol", type=float, default=0.05, help="vehicle vertical drift tolerance, m")
    ap.add_argument("--wp-length", type=float, default=4.0)
    ap.add_argument("--wp-width", type=float, default=2.0)
    ap.add_argument("--balloon-height", type=float, default=2.0)
    ap.add_argument("--settle-time", type=float, default=3.0, help="s, before checking balloon rest position")
    ap.add_argument("--total-time", type=float, default=10.0, help="s, total sim time for the drift check")
    ns = ap.parse_args(argv if argv is not None else sys.argv[1:])

    try:
        run(ns.xml, ns.tol, ns.rect_tol, ns.drift_tol, ns.wp_length, ns.wp_width,
            ns.balloon_height, ns.settle_time, ns.total_time)
    except AssertionError as e:
        print(f"\nFAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
