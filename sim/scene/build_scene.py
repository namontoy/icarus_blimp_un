#!/usr/bin/env python3
"""build_scene.py -- parametric MuJoCo scene generator for the real IAR-LARC 2026 course.

Generates the MJCF (MuJoCo XML) for the actual competition arena, derived from
the rulebook (reference/IAR_LARC2026_rulebook.txt, rules 18-22), NOT from the
upstream mochi-swarm-sim `mochi_BlimpRace_balloon_course.xml` files, which are
a 9-colour, ~90 m slalom demo scene unrelated to this competition (see
docs/07-control-design.md Sec. 9/10 and docs/05-race-strategy.md Sec 3.6).

Everything geometric and physical is a function of the parameters below, so
re-running this script after the weight budget closes (e.g. once the
professor's PCB is weighed) regenerates a consistent scene rather than
requiring hand edits to a static XML.

Usage:
    python build_scene.py                       # course 1, all defaults
    python build_scene.py --course 2 --out x.xml
    python build_scene.py --envelope-diameter 0.69 --air-density 0.89

Run `python build_scene.py --help` for the full parameter list.

-------------------------------------------------------------------------
PHYSICS MODELLING NOTES (what is real, what is approximated) -- IMPORTANT:
this section was corrected after EMPIRICAL testing against the installed
MuJoCo 3.13.0 build, not just the documentation. The first draft of this
generator assumed (from reading MuJoCo's fluid-forces documentation, whose
formulas do include acceleration-dependent terms) that the ellipsoid fluid
model would supply both buoyancy AND added mass automatically. Two direct
tests against the actual engine (see the task report) showed this is wrong
in ways worth recording in case a future version of this file is revisited:

  1. NO BUOYANCY. Neither MuJoCo fluid model (plain density/viscosity, or
     `fluidshape="ellipsoid"`) applies a static, velocity-independent
     Archimedes force. Tested directly: a free sphere geom with
     `density`/`viscosity`/`fluidshape="ellipsoid"` set, at rest (qvel=0),
     shows `qfrc_passive == 0` and falls at exactly -g -- there is no
     hydrostatic lift. (The MuJoCo docs never use the words "buoyant" or
     "Archimedes" for either fluid model; this generator's first draft had
     assumed otherwise from the formulas alone, without checking.) Buoyancy
     here is instead implemented with the `<body gravcomp="...">` attribute,
     which applies exactly `gravcomp * mass * |gravity|` upward, verified
     empirically to exactly cancel gravity at gravcomp=1.0 regardless of the
     mass value used (confirmed: qacc==0 to float precision). `gravcomp` is
     computed per body from real physics (rho_air * displaced volume /
     body mass), not hand-picked -- see `gravcomp_for()` below.
  2. ADDED MASS DOES NOT MEASURABLY APPEAR AS INERTIA. The ellipsoid model's
     documented formula (f_A = -m_A.vdot + ...) suggests an acceleration-
     reactive force. Tested directly: applying a constant external force to
     an at-rest ellipsoid-shaped sphere gives an INITIAL qacc exactly equal
     to F/m_rigid (not F/(m_rigid+m_A)) -- i.e. no measurable added-mass
     softening of the very first instant's acceleration, and the visible
     velocity-dependent falloff at later steps matches ordinary quadratic
     drag reaching a terminal velocity, not a persistent inertia increase.
     (Whether this is a lagged/one-step-behind implementation that never
     converges for a constant force, or something else internal to MuJoCo,
     was not resolved -- undocumented at the level needed, and out of scope
     to chase into the C source for this task.) Given that, THIS GENERATOR
     DOES NOT RELY ON MuJoCo's ellipsoid model for added mass. Instead, the
     vehicle's *effective* translational mass (m_eff = m*(1+k), k=0.5 for a
     sphere per docs/07 Sec 1.1) is used directly as the simulated body
     mass, so F=m*a in the engine already reflects the physically dominant
     added-mass effect docs/05 Sec 3.1 and docs/07 Sec 1.2 describe. Because
     `gravcomp` cancels gravity as a *ratio* of whatever mass is configured
     (see point 1), inflating the mass for inertia purposes does not disturb
     the neutral-buoyancy balance -- both scale together, verified by the
     `--drift-tol` check in verify_scene.py. Rotational inertia is
     deliberately NOT inflated: a sphere's own rotational added inertia is
     classically ~0 (Munk/NACA TR-184, cited in docs/07 Sec 1.2), so leaving
     `diaginertia` at its real (non-added-mass) value is the theoretically
     correct choice, not an oversight.
  3. What MuJoCo's ellipsoid model DOES reliably provide, confirmed
     empirically: velocity-dependent quadratic ("blunt") drag -- a constant
     applied force on an ellipsoid-shaped body was observed to asymptote to
     a finite terminal velocity rather than accelerate indefinitely, the
     signature of v^2 drag. This is genuinely used here (envelope and
     balloon geoms keep `fluidshape="ellipsoid"` for this reason), it is
     just not the buoyancy/added-mass source.

What is deliberately APPROXIMATED (beyond the above):
  - Only the envelope sphere (vehicle) and the balloon spheres get
    `fluidshape="ellipsoid"` (drag) and `gravcomp` (buoyancy). The gondola,
    motor housings and tether-cable capsules get neither -- they
    are geometrically small (each well under 1% of the envelope's volume)
    so the omitted drag/buoyancy from them is small, but it is a real,
    documented simplification.
  - Motor/gondola added mass is neglected (only the envelope's translational
    mass is inflated by (1+k)); they are small appendages.
  - The tether cable is modelled as a short chain of rigid capsules linked
    by 2-DOF universal joints (two orthogonal hinges per link), each with
    damping and a range limit, rather than a true continuum flexible rope.
    This gives bounded, damped, physically-reactive motion (Rule 21) without
    MuJoCo's more expensive native `<flex>`/composite rope machinery, which
    was judged unnecessary for a 4-6 segment tether at this scale.
  - Component mass breakdown for the vehicle (gondola/electronics vs.
    envelope film vs. pods) is a placeholder split, NOT a real BOM -- the
    professor's PCB mass is still unmeasured. Only the *aggregate* rigid
    vehicle mass is physically constrained (forced to rho_air * V_envelope
    for Rule 11, minus the deliberate trim_heavy_gf).
  - Air viscosity is set to a realistic constant (1.8e-5 Pa.s, ~sea-level
    air; Bogota's is within a few % of this and the effect is dominated by
    the quadratic term anyway per docs/07 Sec 1.3), not derived from ambient
    temperature.
  - The balloon's own added mass (relevant to how fast it swings back after
    being bumped) is NOT inflated -- only the vehicle's is, since that is
    what docs/05/07 flag as dominating lap-time dynamics.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

GF_TO_N = 9.81e-3  # 1 gram-force in newtons, g = 9.81 m/s^2 (matches docs/07's convention)
G = 9.81


# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------
@dataclass
class SceneParams:
    # Arena
    arena_length: float = 10.0          # m, along x (rulebook Rule 18/Fig. 3-4: "10 m")
    arena_width: float = 5.0            # m, along y ("5 m")

    # Waypoint rectangle (Rule 18/19/20)
    wp_rect_length: float = 4.0         # m, along x
    wp_rect_width: float = 2.0          # m, along y

    # Waypoint balloons (Rule 21, 22)
    balloon_height: float = 2.0         # m, ground to balloon CENTRE
    balloon_diameter: float = 0.55      # m, inflated diameter (rule: 50-60 cm)
    balloon_film_mass_kg: float = 0.015 # kg, 22-inch mylar envelope + valve
    helium_density: float = 0.123       # kg/m^3 at Bogota (~2640 m)
    tether_segments: int = 4            # number of capsule links in the tether chain
    tether_radius: float = 0.004        # m, cable radius (thin, "lightweight")
    tether_joint_range_deg: float = 20.0  # +/- deg per link -> bounded swing region
    tether_joint_damping: float = 0.0008  # N*m*s/rad per link axis
    balloon_mass_kg: float = 0.015      # kg, mylar film + helium fill (net-positive lift; tether reacts it)

    start_region_diameter: float = 2.0  # m (Rule: "2 m diameter start region")

    # Course selection: which visiting order is annotated in the scene.
    # Geometry (the 4 waypoint positions) is IDENTICAL for both courses --
    # only the announced visiting order differs (Rule 19 rectangle perimeter
    # vs Rule 20 figure-eight via the diagonals).
    course: int = 1                     # 1 = rectangle perimeter, 2 = figure-eight

    # Vehicle envelope
    envelope_diameter: float = 0.78     # m (-> ~0.25 m^3, this task's default)
    added_mass_coeff: float = 0.5       # k; 0.5 = sphere (Lamb). ~0.11 axial
                                        # for a 3:1 prolate ellipsoid.
    # Motors: four FIXED brushed motors, unidirectional (core/include/blimp/types.h).
    # Front pair pushes +x (surge, differential = yaw); rear pair pushes +z (heave).
    # All positions are relative to the envelope centre, in the body frame.
    front_half_span: float = 0.10       # m, front motors' lateral offset (l_f in params.c)
    front_x: float = 0.05               # m, front motors ahead of the envelope centre
    rear_half_span: float = 0.03        # m, rear (vertical) motors' lateral offset
    rear_x: float = -0.06               # m, rear motors behind the envelope centre
    motor_max_gf: float = 35.0          # gf, per-motor cap (PLACEHOLDER, = params.c)
    gondola_mass_kg: float = 0.17       # kg, everything that hangs under the envelope
                                        # (PCB+XIAO, battery, 4 motors, frame, camera):
                                        # PLACEHOLDER until the professor weighs it
    envelope_shell_inertia: float = 0.004  # kg m^2, film + part of the helium about
                                        # any axis (thin shell 2/3 m r^2 with ~30 g
                                        # rotating); NOT the inflated added mass -
                                        # a sphere's rotational added inertia is ~0
    envelope_ang_drag_coef: float = 0.05  # MuJoCo fluidcoef[2]. The default 1.5 is
                                        # for elongated bodies; a sphere has no pressure
                                        # drag moment (viscous only, ~1e-5 N m at 0.5
                                        # rad/s). With 1.5 the sim needed ~3 gf of
                                        # continuous differential thrust just to keep
                                        # turning, and that surge walked the vehicle
                                        # across the arena during every scan. 0.05 keeps
                                        # a little for the gondola/motor appendages.
    contact_timeconst: float = 0.06     # s, MuJoCo solref[0] on envelope/balloon
                                        # contacts. Default 0.02 makes a latex
                                        # envelope meeting a mylar balloon behave like
                                        # billiard balls (a 2 m/s vertical kick was
                                        # observed); both are soft, so slower/softer.
    camera_dz: float = -0.35            # m, camera height relative to the envelope
                                        # centre (= params.c cam_dz_m)
    spawn_facing_wp1: bool = True       # the team places the vehicle facing balloon 1
    trim_heavy_gf: float = 6.0          # gf, deliberate negative buoyancy. Rule 11
                                        # allows +/-10 gf; the rear pair carries this
                                        # in flight and the vehicle sinks gently on it
                                        # when the motors stop (= params.c hover_thrust)
    gondola_mass_fraction: float = 0.0  # reserved: fraction of TOTAL mass forced into
                                         # the gondola beyond the auto-balanced remainder;
                                         # kept at 0 (remainder-based) until the professor's
                                         # PCB mass is known -- see module docstring.

    # Environment
    air_density: float = 0.89           # kg/m^3, Bogota ~2640 m default (docs/07 Sec 1.2)
    air_viscosity: float = 1.8e-5       # Pa*s, approx. air at room temperature

    # Actuation budget (Rule 13: 100 gf total, enforced by the C mixer, not here)
    total_thrust_gf: float = 100.0

    # Simulation
    timestep: float = 0.002
    gravity: float = -9.81

    # Output
    out_path: str = ""                  # resolved to a default in main() if empty

    def envelope_volume(self) -> float:
        return (math.pi / 6.0) * self.envelope_diameter ** 3

    def envelope_radius(self) -> float:
        return self.envelope_diameter / 2.0

    def balloon_radius(self) -> float:
        return self.balloon_diameter / 2.0

    def displaced_air_mass(self) -> float:
        """Mass of the air the envelope displaces. Rule 11 forces the vehicle
        to be neutrally buoyant, so this is also its true RIGID mass."""
        return self.air_density * self.envelope_volume()

    def vehicle_total_mass(self) -> float:
        """The mass MuJoCo should simulate: rigid mass PLUS added mass.

        An accelerating body drags surrounding air with it. For a sphere the
        added mass is half the displaced air (k=0.5, docs/07 Sec 1.1), so the
        effective translational inertia is 1.5x the rigid mass - and on a
        neutrally buoyant vehicle that is a first-order effect, not a
        correction: it is a third of the total resistance to acceleration.

        BUG HISTORY: this module's docstring has always specified the (1+k)
        inflation, but the implementation returned the bare rigid mass, so the
        simulated vehicle accelerated 1.5x too easily. Gains tuned against
        that sim would have been ~50% too weak on real hardware - exactly the
        class of error a simulator is supposed to prevent.

        Neutral buoyancy is preserved regardless, because gravcomp cancels
        gravity as a RATIO of whatever mass is configured: inflating the mass
        scales the compensating force identically. verify_scene.py's drift
        check confirms this empirically.

        Rotational inertia is deliberately NOT inflated - a sphere's added
        moment of inertia is classically zero (Munk, NACA TR-184)."""
        return self.displaced_air_mass() * (1.0 + self.added_mass_coeff)

    def balloon_volume(self) -> float:
        return (math.pi / 6.0) * self.balloon_diameter ** 3

    def balloon_system_mass(self) -> float:
        """Envelope film plus the helium inside it. This is the mass MuJoCo
        must see, because the helium really is carried along by the balloon."""
        return self.balloon_film_mass_kg + self.helium_density * self.balloon_volume()

    def balloon_gravcomp(self) -> float:
        """Waypoint balloons are HELIUM FILLED (Rule 21) and tethered to the
        ground - they pull UP on their tether, they do not hang from it.

        MuJoCo applies `gravcomp * mass * g` upward, so setting gravcomp to
        (displaced air mass / body mass) reproduces true Archimedean lift:
            net_up = (m_displaced - m_system) * g
        For a 0.55 m balloon at Bogota density that is ~52 gf of lift, which
        is what holds the tether taut and the balloon at its 2 m station.

        Getting this wrong was the original scene bug: with no gravcomp the
        balloons were just 15 g weights on a floppy chain and sagged to
        0.44 m instead of standing at 2.0 m."""
        m_sys = self.balloon_system_mass()
        if m_sys <= 0.0:
            return 1.0
        return (self.air_density * self.balloon_volume()) / m_sys

    def vehicle_gravcomp(self) -> float:
        """MuJoCo applies gravcomp * mass * g upward. The vehicle is trimmed
        trim_heavy_gf HEAVY (Rule 11 tolerance), so the net downward force
        must be exactly trim * g regardless of the (1+k) mass inflation:
            (1 - gc) * M * g = m_trim * g   ->   gc = 1 - m_trim / M
        With gc = 1 the vehicle would be exactly neutral (the old scene)."""
        m_total = self.vehicle_total_mass()
        m_trim = self.trim_heavy_gf * 1e-3
        if m_trim >= m_total:
            raise ValueError(f"trim_heavy_gf={self.trim_heavy_gf} exceeds the vehicle mass")
        return 1.0 - m_trim / m_total

    def main_body_mass(self) -> float:
        """The whole vehicle is ONE rigid body now (fixed motors, no tilting
        pods), so the main body carries the entire mass. Kept as a function
        so the print-out in main() and any external caller keep working."""
        return self.vehicle_total_mass()


# --------------------------------------------------------------------------
# Waypoint geometry
# --------------------------------------------------------------------------
def waypoint_corners(p: SceneParams) -> List[Tuple[float, float]]:
    """Returns [(x,y), ...] for waypoints 1..4, matching the rulebook's Fig. 3
    layout (top row '4 3', bottom row '1 2'; 1=bottom-left .. going around the
    perimeter 1->2->3->4->1)."""
    hl = p.wp_rect_length / 2.0
    hw = p.wp_rect_width / 2.0
    return [
        (-hl, -hw),  # waypoint 1: bottom-left
        (+hl, -hw),  # waypoint 2: bottom-right
        (+hl, +hw),  # waypoint 3: top-right
        (-hl, +hw),  # waypoint 4: top-left
    ]


def course_order(p: SceneParams) -> List[int]:
    if p.course == 1:
        return [1, 2, 3, 4]          # Rule 19: rectangle perimeter
    elif p.course == 2:
        return [1, 3, 2, 4]          # Rule 20: figure-eight via the diagonals
    raise ValueError(f"course must be 1 or 2, got {p.course}")


# --------------------------------------------------------------------------
# XML assembly (plain string templates -- keeps the generated MJCF readable)
# --------------------------------------------------------------------------
def _tether_chain_xml(p: SceneParams, wp_index: int, x: float, y: float, indent: str) -> str:
    """Recursively nested capsule chain from the ground anchor (x,y,0) up to
    the balloon body, each link a 2-hinge 'universal joint' pair so the
    tether can bend (bounded, damped) rather than being a rigid post."""
    attach_z = p.balloon_height - p.balloon_radius()   # where the cable meets the balloon
    seg_len = attach_z / p.tether_segments
    rng = math.radians(p.tether_joint_range_deg)

    def link(depth: int, cur_indent: str) -> str:
        if depth == p.tether_segments:
            # Terminal: the balloon itself, attached at the TOP of the last
            # capsule link (pos = seg_len along that link's local z), with
            # its own small damped hinge pair so it can swing independently
            # of the last rigid link. The sphere geom (and touch site) are
            # offset up by balloon_radius from that attachment point, since
            # the cable attaches to the balloon's *bottom*, not its centre.
            return (
                f'{cur_indent}<body name="balloon_{wp_index}" pos="0 0 {seg_len:.6f}" '
                f'gravcomp="{p.balloon_gravcomp():.6f}">\n'
                f'{cur_indent}  <joint name="balloon_{wp_index}_swing_x" type="hinge" axis="1 0 0" '
                f'damping="{p.tether_joint_damping}" range="-{rng:.6f} {rng:.6f}"/>\n'
                f'{cur_indent}  <joint name="balloon_{wp_index}_swing_y" type="hinge" axis="0 1 0" '
                f'damping="{p.tether_joint_damping}" range="-{rng:.6f} {rng:.6f}"/>\n'
                f'{cur_indent}  <geom name="balloon_{wp_index}_geom" type="sphere" '
                f'pos="0 0 {p.balloon_radius():.6f}" '
                f'size="{p.balloon_radius():.6f}" mass="{p.balloon_system_mass():.6f}" '
                f'material="mat_balloon" fluidshape="ellipsoid" '
                f'fluidcoef="0.5 0.25 1.5 1.0 1.0" friction="0.6 0.005 0.0001" '
                f'solref="{p.contact_timeconst:.3f} 1"/>\n'
                f'{cur_indent}  <site name="waypoint_{wp_index}_touch" type="sphere" '
                f'pos="0 0 {p.balloon_radius():.6f}" '
                f'size="{p.balloon_radius() * 1.02:.6f}" rgba="1 1 1 0"/>\n'
                f'{cur_indent}</body>\n'
            )
        seg_name = f"tether_{wp_index}_link{depth}"
        child_pos = f"0 0 {seg_len:.6f}"
        body_pos = "0 0 0" if depth == 0 else f"0 0 {seg_len:.6f}"
        body = (
            f'{cur_indent}<body name="{seg_name}" pos="{body_pos}">\n'
            f'{cur_indent}  <joint name="{seg_name}_x" type="hinge" axis="1 0 0" '
            f'damping="{p.tether_joint_damping}" range="-{rng:.6f} {rng:.6f}"/>\n'
            f'{cur_indent}  <joint name="{seg_name}_y" type="hinge" axis="0 1 0" '
            f'damping="{p.tether_joint_damping}" range="-{rng:.6f} {rng:.6f}"/>\n'
            f'{cur_indent}  <geom name="{seg_name}_geom" type="capsule" '
            f'fromto="0 0 0 0 0 {seg_len:.6f}" size="{p.tether_radius:.6f}" '
            f'mass="0.0008" material="mat_tether" fluidshape="none"/>\n'
            f"{link(depth + 1, cur_indent + '  ')}"
            f'{cur_indent}</body>\n'
        )
        return body

    return link(0, indent).replace("pos=\"0 0 0\"\n", f'pos="0 0 0"\n', 1).replace(
        f'<body name="tether_{wp_index}_link0" pos="0 0 0">',
        f'<body name="tether_{wp_index}_link0" pos="{x:.6f} {y:.6f} 0">',
    )


def _vehicle_xml(p: SceneParams, start_xy: Tuple[float, float], yaw: float,
                 indent: str) -> str:
    """Emit the vehicle as TWO bodies:

      envelope  - the free body. Carries the film, the helium and the whole
                  (1+k) added-mass inflation as translational mass at the
                  sphere centre, with a SMALL explicit rotational inertia
                  (a sphere's rotational added inertia is classically ~0).
                  Its gravcomp supplies ALL the buoyancy, so lift acts at
                  the envelope centre - exactly where Archimedes puts it.
      gondola   - a fixed (jointless) child body carrying the real hanging
                  mass with gravcomp 0. Buoyancy above, weight below: this
                  is what gives the real vehicle its pendulum stiffness in
                  roll and pitch, which a single body with gravcomp at its
                  own centre of mass completely lacks (the previous scene's
                  vehicle had no attitude stiffness at all).

    MuJoCo applies gravcomp * mass * g at each body's centre of mass, so
    with gravity on both bodies (M_env + m_gond) g and the envelope
    compensating (M_env + m_gond - m_trim) g, the net is m_trim g downward.

    The gondola has four legs so the vehicle rests UPRIGHT on the floor
    before take-off (the real frame has legs - see the photos), instead of
    a sphere rolling onto its side.

    Motors are sites on the gondola body: front pair force along the site's
    +x, rear pair along +z, all with ctrlrange [0, motor_max]. Rule 13's
    100 gf TOTAL is the C mixer's job; the scene only caps each motor."""
    r_env = p.envelope_radius()
    gondola_z = -(r_env + 0.03)
    m_total = p.vehicle_total_mass()             # (1+k) inflated
    m_gond = p.gondola_mass_kg
    m_env = m_total - m_gond
    if m_env <= 0.0:
        raise ValueError("gondola_mass_kg exceeds the whole vehicle mass budget")
    m_trim = p.trim_heavy_gf * 1e-3
    gc_env = (m_total - m_trim) / m_env
    i_env = p.envelope_shell_inertia
    # gondola as a small box 0.10 x 0.08 x 0.06 for its own inertia
    ig = m_gond / 12.0
    i_gx = ig * (0.08 ** 2 + 0.06 ** 2)
    i_gy = ig * (0.10 ** 2 + 0.06 ** 2)
    i_gz = ig * (0.10 ** 2 + 0.08 ** 2)
    x0, y0 = start_xy
    half = yaw / 2.0
    quat = f"{math.cos(half):.6f} 0 0 {math.sin(half):.6f}"
    lf, xf = p.front_half_span, p.front_x
    lr, xr = p.rear_half_span, p.rear_x

    def motor(name: str, x: float, y: float, vertical: bool) -> str:
        # positions are relative to the GONDOLA body origin (at gondola_z)
        q = "1 0 0 0" if vertical else "0.7071068 0 0.7071068 0"
        return (f'{indent}    <geom name="{name}_geom" type="cylinder" pos="{x:.6f} {y:.6f} 0"\n'
                f'{indent}          quat="{q}" size="0.0045 0.010" material="mat_pod"\n'
                f'{indent}          fluidshape="none" contype="0" conaffinity="0"/>\n'
                f'{indent}    <site name="{name}_site" pos="{x:.6f} {y:.6f} 0" quat="1 0 0 0"/>')

    motors = "\n".join([
        motor("motor_fl", xf,  lf, vertical=False),
        motor("motor_fr", xf, -lf, vertical=False),
        motor("motor_rl", xr,  lr, vertical=True),
        motor("motor_rr", xr, -lr, vertical=True),
    ])
    legs = "\n".join(
        f'{indent}    <geom name="leg_{i}" type="capsule" fromto="{lx:.3f} {ly:.3f} -0.03 {lx:.3f} {ly:.3f} -0.09"\n'
        f'{indent}          size="0.003" material="mat_pod" fluidshape="none" contype="1" conaffinity="1"/>'
        for i, (lx, ly) in enumerate([(0.06, 0.05), (0.06, -0.05), (-0.06, 0.05), (-0.06, -0.05)])
    )
    cam_z_rel = p.camera_dz - gondola_z

    return f"""
{indent}<body name="vehicle" pos="{x0:.6f} {y0:.6f} 1.0" quat="{quat}" gravcomp="{gc_env:.6f}">
{indent}  <freejoint name="vehicle_free"/>
{indent}  <inertial pos="0 0 0" mass="{m_env:.6f}"
{indent}            diaginertia="{i_env:.8f} {i_env:.8f} {i_env:.8f}"/>
{indent}  <geom name="envelope" type="sphere" size="{r_env:.6f}" material="mat_envelope"
{indent}        fluidshape="ellipsoid" fluidcoef="0.5 0.25 {p.envelope_ang_drag_coef:.3f} 1.0 1.0"
{indent}        contype="1" conaffinity="1" solref="{p.contact_timeconst:.3f} 1"/>
{indent}  <site name="vehicle_cg_marker" pos="0 0 0" size="0.01" rgba="1 1 0 1"/>
{indent}  <body name="gondola" pos="0 0 {gondola_z:.6f}" gravcomp="0">
{indent}    <inertial pos="0 0 0" mass="{m_gond:.6f}"
{indent}              diaginertia="{i_gx:.8f} {i_gy:.8f} {i_gz:.8f}"/>
{indent}    <geom name="gondola" type="box" size="0.05 0.04 0.03"
{indent}          material="mat_gondola" fluidshape="none" contype="1" conaffinity="1"/>
{indent}    <site name="camera_site" pos="0.05 0 {cam_z_rel:.6f}" quat="1 0 0 0"/>
{legs}
{motors}
{indent}  </body>
{indent}</body>
""".rstrip("\n")


def build_xml(p: SceneParams) -> str:
    corners = waypoint_corners(p)
    order = course_order(p)
    x1, y1 = corners[0]  # waypoint 1 -> start region + vehicle start position

    half_arena_l = p.arena_length / 2.0
    half_arena_w = p.arena_width / 2.0
    motor_max_n = max(0.0, p.motor_max_gf * GF_TO_N)

    tethers = "\n".join(
        _tether_chain_xml(p, i + 1, cx, cy, "    ") for i, (cx, cy) in enumerate(corners)
    )
    # Spawn INSIDE the 2 m start region (Rule: "2 m diameter start region"
    # centred on waypoint 1) but OFFSET from waypoint 1 itself. Spawning
    # exactly on the waypoint puts the envelope straight through that
    # balloon's tether cable: the original scene did that and the vehicle
    # spawned with 10 contacts, tumbled to 110 degrees of tilt and sank to
    # the floor, which looked exactly like a buoyancy bug and was not one.
    # Offset toward the arena centre so the approach to waypoint 1 is still
    # a realistic first leg.
    _cx, _cy = 0.0, 0.0
    _dx, _dy = _cx - x1, _cy - y1
    _dn = math.hypot(_dx, _dy) or 1.0
    _off = min(0.8, p.start_region_diameter / 2.0 - 0.1)
    _sx = x1 + _dx / _dn * _off
    _sy = y1 + _dy / _dn * _off
    # Facing waypoint 1 (the team orients it so at placement; PREPARE is
    # untimed anyway) unless the caller wants the harder blind start.
    _yaw = math.atan2(y1 - _sy, x1 - _sx) if p.spawn_facing_wp1 else 0.0
    vehicle = _vehicle_xml(p, (_sx, _sy), _yaw, "    ")

    boundary_geoms = f"""
    <geom name="boundary_north" type="box" pos="0 {half_arena_w:.6f} 0.001"
          size="{half_arena_l:.6f} 0.01 0.001" material="mat_boundary"
          contype="0" conaffinity="0"/>
    <geom name="boundary_south" type="box" pos="0 -{half_arena_w:.6f} 0.001"
          size="{half_arena_l:.6f} 0.01 0.001" material="mat_boundary"
          contype="0" conaffinity="0"/>
    <geom name="boundary_east" type="box" pos="{half_arena_l:.6f} 0 0.001"
          size="0.01 {half_arena_w:.6f} 0.001" material="mat_boundary"
          contype="0" conaffinity="0"/>
    <geom name="boundary_west" type="box" pos="-{half_arena_l:.6f} 0 0.001"
          size="0.01 {half_arena_w:.6f} 0.001" material="mat_boundary"
          contype="0" conaffinity="0"/>
""".rstrip("\n")

    xml = f"""<mujoco model="iar_larc_2026_course{p.course}">
  <!-- Auto-generated by sim/scene/build_scene.py -- DO NOT HAND-EDIT.
       Re-run the generator instead (see its module docstring for why). -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{p.timestep}" gravity="0 0 {p.gravity}"
          density="{p.air_density}" viscosity="{p.air_viscosity}"
          integrator="implicitfast"/>

  <custom>
    <text name="course_name" data="course{p.course}_{'rectangle_perimeter' if p.course == 1 else 'figure_eight'}"/>
    <text name="waypoint_order" data="{','.join(str(o) for o in order)}"/>
    <text name="trim_heavy_gf" data="{p.trim_heavy_gf:.3f}"/>
    <text name="motor_max_gf" data="{p.motor_max_gf:.3f}"/>
    <text name="camera_dz" data="{p.camera_dz:.3f}"/>
  </custom>

  <asset>
    <material name="mat_floor" rgba="0.72 0.68 0.58 1"/>
    <material name="mat_boundary" rgba="0.9 0.85 0.1 1"/>
    <material name="mat_start_region" rgba="0.1 0.4 0.9 0.25"/>
    <material name="mat_balloon" rgba="0.82 0.04 0.04 1" specular="0.6" shininess="0.7"/>
    <material name="mat_tether" rgba="0.5 0.5 0.5 1"/>
    <material name="mat_envelope" rgba="0.85 0.87 0.92 0.35" specular="0.3"/>
    <material name="mat_gondola" rgba="0.2 0.2 0.22 1"/>
    <material name="mat_pod" rgba="0.15 0.15 0.18 1"/>
  </asset>

  <worldbody>
    <light name="sun" pos="0 0 6" dir="0 0 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="{half_arena_l + 1.0:.6f} {half_arena_w + 1.0:.6f} 0.1"
          material="mat_floor"/>
{boundary_geoms}
    <geom name="start_region" type="cylinder" pos="{x1:.6f} {y1:.6f} 0.002"
          size="{p.start_region_diameter / 2.0:.6f} 0.001" material="mat_start_region"
          contype="0" conaffinity="0"/>

{tethers}
{vehicle}
  </worldbody>

  <actuator>
    <general name="motor_fl" site="motor_fl_site" gear="1 0 0 0 0 0"
             ctrlrange="0 {motor_max_n:.6f}" gaintype="fixed" gainprm="1 0 0"/>
    <general name="motor_fr" site="motor_fr_site" gear="1 0 0 0 0 0"
             ctrlrange="0 {motor_max_n:.6f}" gaintype="fixed" gainprm="1 0 0"/>
    <general name="motor_rl" site="motor_rl_site" gear="0 0 1 0 0 0"
             ctrlrange="0 {motor_max_n:.6f}" gaintype="fixed" gainprm="1 0 0"/>
    <general name="motor_rr" site="motor_rr_site" gear="0 0 1 0 0 0"
             ctrlrange="0 {motor_max_n:.6f}" gaintype="fixed" gainprm="1 0 0"/>
  </actuator>
</mujoco>
"""
    return xml


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def parse_args(argv: List[str]) -> SceneParams:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    d = SceneParams()
    ap.add_argument("--arena-length", type=float, default=d.arena_length)
    ap.add_argument("--arena-width", type=float, default=d.arena_width)
    ap.add_argument("--wp-rect-length", type=float, default=d.wp_rect_length)
    ap.add_argument("--wp-rect-width", type=float, default=d.wp_rect_width)
    ap.add_argument("--balloon-height", type=float, default=d.balloon_height)
    ap.add_argument("--balloon-diameter", type=float, default=d.balloon_diameter)
    ap.add_argument("--tether-segments", type=int, default=d.tether_segments)
    ap.add_argument("--envelope-diameter", type=float, default=d.envelope_diameter)
    ap.add_argument("--front-half-span", type=float, default=d.front_half_span,
                     help="m, front motors' lateral offset (params.c front_half_span_m)")
    ap.add_argument("--motor-max-gf", type=float, default=d.motor_max_gf)
    ap.add_argument("--trim-heavy-gf", type=float, default=d.trim_heavy_gf,
                     help="gf heavy the vehicle is trimmed (Rule 11: <= 10)")
    ap.add_argument("--gondola-mass-kg", type=float, default=d.gondola_mass_kg)
    ap.add_argument("--blind-start", action="store_true",
                     help="spawn facing +x instead of toward waypoint 1")
    ap.add_argument("--air-density", type=float, default=d.air_density,
                     help="kg/m^3, default is Bogota (~2640 m)")
    ap.add_argument("--course", type=int, choices=[1, 2], default=d.course)
    ap.add_argument("--total-thrust-gf", type=float, default=d.total_thrust_gf)
    ap.add_argument("--out", type=str, default="", help="output XML path")
    ns = ap.parse_args(argv)

    p = SceneParams(
        arena_length=ns.arena_length,
        arena_width=ns.arena_width,
        wp_rect_length=ns.wp_rect_length,
        wp_rect_width=ns.wp_rect_width,
        balloon_height=ns.balloon_height,
        balloon_diameter=ns.balloon_diameter,
        tether_segments=ns.tether_segments,
        envelope_diameter=ns.envelope_diameter,
        front_half_span=ns.front_half_span,
        motor_max_gf=ns.motor_max_gf,
        trim_heavy_gf=ns.trim_heavy_gf,
        gondola_mass_kg=ns.gondola_mass_kg,
        spawn_facing_wp1=not ns.blind_start,
        air_density=ns.air_density,
        course=ns.course,
        total_thrust_gf=ns.total_thrust_gf,
        out_path=ns.out,
    )
    return p


def default_out_path(course: int) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, f"course{course}.xml")


def main(argv: List[str] = None) -> int:
    p = parse_args(argv if argv is not None else sys.argv[1:])
    out_path = p.out_path or default_out_path(p.course)
    xml = build_xml(p)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(xml)

    m = p.vehicle_total_mass()
    print(f"Wrote {out_path}")
    print(f"  course={p.course} ({'rectangle perimeter' if p.course == 1 else 'figure-eight'}), "
          f"order={course_order(p)}")
    print(f"  envelope: D={p.envelope_diameter} m  V={p.envelope_volume():.4f} m^3  "
          f"rho_air={p.air_density} kg/m^3")
    print(f"  vehicle simulated mass (rigid x (1+k) added mass): {m * 1000:.1f} g")
    print(f"  trimmed {p.trim_heavy_gf:.1f} gf HEAVY (Rule 11: within 10 gf); gondola {p.gondola_mass_kg*1000:.0f} g "
          f"hangs {abs(p.envelope_radius()+0.03):.2f} m below the envelope centre (pendulum stiffness)")
    print(f"  motors: 4 x {p.motor_max_gf:.0f} gf cap, front pair at x={p.front_x} y=+/-{p.front_half_span}, "
          f"rear pair at x={p.rear_x} y=+/-{p.rear_half_span}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
