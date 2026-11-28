"""A skeleton mechanical watch movement with its going train left visible.

The brief is a working mechanical watch you can look inside: a barrel and
mainspring, a three-stage wheel train, an escape wheel, a pallet-fork anchor,
a balance wheel and hairspring, held in an open case by narrow bridges.

The layout is a real compound train, not a flat decoration.  Each arbor
carries a wheel and a pinion at different heights, so the barrel drives the
center pinion, the center wheel drives the third pinion, and the third wheel
drives the escape pinion.  Tooth profiles are involute (20 deg pressure angle)
with a little backlash, the centre distances are the exact pitch sums, and
every mesh is phased so a tooth enters a gap rather than a tooth.

`train_deg` is the barrel angle in degrees.  Every gear follows at its own
ratio (barrel -> centre -> third -> escape), and because the initial phases are
correct the train keeps meshing at every angle.  Set it with `set_parameter`
and the wheels turn; the mainspring and the hands follow too.  The anchor and
balance are posed at rest because wright models rigid parts, not joints.

The 66 mm case, open back plate, bridges, screws, jewels, blued springs and
polished hands are all parameterised by the constants at the top.  The output
is a display movement (the parts are separate solids), not a running watch:
there is no spring power and no dynamic impulse to keep a balance oscillating.

Usage: .venv/bin/python examples/skeleton_watch.py
Writes examples/skeleton_watch/{skeleton_watch.wcad, iso.png, top.png,
detail.png, in_motion.png, skeleton_watch.stl, skeleton_watch.3mf}
"""

import math
import os
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

OUT = os.path.join("examples", "skeleton_watch")

# --- palette ---------------------------------------------------------------
BRASS = (0.86, 0.61, 0.24, 1.0)
GOLD = (0.96, 0.78, 0.35, 1.0)
STEEL = (0.70, 0.74, 0.80, 1.0)
DARK = (0.17, 0.20, 0.25, 1.0)
PLATE = (0.21, 0.24, 0.29, 1.0)
BLUED = (0.24, 0.35, 0.68, 1.0)
RUBY = (0.74, 0.035, 0.10, 1.0)
WHITE = (0.88, 0.90, 0.94, 1.0)

# --- mechanism -------------------------------------------------------------
MODULE = 0.5          # gear module [mm]
BARREL_T = 56         # barrel wheel
CENTER_P = 10         # centre pinion
CENTER_T = 48         # centre wheel
THIRD_P = 9           # third pinion
THIRD_T = 36          # third wheel
ESCAPE_P = 7          # escape pinion
ESCAPE_T = 15         # escape wheel

# Wheel centres [mm].  Distances are module*(teeth_a+teeth_b)/2 exactly.
B = (-12.64, 10.61)   # barrel
C = (0.0, 0.0)        # centre
T = (13.39, -4.87)    # third
E = (17.07, -14.97)   # escape
BL = (3.0, -19.0)     # balance
AP = (10.55, -16.83)  # anchor pivot, between escape wheel and balance

# Heights [mm].  The train descends: each wheel sits on the arbor above its
# pinion, so from the open bridge side you can see every pinion entering the
# wheel it drives.  Gear thickness is 1.4, one level every 2.4.
Z_BARREL = 13.6
Z_CENTER = 11.2
Z_THIRD = 8.8
Z_ESCAPE = 6.4
Z_ANCHOR = 6.3
Z_BALANCE = 8.8
Z_HAIRSPRING = 10.4
Z_BALANCE_COCK = 12.0
Z_TRAIN_BRIDGE = 13.2
Z_BARREL_BRIDGE = 16.2

CASE_R = 33.0
CASE_W = 4.2


def _angle(a, b):
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def _gear_points(n, m, pressure=20.0, addendum=1.0, dedendum=1.25,
                 samples=4, backlash=0.08):
    """Closed involute spur-gear outline, tooth centred on local angle 0."""
    rp = m * n / 2
    ra = rp + addendum * m
    rf = rp - dedendum * m
    rb = rp * math.cos(math.radians(pressure))
    inv_p = math.tan(math.radians(pressure)) - math.radians(pressure)
    r0 = max(rb, rf)
    radii = [r0 + (ra - r0) * i / (samples - 1) for i in range(samples)]
    back = backlash / (2 * rp)
    pts = []
    step = 2 * math.pi / n

    def psi(r):
        a = math.acos(max(-1.0, min(1.0, rb / r)))
        return math.pi / (2 * n) + inv_p - (math.tan(a) - a) - back

    for k in range(n):
        th = k * step
        for r in radii:
            p = psi(r)
            pts.append((r * math.cos(th - p), r * math.sin(th - p)))
        tip = psi(ra)
        for j in range(1, 4):
            ang = th - tip + 2 * tip * j / 4
            pts.append((ra * math.cos(ang), ra * math.sin(ang)))
        for r in reversed(radii):
            p = psi(r)
            pts.append((r * math.cos(th + p), r * math.sin(th + p)))
        for j in range(1, 3):
            ang = th + step * j / 3
            pts.append((rf * math.cos(ang), rf * math.sin(ang)))
    return pts


def _escape_points():
    """A 15-tooth club-tooth escape wheel: pointed tips, undercut backs."""
    n = ESCAPE_T
    r_root, r_tip = 2.55, 4.30
    pts = []
    step = 2 * math.pi / n
    for k in range(n):
        th = k * step
        # root, rising back, club tip, steep locking face, root
        pts.append((r_root * math.cos(th), r_root * math.sin(th)))
        pts.append((3.35 * math.cos(th + step * 0.36),
                    3.35 * math.sin(th + step * 0.36)))
        pts.append((r_tip * math.cos(th + step * 0.62),
                    r_tip * math.sin(th + step * 0.62)))
        pts.append((r_tip * 0.97 * math.cos(th + step * 0.80),
                    r_tip * 0.97 * math.sin(th + step * 0.80)))
        pts.append((r_root * math.cos(th + step * 0.97),
                    r_root * math.sin(th + step * 0.97)))
    return pts


def _sketch_loop(cad, doc, name, loops):
    """Bulk-create a sketch from one or more closed polygons of points.

    Coordinates are snapped to 1e-9 mm (and exact zero) so the sketch solver's
    replay is byte-stable: the profiles are generated geometrically, and a
    last-bit difference in a coordinate would otherwise change the saved
    `.wcad` on the next open.
    """
    def clean(v):
        v = round(float(v), 9)
        return 0.0 if abs(v) < 1e-9 else v

    elems = []
    eid = 1
    for pts in loops:
        for i, p in enumerate(pts):
            q = pts[(i + 1) % len(pts)]
            elems.append({"type": "line", "id": eid,
                          "p1": [clean(p[0]), clean(p[1])],
                          "p2": [clean(q[0]), clean(q[1])]})
            eid += 1
    return cad.add_sketch_curves(doc, name, elems)["sketch"]


def _ring_points(cx, cy, r, segments=48, reverse=False):
    pts = [(cx + r * math.cos(2 * math.pi * i / segments),
            cy + r * math.sin(2 * math.pi * i / segments))
           for i in range(segments)]
    return list(reversed(pts)) if reverse else pts


def _gear(cad, doc, name, teeth, center, z, color, thickness=1.4,
          phase=0.0, ratio=0.0, bore=0.0, holes=0, hole_r=0.0, hole_at=0.0,
          profile=None):
    """Extrude a gear profile and place it at `center`, with a phase knob."""
    pts = profile if profile is not None else _gear_points(teeth, MODULE)
    loops = [pts]
    if bore > 0:
        loops.append(_ring_points(0, 0, bore, 40))
    for k in range(holes):
        a = 2 * math.pi * k / holes
        loops.append(_ring_points(hole_at * math.cos(a),
                                  hole_at * math.sin(a), hole_r, 24))
    sk = _sketch_loop(cad, doc, name + "_profile", loops)
    solid = cad.extrude(doc, sk, thickness, name=name, color=color)["entity"]
    rot = f"{phase:g}"
    if ratio:
        rot += f"+train_deg*{ratio:g}"
    cad.transform(doc, solid, translation=(center[0], center[1], z),
                  rotation_deg=(0, 0, rot))
    # Hide the construction ribbon behind the dark plate.
    cad.set_color(doc, sk, PLATE)
    return solid


def _spiral(cad, doc, name, center, z, r0, r1, turns, width, height, color,
            segments_per_turn=48, rotation=None):
    """A flat spiral ribbon (mainspring or hairspring) as one extruded loop."""
    n = turns * segments_per_turn
    outer, inner = [], []
    for i in range(n + 1):
        t = i / n
        a = 2 * math.pi * turns * t
        r = r0 + (r1 - r0) * t
        outer.append((r + width / 2, a))
        inner.append((r - width / 2, a))
    pts = [(rr * math.cos(a), rr * math.sin(a)) for rr, a in outer]
    pts += [(rr * math.cos(a), rr * math.sin(a)) for rr, a in reversed(inner)]
    sk = _sketch_loop(cad, doc, name + "_profile", [pts])
    solid = cad.extrude(doc, sk, height, name=name, color=color)["entity"]
    cad.transform(doc, solid, translation=(center[0], center[1], z),
                  rotation_deg=(0, 0, rotation) if rotation else None)
    cad.set_color(doc, sk, PLATE)
    return solid


def _bar(cad, doc, name, p0, p1, width, z0, z1, color, extend=0.4):
    """A rectangular bar between two XY points, extruded in Z."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    a = math.degrees(math.atan2(dy, dx))
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (z0 + z1) / 2)
    e = cad.add_box(doc, name=name, center=(0, 0, 0),
                    size=(length + extend, width, z1 - z0), color=color)["entity"]
    cad.transform(doc, e, rotation_deg=(0, 0, a), translation=mid)
    return e


def _beam(cad, doc, name, p0, p1, width, z0, z1, color, bosses=(), pillar=None):
    """A bridge bar with bosses and a slim turned pillar down to its support."""
    parts = [_bar(cad, doc, name + "_web", p0, p1, width, z0, z1, color)]
    for i, (bx, by, br) in enumerate(bosses):
        parts.append(cad.add_cylinder(
            doc, name=f"{name}_boss{i}", center=(bx, by, (z0 + z1) / 2),
            radius=br, height=z1 - z0, segments=40, color=color)["entity"])
    bridge = cad.boolean(doc, "union", parts, name=name, color=color)["entity"]
    for part in parts:
        cad.delete(doc, part)
    if pillar is not None:
        px, py, from_z = pillar
        cad.add_cylinder(doc, name=name + "_pillar",
                         center=(px, py, (from_z + z0) / 2), radius=0.85,
                         height=z0 - from_z, segments=28, color=STEEL)
        cad.add_cylinder(doc, name=name + "_pillar_foot",
                         center=(px, py, from_z + 0.25), radius=1.35,
                         height=0.5, segments=28, color=STEEL)
        cad.add_cylinder(doc, name=name + "_pillar_cap",
                         center=(px, py, z0 - 0.2), radius=1.15,
                         height=0.4, segments=28, color=STEEL)
    return bridge


def _screw(cad, doc, name, x, y, z, color=STEEL):
    cad.add_cylinder(doc, name=name, center=(x, y, z + 0.3),
                     radius=1.15, height=0.6, segments=24, color=color)
    cad.add_box(doc, name=name + "_slot", center=(x, y, z + 0.62),
                size=(2.1, 0.30, 0.12), color=DARK)


def _jewel(cad, doc, name, x, y, z, radius):
    cad.add_cylinder(doc, name=name, center=(x, y, z + 0.16),
                     radius=radius, height=0.32, segments=32, color=RUBY)


def main():
    os.makedirs(OUT, exist_ok=True)

    cad = Wright.spawn()
    doc = cad.create_document("skeleton_watch")["doc"]
    cad.set_parameter(doc, "train_deg", 0.0,
                      "barrel rotation; the whole going train follows")

    # --- case and back plate (primitives: no construction ribbon) ----------
    plate = cad.add_cylinder(
        doc, name="back_plate", center=(0, 0, 1.1), radius=30.2,
        height=2.2, segments=128, color=PLATE)["entity"]

    case_out = cad.add_cylinder(
        doc, name="case_outer", center=(0, 0, CASE_W / 2), radius=CASE_R,
        height=CASE_W, segments=160, color=DARK)["entity"]
    case_in = cad.add_cylinder(
        doc, name="case_inner_cut", center=(0, 0, CASE_W / 2 + 0.2),
        radius=30.0, height=CASE_W + 0.4, segments=160, color=DARK)["entity"]
    case = cad.boolean(doc, "difference", [case_out, case_in],
                       name="case", color=DARK)["entity"]
    cad.delete(doc, case_out)
    cad.delete(doc, case_in)
    cad.chamfer(doc, case, 0.35, edges={"normals": [[0, 0, 1]]})

    # Shallow oil sinks are deliberately omitted: the plate is a clean dark
    # backdrop, and any through-hole would expose the construction ribbons at
    # z=0 below it.

    # Chapter ring: eleven raised gold batons on the bezel; 3 o'clock is the
    # crown, so that position is left open.
    for k in range(1, 12):
        a = math.radians(30 * k)
        baton = cad.add_box(doc, name=f"baton{k}", center=(0, 0, 0),
                            size=(2.0, 0.65, 0.5), color=GOLD)["entity"]
        cad.transform(doc, baton, rotation_deg=(0, 0, 30 * k),
                      translation=(31.5 * math.cos(a), 31.5 * math.sin(a),
                                   CASE_W + 0.25))

    # --- going train --------------------------------------------------------
    # Driver phases point a tooth at the next pinion; pinion phases point a
    # gap back at the driver.  Ratios follow from the tooth counts.
    r_center = -BARREL_T / CENTER_P
    r_third = r_center * (-CENTER_T / THIRD_P)
    r_escape = r_third * (-THIRD_T / ESCAPE_P)

    barrel = _gear(cad, doc, "barrel_wheel", BARREL_T, B, Z_BARREL, BRASS,
                   phase=_angle(B, C), ratio=1.0, bore=1.6)
    center_pinion = _gear(cad, doc, "center_pinion", CENTER_P, C,
                          Z_BARREL - 0.1, STEEL, thickness=1.8,
                          phase=_angle(C, B) - 180 / CENTER_P,
                          ratio=r_center)
    center_wheel = _gear(cad, doc, "center_wheel", CENTER_T, C, Z_CENTER,
                         BRASS, phase=_angle(C, T), ratio=r_center, bore=1.0,
                         holes=5, hole_r=1.9, hole_at=7.3)
    third_pinion = _gear(cad, doc, "third_pinion", THIRD_P, T,
                         Z_CENTER - 0.1, STEEL, thickness=1.8,
                         phase=_angle(T, C) - 180 / THIRD_P, ratio=r_third)
    third_wheel = _gear(cad, doc, "third_wheel", THIRD_T, T, Z_THIRD, BRASS,
                        phase=_angle(T, E), ratio=r_third, bore=0.8,
                        holes=4, hole_r=1.45, hole_at=5.4)
    escape_pinion = _gear(cad, doc, "escape_pinion", ESCAPE_P, E,
                          Z_THIRD - 0.1, STEEL, thickness=1.8,
                          phase=_angle(E, T) - 180 / ESCAPE_P,
                          ratio=r_escape)
    # A tooth sits under the upper pallet and a gap under the lower one.
    escape_wheel = _gear(cad, doc, "escape_wheel", ESCAPE_T, E, Z_ESCAPE,
                         STEEL, profile=_escape_points(), bore=0.8,
                         phase=_angle(E, AP) - 30.0, ratio=r_escape,
                         holes=4, hole_r=0.38, hole_at=1.65)

    # Barrel arbor and mainspring (the spring sits on top of the barrel wheel,
    # where it is visible under the open barrel bridge).
    cad.add_cylinder(doc, name="barrel_arbor", center=(B[0], B[1], 9.7),
                     radius=1.5, height=15.0, segments=32, color=STEEL)
    cad.add_cylinder(doc, name="barrel_hub", center=(B[0], B[1], 15.4),
                     radius=2.45, height=0.8, segments=32, color=STEEL)
    _spiral(cad, doc, "mainspring", B, 15.2, 2.2, 9.2, 7, 0.32, 0.55, BLUED,
            rotation="train_deg")

    # Centre, third, escape and balance arbors.
    cad.add_cylinder(doc, name="center_arbor", center=(C[0], C[1], 11.4),
                     radius=0.95, height=18.6, segments=28, color=STEEL)
    cad.add_cylinder(doc, name="third_arbor", center=(T[0], T[1], 8.4),
                     radius=0.75, height=12.4, segments=24, color=STEEL)
    cad.add_cylinder(doc, name="escape_arbor", center=(E[0], E[1], 8.4),
                     radius=0.65, height=12.4, segments=24, color=STEEL)
    cad.add_cylinder(doc, name="balance_staff", center=(BL[0], BL[1], 7.5),
                     radius=0.70, height=10.6, segments=24, color=STEEL)

    # --- anchor -------------------------------------------------------------
    # Steel lever: pivot boss, two pallet arms around the escape wheel and a
    # fork tail reaching the balance roller.
    wheel_dir = math.atan2(AP[1] - E[1], AP[0] - E[0])
    pallet_r = 4.70
    pallets = []
    for da in (-30.0, 30.0):
        a = wheel_dir + math.radians(da)
        pallets.append((E[0] + pallet_r * math.cos(a),
                        E[1] + pallet_r * math.sin(a), a))
    tail_dir = math.atan2(BL[1] - AP[1], BL[0] - AP[0])
    tail_end = (AP[0] + 4.0 * math.cos(tail_dir),
                AP[1] + 4.0 * math.sin(tail_dir))

    anchor_parts = [cad.add_cylinder(
        doc, name="anchor_boss", center=(AP[0], AP[1], Z_ANCHOR + 0.6),
        radius=1.35, height=1.2, segments=32, color=STEEL)["entity"]]
    for i, (px, py, pa) in enumerate(pallets):
        anchor_parts.append(_bar(cad, doc, f"anchor_arm{i}", AP, (px, py),
                                 0.60, Z_ANCHOR, Z_ANCHOR + 1.2, STEEL,
                                 extend=0.0))
    anchor_parts.append(_bar(cad, doc, "anchor_tail", AP, tail_end, 0.82,
                             Z_ANCHOR, Z_ANCHOR + 1.2, STEEL, extend=0.2))
    perp = (math.sin(tail_dir), -math.cos(tail_dir))
    for s in (-1, 1):
        p0 = (tail_end[0] - 0.15 * math.cos(tail_dir) + s * 0.60 * perp[0],
              tail_end[1] - 0.15 * math.sin(tail_dir) + s * 0.60 * perp[1])
        p1 = (tail_end[0] + 1.65 * math.cos(tail_dir) + s * 0.60 * perp[0],
              tail_end[1] + 1.65 * math.sin(tail_dir) + s * 0.60 * perp[1])
        anchor_parts.append(_bar(cad, doc, f"anchor_fork{s}", p0, p1, 0.26,
                                 Z_ANCHOR, Z_ANCHOR + 1.2, STEEL, extend=0.0))
    cad.boolean(doc, "union", anchor_parts, name="anchor", color=STEEL)
    for part in anchor_parts:
        cad.delete(doc, part)

    # Ruby pallet stones at the two escape-wheel contacts.
    pallet_loops = []
    for px, py, pa in pallets:
        # local rectangle, long axis radial to the escape wheel
        rx, ry = math.cos(pa), math.sin(pa)
        tx, ty = -ry, rx
        c = (px - AP[0], py - AP[1])
        half_r, half_t = 0.30, 0.55
        pallet_loops.append([
            (c[0] - half_r * rx - half_t * tx, c[1] - half_r * ry - half_t * ty),
            (c[0] + half_r * rx - half_t * tx, c[1] + half_r * ry - half_t * ty),
            (c[0] + half_r * rx + half_t * tx, c[1] + half_r * ry + half_t * ty),
            (c[0] - half_r * rx + half_t * tx, c[1] - half_r * ry + half_t * ty),
        ])
    psk = _sketch_loop(cad, doc, "pallets_profile", pallet_loops)
    pal = cad.extrude(doc, psk, 1.0, name="pallets", color=RUBY)["entity"]
    cad.transform(doc, pal, translation=(AP[0], AP[1], Z_ANCHOR + 0.1))
    cad.set_color(doc, psk, PLATE)
    cad.add_cylinder(doc, name="anchor_arbor", center=(AP[0], AP[1], 6.9),
                     radius=0.8, height=1.2, segments=24, color=STEEL)
    # The anchor is carried by a short post rising straight out of the plate;
    # it disappears under the anchor boss in the top view.
    cad.add_cylinder(doc, name="anchor_post", center=(AP[0], AP[1], 4.25),
                     radius=0.7, height=4.1, segments=28, color=STEEL)
    cad.add_cylinder(doc, name="anchor_post_foot",
                     center=(AP[0], AP[1], 2.45), radius=1.15,
                     height=0.5, segments=28, color=STEEL)
    cad.add_cylinder(doc, name="anchor_post_cap",
                     center=(AP[0], AP[1], 6.1), radius=1.0,
                     height=0.4, segments=28, color=STEEL)

    # --- balance, roller and hairspring ------------------------------------
    ring_out = cad.add_cylinder(
        doc, name="balance_blank", center=(BL[0], BL[1], Z_BALANCE + 0.5),
        radius=7.0, height=1.0, segments=72, color=BRASS)["entity"]
    ring_cut = cad.add_cylinder(
        doc, name="balance_cut", center=(BL[0], BL[1], Z_BALANCE + 0.5),
        radius=5.9, height=1.2, segments=72, color=BRASS)["entity"]
    ring = cad.boolean(doc, "difference", [ring_out, ring_cut],
                       name="balance_rim", color=BRASS)["entity"]
    cad.delete(doc, ring_out)
    cad.delete(doc, ring_cut)
    balance_parts = [ring]
    for k in range(3):
        a = math.radians(120 * k)
        cx = BL[0] + 2.95 * math.cos(a)
        cy = BL[1] + 2.95 * math.sin(a)
        spoke = cad.add_box(doc, name=f"balance_spoke{k}",
                            center=(0, 0, 0), size=(6.9, 0.95, 0.78),
                            color=BRASS)["entity"]
        cad.transform(doc, spoke, rotation_deg=(0, 0, 120 * k),
                      translation=(cx, cy, Z_BALANCE + 0.5))
        balance_parts.append(spoke)
    balance_parts.append(cad.add_cylinder(
        doc, name="balance_hub", center=(BL[0], BL[1], Z_BALANCE + 0.7),
        radius=1.6, height=1.6, segments=32, color=BRASS)["entity"])
    cad.boolean(doc, "union", balance_parts, name="balance", color=BRASS)
    for part in balance_parts:
        cad.delete(doc, part)

    cad.add_cylinder(doc, name="roller_table", center=(BL[0], BL[1], 8.5),
                     radius=2.2, height=0.45, segments=36, color=STEEL)
    pin_dir = math.atan2(AP[1] - BL[1], AP[0] - BL[0])
    pin = (BL[0] + 3.2 * math.cos(pin_dir), BL[1] + 3.2 * math.sin(pin_dir))
    cad.add_cylinder(doc, name="impulse_pin", center=(pin[0], pin[1], 7.95),
                     radius=0.33, height=1.5, segments=20, color=STEEL)

    _spiral(cad, doc, "hairspring", BL, Z_HAIRSPRING, 1.3, 5.7, 5, 0.16, 0.35,
            BLUED, segments_per_turn=56)
    cad.add_cylinder(doc, name="hairspring_collet",
                     center=(BL[0], BL[1], Z_HAIRSPRING + 0.3),
                     radius=1.35, height=0.7, segments=28, color=STEEL)

    # --- bridges, cocks, pillars and screws --------------------------------
    barrel_bridge = _beam(
        cad, doc, "barrel_bridge", (-29.13, 10.60), B, 2.8,
        Z_BARREL_BRIDGE, Z_BARREL_BRIDGE + 1.0, STEEL,
        bosses=[(B[0], B[1], 2.7)], pillar=(-29.13, 10.60, CASE_W))
    train_bridge = _beam(
        cad, doc, "train_bridge", (24.0, -13.0), E, 2.4,
        Z_TRAIN_BRIDGE, Z_TRAIN_BRIDGE + 1.0, STEEL,
        bosses=[(E[0], E[1], 1.3)], pillar=(24.0, -13.0, 2.2))
    bridge_parts = [
        train_bridge,
        _bar(cad, doc, "train_bridge_web2", E, T, 2.4,
             Z_TRAIN_BRIDGE, Z_TRAIN_BRIDGE + 1.0, STEEL),
        cad.add_cylinder(doc, name="train_boss_T",
                         center=(T[0], T[1], Z_TRAIN_BRIDGE + 0.5),
                         radius=1.9, height=1.0, segments=40,
                         color=STEEL)["entity"],
    ]
    cad.boolean(doc, "union", bridge_parts, name="train_bridge", color=STEEL)
    for part in bridge_parts:
        cad.delete(doc, part)
    balance_cock = _beam(
        cad, doc, "balance_cock", (8.02, -29.94), BL, 2.8,
        Z_BALANCE_COCK, Z_BALANCE_COCK + 1.0, STEEL,
        bosses=[(BL[0], BL[1], 2.2)], pillar=(8.02, -29.94, CASE_W))

    _screw(cad, doc, "screw_barrel", -29.13, 10.60, Z_BARREL_BRIDGE + 1.0)
    _screw(cad, doc, "screw_train", 24.0, -13.0, Z_TRAIN_BRIDGE + 1.0)
    _screw(cad, doc, "screw_balance", 8.02, -29.94, Z_BALANCE_COCK + 1.0)
    _jewel(cad, doc, "jewel_barrel", B[0], B[1], Z_BARREL_BRIDGE + 1.0, 1.9)
    _jewel(cad, doc, "jewel_T", T[0], T[1], Z_TRAIN_BRIDGE + 1.0, 1.45)
    _jewel(cad, doc, "jewel_E", E[0], E[1], Z_TRAIN_BRIDGE + 1.0, 1.0)
    _jewel(cad, doc, "jewel_balance", BL[0], BL[1], Z_BALANCE_COCK + 1.0, 1.65)

    # --- crown at 3 o'clock -------------------------------------------------
    crown = _gear(cad, doc, "crown", 20, (0, 0), 0, STEEL, thickness=3.8,
                  profile=_gear_points(20, 0.30, addendum=1.0, dedendum=1.0))
    cad.transform(doc, crown, rotation_deg=(0, 90, 0),
                  translation=(35.6, 0, 3.4))
    stem = cad.add_cylinder(doc, name="crown_stem", center=(0, 0, 0),
                            radius=1.2, height=4.4, segments=24,
                            color=STEEL)["entity"]
    cad.transform(doc, stem, rotation_deg=(0, 90, 0),
                  translation=(33.6, 0, 3.4))

    # --- hands --------------------------------------------------------------
    def hand(name, length, width, z, angle, rot=None):
        outer = [
            (-2.3, -0.32), (-1.2, -1.0), (-0.2, -1.15), (1.3, -0.55),
            (2.6, -width), (length - 1.3, -width * 0.55),
            (length, 0.0), (length - 1.3, width * 0.55), (2.6, width),
            (1.3, 0.55), (-0.2, 1.15), (-1.2, 1.0), (-2.3, 0.32),
            (-2.8, 0.0),
        ]
        slot = [
            (4.2, 0.0), (6.0, 0.30), (length - 2.6, 0.18),
            (length - 1.5, 0.0), (length - 2.6, -0.18), (6.0, -0.30),
        ]
        sk = _sketch_loop(cad, doc, name + "_profile", [outer, slot])
        solid = cad.extrude(doc, sk, 0.48, name=name, color=WHITE)["entity"]
        turn = f"{angle}+{rot}" if rot else angle
        cad.transform(doc, solid, rotation_deg=(0, 0, turn),
                      translation=(0, 0, z))
        cad.set_color(doc, sk, PLATE)
        return solid

    # The hands keep a true 12:1 ratio while the centre wheel drives the
    # minute hand; the hour hand is display-geared (there is no motion work).
    hand("hour_hand", 16.0, 1.05, 19.0, 150.0, rot="-0.4666667*train_deg")
    hand("minute_hand", 24.0, 0.85, 18.4, 30.0, rot="-5.6*train_deg")
    cad.add_cylinder(doc, name="hand_cap", center=(0, 0, 19.75),
                     radius=2.0, height=1.15, segments=36, color=STEEL)
    cad.add_sphere(doc, name="hand_cap_dome", center=(0, 0, 20.65),
                   radius=1.05, segments=20, color=STEEL)

    # --- checks and deliverables -------------------------------------------
    scene = cad.get_scene(doc)
    print(f"parts: {len(scene['entities'])} entities")
    for name in ("barrel_wheel", "center_wheel", "third_wheel", "escape_wheel",
                 "anchor", "balance", "case"):
        e = next(x for x in scene["entities"] if x["name"] == name)
        print(f"  {name:14s} {len(e['mesh']['positions']):5d} verts  "
              f"{len(e['mesh']['indices']) // 3:5d} tris")

    # The train must stay clear at every phase: roll the barrel through a full
    # turn and check each meshing pair.  (The profiles carry 0.08 mm of
    # backlash, so the flanks never touch.)
    pairs = [
        ("barrel/center", "barrel_wheel", "center_pinion"),
        ("center/third", "center_wheel", "third_pinion"),
        ("third/escape", "third_wheel", "escape_pinion"),
        ("anchor/escape", "anchor", "escape_wheel"),
    ]
    ids = {e["name"]: e["id"] for e in scene["entities"]}
    for deg in range(0, 360, 15):
        cad.set_parameter(doc, "train_deg", deg)
        for label, a, b in pairs:
            hit = cad.check_interference(doc, entities=[ids[a], ids[b]],
                                         tolerance=0.0005)
            if hit["hits"]:
                print(f"  interference {label} at train_deg={deg}")

    cad.set_parameter(doc, "train_deg", 0.0)
    report = cad.validate(doc)
    print(f"validate: ok={report['ok']} solids={report['totals']['solids']} "
          f"sketches={report['totals']['sketches']} "
          f"volume={report['totals']['volume']:.0f} mm3")
    for solid in report["solids"]:
        if not solid["ok"]:
            print(f"  bad solid: {solid['name']} open={solid['open_edges']} "
                  f"nonmanifold={solid['non_manifold_edges']} "
                  f"self={solid['self_intersections']} "
                  f"degenerate={solid['degenerate_tris']}")

    for name, eye, target, up, path, size in [
        ("iso", (92.0, -104.0, 96.0), (0.0, 0.0, 6.5), (0.0, 0.0, 1.0),
         "iso.png", (1300, 1100)),
        ("top", (0.0, 0.0, 130.0), (0.0, 0.0, 0.0), (0.0, 1.0, 0.0),
         "top.png", (1100, 1100)),
        ("detail", (38.0, -58.0, 36.0), (9.0, -16.0, 8.5), (0.0, 0.0, 1.0),
         "detail.png", (1200, 1000)),
    ]:
        p = os.path.join(OUT, path)
        cad.render_offscreen(doc, width=size[0], height=size[1], eye=eye,
                             target=target, up=up, fov_deg=32.0,
                             background=(0.055, 0.065, 0.085, 1.0), path=p)
        print(f"rendered {name:6s} -> {p}")

    # A posed frame proves the train rolls without fouling.
    cad.set_parameter(doc, "train_deg", 4.0)
    p = os.path.join(OUT, "in_motion.png")
    cad.render_offscreen(doc, width=1000, height=1000, eye=(80, -95, 90),
                         target=(0, 0, 7), up=(0, 0, 1), fov_deg=32.0,
                         background=(0.055, 0.065, 0.085, 1.0), path=p)
    cad.set_parameter(doc, "train_deg", 0.0)

    stl = os.path.abspath(os.path.join(OUT, "skeleton_watch.stl"))
    mf = os.path.abspath(os.path.join(OUT, "skeleton_watch.3mf"))
    s = cad.export_stl(doc, stl)
    print(f"stl: {s['tris']} tris, {s['bytes']} bytes")
    m = cad.export_3mf(doc, mf)
    print(f"3mf: {m['objects']} objects, {m['tris']} tris, {m['bytes']} bytes")

    with open(__file__, encoding="utf-8") as f:
        source = {"language": "python", "text": f.read()}
    saved = cad.save_document(doc, os.path.abspath(
        os.path.join(OUT, "skeleton_watch.wcad")), source=source)
    print(f"wcad: {saved['bytes']} bytes, {saved['params']} params")
    cad.close()


if __name__ == "__main__":
    main()
