"""An open car differential with the gears left visible.

The model is a classic two-pinion open differential: two side (sun) gears on
the axle axis, two spider (planet) pinions on the cross pin, a crown ring gear
driven by a spur pinion, and an open carrier cage instead of a housing.  The
bevel gears share one helper: a conical blank plus one trapezoidal tooth that
is circular-patterned and unioned.  The constants at the top drive the build;
edit them and rerun the script to regenerate the gear.

The gears are deliberately not enclosed: the carrier is a ring and four
longitudinal bars, so the bronze side/spider gears stay in view.

Usage: .venv/bin/python examples/differential.py
Writes examples/differential/{differential.wcad,iso.png,front.png,top.png,
preview.png,differential.stl,differential.3mf}
"""

import math
import os
import shutil
import subprocess
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402

OUT = os.path.join("examples", "differential")

BRONZE = (0.82, 0.54, 0.20, 1.0)
COPPER = (0.72, 0.36, 0.16, 1.0)
STEEL = (0.56, 0.61, 0.69, 1.0)
DARK = (0.20, 0.22, 0.25, 1.0)
SHAFT = (0.74, 0.76, 0.80, 1.0)

# Side gears: 58 deg pitch cone, 14 teeth, on the X axle axis.
SIDE_X0, SIDE_X1 = 5.0, 15.0
SIDE_R0, SIDE_R1 = 8.0, 24.0
SIDE_BORE = 4.0
SIDE_TEETH = 14

# Spider gears: 32 deg pitch cone, 8 teeth, on the Z cross-pin axis.
SPIDER_Z0, SPIDER_Z1 = 6.0, 14.0
SPIDER_R0, SPIDER_R1 = 3.75, 8.75
SPIDER_BORE = 2.0
SPIDER_TEETH = 8

# Ring gear: a crown/face gear, 36 axial teeth at a 36 mm pitch radius.
RING_X0, RING_X1 = 12.0, 17.0
RING_R0, RING_R1 = 28.0, 44.0
RING_PITCH_R = 36.0
RING_TEETH = 36

# Drive pinion: a 12-tooth spur gear on a Z axis, meshing with the crown face.
PINION_Z0, PINION_Z1 = 32.0, 40.0
PINION_R = 12.0
PINION_BORE = 4.0
PINION_TEETH = 12

ROOT = 0.8  # tooth root sinks this far into the blank
TIP_W = 0.55  # tooth tip width as a fraction of its root width


def _loop(cad, doc, sketch, points):
    for i, p1 in enumerate(points):
        cad.add_sketch_line(doc, sketch, p1, points[(i + 1) % len(points)])


def _x_gear(cad, doc, name, x0, x1, r0, r1, bore, teeth, tooth_h, color):
    """A straight bevel gear whose axis is world +X, opening toward +X."""
    body_sk = cad.add_sketch(doc, f"{name}_blank", plane="xy")["entity"]
    _loop(cad, doc, body_sk, [(x0, bore), (x0, r0), (x1, r1), (x1, bore)])
    body = cad.revolve(
        doc, body_sk, angle_deg=360, axis="x", segments=128,
        name=f"{name}_blank_solid", color=color,
    )["entity"]
    cad.repair_mesh(doc, body, fix_winding=True)

    w0 = 2 * math.pi * r0 / teeth * 0.45
    w1 = 2 * math.pi * r1 / teeth * 0.45
    t0, t1 = w0 * TIP_W, w1 * TIP_W
    sk0 = cad.add_sketch(
        doc, f"{name}_tooth_in",
        plane={"origin": [x0, 0, 0], "normal": [1, 0, 0], "x_axis": [0, 1, 0]},
    )["entity"]
    _loop(cad, doc, sk0, [(-w0 / 2, r0 - ROOT), (w0 / 2, r0 - ROOT),
                          (t0 / 2, r0 + tooth_h), (-t0 / 2, r0 + tooth_h)])
    sk1 = cad.add_sketch(
        doc, f"{name}_tooth_out",
        plane={"origin": [x1, 0, 0], "normal": [1, 0, 0], "x_axis": [0, 1, 0]},
    )["entity"]
    _loop(cad, doc, sk1, [(-w1 / 2, r1 - ROOT), (w1 / 2, r1 - ROOT),
                          (t1 / 2, r1 + tooth_h), (-t1 / 2, r1 + tooth_h)])
    tooth = cad.loft(doc, [sk0, sk1], name=f"{name}_tooth", color=color)["entity"]
    copies = cad.circular_pattern(
        doc, [tooth], count=teeth, axis=[1, 0, 0],
        name=f"{name}_tooth", color=color,
    )["entities"]
    gear = cad.boolean(
        doc, "union", [body, tooth, *copies], name=name, color=color,
    )["entity"]

    # Drop the construction geometry; the union is baked and independent.
    cad.delete(doc, body_sk)
    cad.delete(doc, tooth)  # also removes the circular-pattern copies
    cad.delete(doc, sk0)
    cad.delete(doc, sk1)
    return gear


def _z_gear(cad, doc, name, z0, z1, r0, r1, bore, teeth, tooth_h, color):
    """A straight bevel gear whose axis is world +Z, opening toward +Z."""
    body_sk = cad.add_sketch(doc, f"{name}_blank", plane="xz")["entity"]
    _loop(cad, doc, body_sk, [(bore, z0), (r0, z0), (r1, z1), (bore, z1)])
    body = cad.revolve(
        doc, body_sk, angle_deg=360, axis="y", segments=128,
        name=f"{name}_blank_solid", color=color,
    )["entity"]
    cad.repair_mesh(doc, body, fix_winding=True)

    w0 = 2 * math.pi * r0 / teeth * 0.45
    w1 = 2 * math.pi * r1 / teeth * 0.45
    t0, t1 = w0 * TIP_W, w1 * TIP_W
    sk0 = cad.add_sketch(
        doc, f"{name}_tooth_in",
        plane={"origin": [0, 0, z0], "normal": [0, 0, 1], "x_axis": [1, 0, 0]},
    )["entity"]
    _loop(cad, doc, sk0, [(r0 - ROOT, -w0 / 2), (r0 + tooth_h, -t0 / 2),
                          (r0 + tooth_h, t0 / 2), (r0 - ROOT, w0 / 2)])
    sk1 = cad.add_sketch(
        doc, f"{name}_tooth_out",
        plane={"origin": [0, 0, z1], "normal": [0, 0, 1], "x_axis": [1, 0, 0]},
    )["entity"]
    _loop(cad, doc, sk1, [(r1 - ROOT, -w1 / 2), (r1 + tooth_h, -t1 / 2),
                          (r1 + tooth_h, t1 / 2), (r1 - ROOT, w1 / 2)])
    tooth = cad.loft(doc, [sk0, sk1], name=f"{name}_tooth", color=color)["entity"]
    copies = cad.circular_pattern(
        doc, [tooth], count=teeth, axis=[0, 0, 1],
        name=f"{name}_tooth", color=color,
    )["entities"]
    gear = cad.boolean(
        doc, "union", [body, tooth, *copies], name=name, color=color,
    )["entity"]

    cad.delete(doc, body_sk)
    cad.delete(doc, tooth)  # also removes the circular-pattern copies
    cad.delete(doc, sk0)
    cad.delete(doc, sk1)
    return gear


def _crown_ring(cad, doc, name, x0, x1, r_in, r_out, pitch_r, teeth, color):
    """A flat crown (face) gear: an annular body with axial teeth on +X."""
    body_sk = cad.add_sketch(doc, f"{name}_blank", plane="xy")["entity"]
    _loop(cad, doc, body_sk, [(x0, r_in), (x0, r_out), (x1, r_out), (x1, r_in)])
    body = cad.revolve(
        doc, body_sk, angle_deg=360, axis="x", segments=160,
        name=f"{name}_blank_solid", color=color,
    )["entity"]
    cad.repair_mesh(doc, body, fix_winding=True)

    width = 2 * math.pi * pitch_r / teeth * 0.52
    depth = 8.0
    height = 6.0
    tooth = cad.add_box(
        doc, name=f"{name}_tooth",
        center=(x1 + height / 2 - 1.0, 0, pitch_r),
        size=(height, width, depth), color=color,
    )["entity"]
    copies = cad.circular_pattern(
        doc, [tooth], count=teeth, axis=[1, 0, 0],
        name=f"{name}_tooth", color=color,
    )["entities"]
    ring = cad.boolean(
        doc, "union", [body, tooth, *copies], name=name, color=color,
    )["entity"]

    cad.delete(doc, body_sk)
    cad.delete(doc, tooth)
    return ring


def _shaft_x(cad, doc, name, center_x, length, radius):
    e = cad.add_cylinder(
        doc, name=name, center=(0, 0, 0), radius=radius, height=length,
        segments=48, color=SHAFT,
    )["entity"]
    cad.transform(doc, e, rotation_deg=(0, 90, 0), translation=(center_x, 0, 0))
    return e


def main() -> None:
    os.makedirs(OUT, exist_ok=True)

    cad = Wright.spawn()
    doc = cad.create_document("open_differential")["doc"]

    # --- bevel gear set ------------------------------------------------------
    side_l = _x_gear(
        cad, doc, "side_gear_L", SIDE_X0, SIDE_X1, SIDE_R0, SIDE_R1,
        SIDE_BORE, SIDE_TEETH, 2.0, BRONZE,
    )
    side_r = cad.mirror(
        doc, [side_l], plane="yz", name="side_gear_R", color=BRONZE,
    )["entities"][0]

    spider_p = _z_gear(
        cad, doc, "spider_gear_P", SPIDER_Z0, SPIDER_Z1, SPIDER_R0, SPIDER_R1,
        SPIDER_BORE, SPIDER_TEETH, 1.5, COPPER,
    )
    # Half a tooth pitch puts a spider gap where a side-gear tooth arrives.
    cad.transform(doc, spider_p, rotation_deg=(0, 0, 180 / SPIDER_TEETH))
    spider_n = cad.mirror(
        doc, [spider_p], plane="xy", name="spider_gear_N", color=COPPER,
    )["entities"][0]

    ring = _crown_ring(
        cad, doc, "ring_gear", RING_X0, RING_X1, RING_R0, RING_R1,
        RING_PITCH_R, RING_TEETH, STEEL,
    )
    pinion = _z_gear(
        cad, doc, "drive_pinion", PINION_Z0, PINION_Z1, PINION_R, PINION_R,
        PINION_BORE, PINION_TEETH, 1.8, STEEL,
    )
    # The pinion root circle sits just outside the crown tooth tips; its
    # teeth then reach back into the crown tooth spaces.
    pinion_x = RING_X1 + 5.0 + PINION_R
    cad.transform(
        doc, pinion,
        rotation_deg=(0, 0, 180 / PINION_TEETH),
        translation=(pinion_x, 0, (PINION_Z0 + PINION_Z1) / 2),
    )

    # --- open carrier --------------------------------------------------------
    hoop_sk = cad.add_sketch(doc, "carrier_hoop_profile", plane="xy")["entity"]
    _loop(cad, doc, hoop_sk, [(-2.2, 26.5), (-2.2, 29.5),
                              (2.2, 29.5), (2.2, 26.5)])
    hoop = cad.revolve(
        doc, hoop_sk, angle_deg=360, axis="x", segments=128,
        name="carrier_hoop_solid", color=DARK,
    )["entity"]
    cad.repair_mesh(doc, hoop, fix_winding=True)

    bars = []
    for i, (cy, cz) in enumerate([(0, 28), (0, -28), (28, 0), (-28, 0)]):
        bars.append(
            cad.add_box(
                doc, name=f"carrier_bar_{i}", center=(0, cy, cz),
                size=(30, 4, 4), color=DARK,
            )["entity"]
        )
    carrier = cad.boolean(
        doc, "union", [hoop, *bars], name="carrier", color=DARK,
    )["entity"]
    cad.delete(doc, hoop)
    for b in bars:
        cad.delete(doc, b)
    cad.delete(doc, hoop_sk)

    # --- cross pin and shafts ----------------------------------------------
    cross_pin = cad.add_cylinder(
        doc, name="cross_pin", center=(0, 0, 0), radius=2.0, height=56,
        segments=48, color=SHAFT,
    )["entity"]

    axle_l = _shaft_x(cad, doc, "axle_L", -25.0, 20.0, 3.2)
    axle_r = _shaft_x(cad, doc, "axle_R", 25.0, 20.0, 3.2)
    pinion_shaft = cad.add_cylinder(
        doc, name="pinion_shaft", center=(pinion_x, 0, 48),
        radius=3.5, height=16, segments=48, color=SHAFT,
    )["entity"]
    pinion_flange = cad.add_cylinder(
        doc, name="pinion_flange", center=(pinion_x, 0, 56.5),
        radius=7.0, height=3.0, segments=48, color=SHAFT,
    )["entity"]

    # --- report and save ----------------------------------------------------
    gears = [side_l, side_r, spider_p, spider_n, ring, pinion]
    for eid in gears:
        info = next(e for e in cad.get_scene(doc)["entities"] if e["id"] == eid)
        print(f"{info['name']:16s} id={eid:3d} "
              f"bbox={info['world_bbox']['min']}..{info['world_bbox']['max']}")

    report = cad.validate(doc, entities=gears)
    print(f"validate gears: ok={report['ok']} solids={len(report['solids'])}")
    for s in report["solids"]:
        if not s["ok"]:
            print(f"  {s['name']}: open={s['open_edges']} "
                  f"nonmanifold={s['non_manifold_edges']} "
                  f"self={s['self_intersections']} vol={s['volume']:.0f}")

    stl = os.path.abspath(os.path.join(OUT, "differential.stl"))
    mf = os.path.abspath(os.path.join(OUT, "differential.3mf"))
    print("stl:", cad.export_stl(doc, stl))
    print("3mf:", cad.export_3mf(doc, mf))

    with open(__file__, encoding="utf-8") as f:
        source = {"language": "python", "text": f.read()}
    wcad = os.path.abspath(os.path.join(OUT, "differential.wcad"))
    saved = cad.save_document(doc, wcad, source=source)
    print(f"saved {saved['path']} ({saved['bytes']} bytes, {saved['params']} params)")

    views = [
        ("iso", (-80, -145, 75), (4, 0, 12), 42, False),
        ("front", (-30, -170, 35), (4, 0, 14), 40, True),
        ("top", (-40, -120, 120), (4, 0, 8), 42, False),
    ]
    for name, eye, target, fov, ortho in views:
        png = cad.render_offscreen(
            doc, width=1000, height=850,
            eye=eye, target=target, up=(0, 0, 1),
            fov_deg=fov, ortho=ortho,
            background=(1.0, 1.0, 1.0, 1.0),
            path=os.path.join(OUT, f"{name}.png"),
        )
        print(f"rendered {name}: {png['path']}")

    magick = shutil.which("magick")
    if magick:
        for name in ("iso", "front", "top"):
            path = os.path.join(OUT, f"{name}.png")
            subprocess.run(
                [magick, path, "-fuzz", "2%", "-trim", "+repage",
                 "-bordercolor", "white", "-border", "30",
                 "-strip", "-define", "png:exclude-chunks=date,time", path],
                check=True,
            )
        subprocess.run(
            [magick, *[os.path.join(OUT, f"{n}.png") for n in ("iso", "front", "top")],
             "-resize", "620x527", "-background", "white", "-gravity", "center",
             "-extent", "620x527", "+append",
             "-strip", "-define", "png:exclude-chunks=date,time",
             os.path.join(OUT, "preview.png")],
            check=True,
        )
        print(f"rendered preview: {os.path.join(OUT, 'preview.png')}")


if __name__ == "__main__":
    main()
