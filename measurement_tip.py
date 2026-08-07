"""Parametric 3D-printable optical-tracking measurement tip (ART pointer replacement).

Run headless:
    "C:\\Program Files\\FreeCAD 1.1\\bin\\freecadcmd.exe" measurement_tip.py

Writes measurement_tip.FCStd, measurement_tip.step, tip_body.stl, handle.stl and
markers.json next to this file.

Frame: origin at the front face of the printed nose, +X toward the handle,
+Z up (= print bed normal). The nominal measurement point is the ground end of
the steel pin at X = -PIN_PROTRUDE, Z = PIN_Z; its true offset comes from pivot
calibration, the printed geometry only has to be rigid and repeatable.
"""
import itertools
import json
import os

import FreeCAD as App
import Part
import Mesh
import MeshPart
from FreeCAD import Vector

# ---------------------------------------------------------------- parameters

BEAM_W = 12.0          # shaft/arm width  (Y)
BEAM_H = 14.0          # shaft/arm height (Z), bottom face on the bed at Z=0

NOSE_LEN = 26.0        # tapered nose, front face at X=0; M1 sits behind it
NOSE_TIP_W = 10.0      # front face width  (Y)
NOSE_TIP_H = 10.0      # front face height (Z)

SHAFT_END = 155.0      # rear end of the tracked body

# Printed stylus -- the measurement point itself, a separate part.
#
# A BALL, not a printed "point". A 0.4 mm nozzle cannot make a point: the apex
# comes out a blob roughly one extrusion wide, it deforms on contact, and its
# contact patch shifts as you tilt the tool, so the pivot calibration it feeds
# is biased by however you happened to hold it. Pivoting a sphere in a divot
# recovers the sphere CENTRE exactly, at any tilt, and the ball diameter drops
# out entirely. This is why CMM styli are spheres. A sphere is also the better
# sighting target for SPAAM - it presents the same silhouette from every
# direction, so there is no foreshortening bias when you line it up on the
# crosshair.
#
# Separate part because the ball cannot print on the lying-flat body: its
# underside would be an unsupported overhang. Standing upright it is
# self-supporting, and it becomes the replaceable wear item.
TIP_STYLE = "ball"     # "ball" (recommended) or "cone" for probing features
TIP_Z = 5.0            # stylus axis height, centred in the nose front face
TIP_REACH = 13.0       # nose face -> ball centre / cone apex
TIP_BALL_D = 5.0
TIP_CONE_L = 5.0       # cone variant only
TIP_NECK_D = 3.8       # ball meets the neck at 49 deg, so no support needed
STYLUS_PLUG_D = 4.0
STYLUS_PLUG_L = 12.0
STYLUS_SEAT_D = 6.5    # conical seat: self-centring, and it sets insertion
                       # depth so a replacement stylus lands in the same place.
                       # Its length is forced: the seat is an overhang on BOTH
                       # parts and the two constraints are reciprocal (the nose
                       # countersink prints lying down, the stylus collar prints
                       # standing up), so only length == radial step puts both
                       # at 45 deg. Do not change one without the other.
STYLUS_SEAT_L = (STYLUS_SEAT_D - STYLUS_PLUG_D) / 2
FIT = 0.2              # bore clearance

# Marker layout, solved by opt_markers.py. Two properties matter and they pull
# against each other: all 10 pairwise distances distinct (smallest gap 5.33 mm,
# the tracker's labelling margin), and every 4-marker SUBSET non-coplanar
# (worst 6.52 mm out-of-plane). The second is why all five heights differ -- a
# planar set has a mirror-flip pose ambiguity, so a body that is non-planar
# only thanks to one raised marker turns ambiguous the moment that marker is
# occluded, rather than merely less accurate.
CROSS_X = 73.5
ARM_NEG = 42.5         # marker centre on the arm at -Y
ARM_POS = 57.0         # marker centre on the arm at +Y
MARKERS = [            # (name, x, y, z)
    ("M1", 30.0, 0.0, 36.0),
    ("M2", CROSS_X, -ARM_NEG, 40.0),
    ("M3", CROSS_X, ARM_POS, 44.0),
    ("M4", 104.5, 0.0, 57.5),
    ("M5", 144.0, 0.0, 29.0),
]

MARKER_D = 12.0        # reflective sphere diameter
BASE_TO_CENTRE = 6.0   # post top face -> sphere centre, for your marker studs
POST_D = 7.0           # shank diameter of the shortest post
TALL_POST_D = 11.0     # ... of the tallest; scaled linearly in between so the
POST_H_REF = 36.0      # long posts don't become the compliant link
FLARE_H = 6.0          # conical post base, keeps the joint stiff and printable
STUD_D = 2.5           # pilot for a tapped M3 -- 4.0 for an M3 heat-set insert
STUD_DEPTH = 9.0

ARM_OVERHANG = 6.0     # arm material past the outermost marker centre
GUSSET = 14.0

TENON_W = 8.0          # keyed, flat-bottomed handle joint (behind every marker)
TENON_H = 10.0
TENON_LEN = 27.0

HANDLE_LEN = 95.0
HANDLE_W = 20.0
HANDLE_H = 18.0
HANDLE_FIT = 0.25      # per-side clearance on the socket
GROOVE_R = 3.0
GROOVE_N = 5

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- helpers


def box(x0, y0, z0, dx, dy, dz):
    return Part.makeBox(dx, dy, dz, Vector(x0, y0, z0))


def rect_wire(x, half_w, h):
    """Rectangle in the YZ plane at the given X, corners in a fixed order."""
    pts = [Vector(x, -half_w, 0), Vector(x, half_w, 0),
           Vector(x, half_w, h), Vector(x, -half_w, h)]
    return Part.makePolygon(pts + [pts[0]])


def teardrop(r, length, start):
    """Horizontal bore with a 45 deg gabled roof.

    A plain round hole printed on its side has a horizontal ceiling that droops
    into the bore. This one is self-supporting, which keeps the stylus seating
    on axis.
    """
    k = r / 2 ** 0.5
    gable = [start + Vector(0, -k, k), start + Vector(0, 0, r * 2 ** 0.5),
             start + Vector(0, k, k)]
    roof = Part.Face(Part.makePolygon(gable + [gable[0]]))
    return (Part.makeCylinder(r, length, start, Vector(1, 0, 0))
            .fuse(roof.extrude(Vector(length, 0, 0))))


def gusset(x, y, sx, sy):
    """Right triangle in the corner at (x, y), legs pointing along sx/sy."""
    pts = [Vector(x + sx * GUSSET, y, 0), Vector(x, y, 0),
           Vector(x, y + sy * GUSSET, 0)]
    face = Part.Face(Part.makePolygon(pts + [pts[0]]))
    return face.extrude(Vector(0, 0, BEAM_H))


def post(x, y, z_centre):
    """Marker post: conical flare, straight shank, tapped stud hole.

    Shank diameter grows with exposed length -- a tall thin post would be the
    softest element in the chain, and post flex is measurement error.
    """
    top = z_centre - BASE_TO_CENTRE
    exposed = max(0.0, top - BEAM_H)
    t = min(1.0, exposed / POST_H_REF)
    d = POST_D + (TALL_POST_D - POST_D) * t
    # Cap the flare at the beam width: any wider and it hangs off the side of
    # the beam with nothing under it, which would force supports.
    flare_d = min(d * 1.7, BEAM_W)
    base_z = 10.0                                   # start inside the beam
    flare = Part.makeCone(flare_d / 2, d / 2, FLARE_H, Vector(x, y, base_z))
    shank = Part.makeCylinder(d / 2, top - base_z, Vector(x, y, base_z))
    hole = Part.makeCylinder(STUD_D / 2, STUD_DEPTH + 1,
                             Vector(x, y, top - STUD_DEPTH))
    return flare.fuse(shank), hole


# ---------------------------------------------------------------- tip body

nose = Part.makeLoft([rect_wire(0.0, NOSE_TIP_W / 2, NOSE_TIP_H),
                      rect_wire(NOSE_LEN, BEAM_W / 2, BEAM_H)], True)

shaft = box(NOSE_LEN, -BEAM_W / 2, 0.0, SHAFT_END - NOSE_LEN, BEAM_W, BEAM_H)

arm_y0 = -(ARM_NEG + ARM_OVERHANG)
arm_y1 = ARM_POS + ARM_OVERHANG
arm = box(CROSS_X - BEAM_W / 2, arm_y0, 0.0, BEAM_W, arm_y1 - arm_y0, BEAM_H)

tenon = box(SHAFT_END, -TENON_W / 2, 0.0, TENON_LEN, TENON_W, TENON_H)

body = nose.fuse(shaft).fuse(arm).fuse(tenon)
for sx in (-1, 1):
    for sy in (-1, 1):
        body = body.fuse(gusset(CROSS_X + sx * BEAM_W / 2, sy * BEAM_W / 2, sx, sy))

bore_d = STYLUS_PLUG_D + FIT
holes = [teardrop(bore_d / 2, STYLUS_PLUG_L + 2, Vector(0, 0, TIP_Z)),
         Part.makeCone((STYLUS_SEAT_D + FIT) / 2, bore_d / 2, STYLUS_SEAT_L,
                       Vector(-0.001, 0, TIP_Z), Vector(1, 0, 0))]
for name, x, y, z in MARKERS:
    solid, hole = post(x, y, z)
    body = body.fuse(solid)
    holes.append(hole)

for h in holes:
    body = body.cut(h)
body = body.removeSplitter()

# ---------------------------------------------------------------- stylus

# Modelled upright, which is also how it prints: plug on the bed, ball on top,
# every cone converging upward so nothing overhangs.
seat_z = STYLUS_PLUG_L + STYLUS_SEAT_L          # plane that meets the nose face
apex_z = seat_z + TIP_REACH
neck_r = TIP_NECK_D / 2

stylus = Part.makeCylinder(STYLUS_PLUG_D / 2, STYLUS_PLUG_L, Vector(0, 0, 0))
stylus = stylus.fuse(Part.makeCone(STYLUS_PLUG_D / 2, STYLUS_SEAT_D / 2,
                                   STYLUS_SEAT_L, Vector(0, 0, STYLUS_PLUG_L)))
if TIP_STYLE == "ball":
    ball_r = TIP_BALL_D / 2
    neck_top = apex_z - (ball_r ** 2 - neck_r ** 2) ** 0.5
    stylus = stylus.fuse(Part.makeCone(STYLUS_SEAT_D / 2, neck_r,
                                       neck_top - seat_z, Vector(0, 0, seat_z)))
    stylus = stylus.fuse(Part.makeSphere(ball_r, Vector(0, 0, apex_z)))
else:
    neck_top = apex_z - TIP_CONE_L
    stylus = stylus.fuse(Part.makeCone(STYLUS_SEAT_D / 2, neck_r,
                                       neck_top - seat_z, Vector(0, 0, seat_z)))
    stylus = stylus.fuse(Part.makeCone(neck_r, 0.0, TIP_CONE_L,
                                       Vector(0, 0, neck_top)))
stylus = stylus.removeSplitter()

# Same solid rotated into place on the tool: stylus +Z maps to body -X.
stylus_fitted = stylus.copy()
stylus_fitted.rotate(Vector(0, 0, 0), Vector(0, 1, 0), -90)
stylus_fitted.translate(Vector(seat_z, 0, TIP_Z))

# ---------------------------------------------------------------- handle

socket_w = TENON_W + 2 * HANDLE_FIT
socket_h = TENON_H + HANDLE_FIT
handle = box(SHAFT_END, -HANDLE_W / 2, 0.0, HANDLE_LEN, HANDLE_W, HANDLE_H)
handle = handle.cut(box(SHAFT_END - 1, -socket_w / 2, 0.0,
                        TENON_LEN + 1.5, socket_w, socket_h))
for i in range(GROOVE_N):
    gx = SHAFT_END + TENON_LEN + 12.0 + i * (GROOVE_R * 2 + 6.0)
    handle = handle.cut(Part.makeCylinder(
        GROOVE_R, HANDLE_W + 2, Vector(gx, -HANDLE_W / 2 - 1, HANDLE_H),
        Vector(0, 1, 0)))
handle = handle.removeSplitter()

# ---------------------------------------------------------------- output

doc = App.newDocument("measurement_tip")
o_body = doc.addObject("Part::Feature", "tip_body")
o_body.Shape = body
o_handle = doc.addObject("Part::Feature", "handle")
o_handle.Shape = handle
o_stylus = doc.addObject("Part::Feature", "stylus")       # print orientation
o_stylus.Shape = stylus
o_fitted = doc.addObject("Part::Feature", "stylus_fitted")  # assembly position
o_fitted.Shape = stylus_fitted
o_markers = []
for name, x, y, z in MARKERS:
    o = doc.addObject("Part::Feature", "marker_" + name)
    o.Shape = Part.makeSphere(MARKER_D / 2, Vector(x, y, z))
    o_markers.append(o)
doc.recompute()
doc.saveAs(os.path.join(HERE, "measurement_tip.FCStd"))

Part.export([o_body, o_handle, o_fitted], os.path.join(HERE, "measurement_tip.step"))
for obj, name in ((o_body, "tip_body"), (o_handle, "handle"), (o_stylus, "stylus")):
    mesh = MeshPart.meshFromShape(Shape=obj.Shape, LinearDeflection=0.02,
                                  AngularDeflection=0.1, Relative=False)
    mesh.write(os.path.join(HERE, name + ".stl"))

pairs = [{"pair": f"{a[0]}-{b[0]}",
          "mm": round(Vector(*a[1:]).sub(Vector(*b[1:])).Length, 3)}
         for a, b in itertools.combinations(MARKERS, 2)]
pairs.sort(key=lambda p: p["mm"])
data = {
    "frame": ("origin at the stylus ball centre, +X toward the handle; "
              "nominal only, pivot calibration measures the real offset"),
    "units": "mm",
    "tip_style": TIP_STYLE,
    "tip_ball_diameter": TIP_BALL_D if TIP_STYLE == "ball" else None,
    "markers": [{"name": n, "x": round(x + TIP_REACH, 3), "y": round(y, 3),
                 "z": round(z - TIP_Z, 3)} for n, x, y, z in MARKERS],
    "marker_diameter": MARKER_D,
    "pairwise_distances": pairs,
    "min_pairwise_gap": round(min(b["mm"] - a["mm"]
                                  for a, b in zip(pairs, pairs[1:])), 3),
}
with open(os.path.join(HERE, "markers.json"), "w") as f:
    json.dump(data, f, indent=2)

bb = body.BoundBox
print("tip body   %.1f x %.1f x %.1f mm, %.1f cm3, solid=%s"
      % (bb.XLength, bb.YLength, bb.ZLength, body.Volume / 1000.0, body.isValid()))
bb = handle.BoundBox
print("handle     %.1f x %.1f x %.1f mm, %.1f cm3, solid=%s"
      % (bb.XLength, bb.YLength, bb.ZLength, handle.Volume / 1000.0, handle.isValid()))
bb = stylus.BoundBox
print("stylus     %.1f x %.1f x %.1f mm, %.2f cm3, solid=%s, style=%s, reach %.1f mm"
      % (bb.XLength, bb.YLength, bb.ZLength, stylus.Volume / 1000.0,
         stylus.isValid(), TIP_STYLE, TIP_REACH))
print("min pairwise gap %.2f mm, closest pair %.2f mm"
      % (data["min_pairwise_gap"], pairs[0]["mm"]))
