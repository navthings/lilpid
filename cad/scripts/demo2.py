import json
import math
import os
import cadquery as cq
import demo as D

v = lambda w: w.val() if hasattr(w, "val") else w
HERE = os.path.dirname(os.path.abspath(__file__))

# sizes in mm, z = 0 is the gimbal pivot
R = D.TUBE_OD / 2
RI = D.TUBE_ID / 2
TOP = 290.0
NOSE = 175.0
LIP = 8.0
LIP_CLEAR = 0.15
STRUT_R = 39.8
STRUT_D = 4.4
BRACE_D = 2.6
N_STRUT = 8
PEG_D = 2.0
PEG_L = 1.5
RING_T = 2.0
FIN_T = 2.0
FIN_SPAN = 24.0
INK = 0.8
FONT = "/usr/share/fonts/opentype/inter/Inter-Medium.otf"

DECK = (D.DECK_Z, D.DECK_Z + D.DECK_T)
SLED = (D.DECK2_Z, D.DECK2_Z + 3)
TOPRING = (152.0, 162.0)
PIVOT_Z = 120.0
BAND = (PIVOT_Z - 7.0, PIVOT_Z + 18.0)
angs = [22.5 + 360 / N_STRUT * i for i in range(N_STRUT)]


def ring(r0, r1, z0, z1):
    return cq.Workplane("XY").circle(r1).circle(r0).extrude(z1 - z0).translate((0, 0, z0))


def disc(r, z0, z1):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((0, 0, z0))


def rod(p0, p1, d):
    p0, p1 = cq.Vector(*p0), cq.Vector(*p1)
    dv = p1 - p0
    return cq.Workplane().add(cq.Solid.makeCylinder(d / 2, dv.Length, p0, dv.normalized()))


def P(a, z, r=STRUT_R):
    return (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z)


def struts(z0, z1, pegs=True):
    out = None
    for a in angs:
        s = rod(P(a, z0), P(a, z1), STRUT_D)
        if pegs:
            s = s.union(rod(P(a, z0 - PEG_L), P(a, z0), PEG_D)).union(rod(P(a, z1), P(a, z1 + PEG_L), PEG_D))
        out = s if out is None else out.union(s)
    return out


def braces(z0, z1):
    out = None
    for i, a in enumerate(angs):
        b = angs[(i + 1) % N_STRUT]
        for p, q in ((P(a, z0), P(b, z1)), (P(b, z0), P(a, z1))):
            s = rod(p, q, BRACE_D)
            out = s if out is None else out.union(s)
    return out


def peg_holes(z0, z1):
    out = None
    for a in angs:
        h = rod(P(a, z0 - 1), P(a, z1 + 1), PEG_D + 0.25)
        out = h if out is None else out.union(h)
    return out


def ogive(L, r, z0):
    rho = (r * r + L * L) / (2 * r)
    pts = [(0, z0 + L)]
    for i in range(1, 60):
        x = L * i / 60
        pts.append((math.sqrt(max(rho * rho - (L - x) ** 2, 0)) + r - rho, z0 + L - x))
    pts += [(r, z0), (0, z0)]
    return cq.Workplane("XZ").polyline(pts).close().revolve(360, (0, 0, 0), (0, 1, 0))


# engine bay: holds the gimbal, fins, open so the fan can breathe
engine = ring(RI, R, -D.H / 2, D.H / 2).union(struts(D.H / 2, DECK[0], pegs=False))
for a in angs:
    engine = engine.union(rod(P(a, DECK[0]), P(a, DECK[0] + PEG_L), PEG_D))
fin = (cq.Workplane("XZ").polyline([(RI + 0.3, -D.H / 2), (RI + 0.3, DECK[0]), (R + FIN_SPAN, 4.0),
                                    (R + FIN_SPAN, -D.H / 2)]).close().extrude(FIN_T / 2, both=True))
for a in angs[::2]:
    engine = engine.union(fin.rotate((0, 0, 0), (0, 0, 1), a))
# holes so the mount ring's pins can go in from outside
engine = engine.cut(cq.Workplane("XZ").circle(1.1).extrude(60, both=True))

# servo deck: full width disc, servos press fit into the plates (no screws)
deck = disc(R, *DECK)


def plate(sv, axis):
    bb = sv.val().BoundingBox()
    zs = (bb.zmin + 4.75 - 0.1, bb.zmax - 4.75 + 0.1)
    if axis == "x":
        f = bb.xmin + D.SERVO_TAB_Z - D.SERVO_TAB_T
        p = D.box(f - D.PLATE_T, f, bb.ymin - 4, bb.ymax + 4, DECK[1] - 0.5, D.PLATE_TOP)
        return p.cut(D.box(f - D.PLATE_T - 1, f + 1, bb.ymin - 0.1, bb.ymax + 0.1, *zs))
    f = bb.ymin + D.SERVO_TAB_Z - D.SERVO_TAB_T
    p = D.box(bb.xmin - 1, bb.xmax + 4, f - D.PLATE_T, f, DECK[1] - 0.5, D.PLATE_TOP)
    return p.cut(D.box(bb.xmin - 0.1, bb.xmax + 0.1, f - D.PLATE_T - 1, f + 1, *zs))


deck = deck.union(plate(D.servo_a, "x")).union(plate(D.servo_b, "y"))
# rod slots: a is wide because the inner ring's rod sways sideways when the outer ring tilts
deck = deck.cut(D.box(-9, 9, 7, 28.5, DECK[0] - 1, DECK[1] + 1))
deck = deck.cut(D.box(17.5, 33, -2.5, 2.5, DECK[0] - 1, DECK[1] + 1))
deck = deck.cut(cq.Workplane("XY").circle(6).extrude(10).translate((-20, -22, DECK[0] - 2)))
deck = deck.cut(peg_holes(*DECK))

# servo bay: struts + cross braces between deck and sled
servo_bay = (ring(RI, R, DECK[1], DECK[1] + RING_T).union(ring(RI, R, SLED[0] - RING_T, SLED[0]))
             .union(struts(DECK[1], SLED[0])).union(braces(DECK[1] + RING_T, SLED[0] - RING_T)))

# electronics sled: pcb slides into the rails
sled = disc(R, *SLED)
for y0, y1, g0, g1 in ((17.6, 22, 17.0, 19.2), (-22, -17.6, -19.2, -17.0)):
    sled = sled.union(D.box(-4, 4, y0, y1, SLED[1], D.RAIL_TOP)).cut(D.box(-0.9, 0.9, g0, g1, SLED[1] + 2, D.RAIL_TOP + 1))
sled = sled.cut(cq.Workplane("XY").circle(6).extrude(10).translate((-20, -22, SLED[0] - 2)))
sled = sled.cut(peg_holes(*SLED))

# avionics bay: solid band in the middle is where the stand grabs it
avionics_bay = (ring(RI, R, SLED[1], SLED[1] + RING_T).union(ring(RI, R, *TOPRING)).union(ring(RI, R, *BAND))
                .union(struts(SLED[1], TOPRING[0]))
                .union(braces(SLED[1] + RING_T, TOPRING[0])))
avionics_bay = avionics_bay.cut(cq.Workplane("XY").circle(RI).extrude(TOPRING[1] - TOPRING[0]).translate((0, 0, TOPRING[0])))

# body: lips slide into the band below and the nose above
lip_r = RI - LIP_CLEAR
body = (ring(RI, R, TOPRING[1], TOP)
        .union(ring(lip_r - 1.6, lip_r, TOPRING[1] - LIP, TOPRING[1]))
        .union(ring(lip_r - 1.6, lip_r, TOP, TOP + LIP)))

nose = ogive(NOSE, R, TOP).cut(ogive(NOSE - 4, R - 1.6, TOP))


# name and logo, inlaid as their own part for a second filament
def wrap(flat, z, angle):
    w = flat.rotate((0, 0, 0), (1, 0, 1), 180).translate((R - 3, 0, 0))
    shell = ring(R - INK, R + 1, -100, 600)
    return w.intersect(shell).translate((0, 0, z)).rotate((0, 0, 0), (0, 0, 1), angle)


name = cq.Workplane("XY").text("liltvc", 15, 6, fontPath=FONT, halign="left", valign="center")
lg = json.load(open(os.path.join(HERE, "logo.json")))
xs = [p[0] for poly in lg["polys"] for p in poly]
ys = [p[1] for poly in lg["polys"] for p in poly]
cx, cy, sc = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, 26.0 / (max(xs) - min(xs))
logo = None
for poly in lg["polys"]:
    pts = [((x - cx) * sc, -(y - cy) * sc) for x, y in poly]
    s = cq.Workplane("XY").polyline(pts).close().extrude(6)
    logo = s if logo is None else logo.union(s)
for x, y, r in lg["circles"]:
    logo = logo.union(cq.Workplane("XY").circle(r * sc).extrude(6).translate(((x - cx) * sc, -(y - cy) * sc, 0)))
logo = logo.rotate((0, 0, 0), (0, 0, 1), -90)
FACE = -55
ink = wrap(name, 186, FACE).union(wrap(logo, 255, FACE))
body = body.cut(ink)
ink_fill = ink.intersect(ring(R - INK, R, -100, 600))

print_parts = {"engine_bay": engine, "deck": deck, "servo_bay": servo_bay, "sled": sled,
               "avionics_bay": avionics_bay, "body": body, "body_ink": ink_fill, "nose": nose,
               "inner_ring": D.inner, "outer_ring": D.outer, "mount_ring": D.mount}
dummies = {k: v_ for k, v_ in D.dummies.items()}

if __name__ == "__main__":
    for n, p in print_parts.items():
        bb = v(p).BoundingBox()
        print(f"{n:13s} {bb.xlen:6.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f}  valid={v(p).isValid()}")
