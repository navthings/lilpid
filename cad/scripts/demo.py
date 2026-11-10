import math
import os
import sys
import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stand as S

# sizes in mm, z = 0 is the gimbal pivot
EDF_L = 45.0

TUBE_ID = 80.0
TUBE_WALL = 2.0
TUBE_OD = TUBE_ID + 2 * TUBE_WALL

H = 15.0

DECK_Z = 33.0
DECK_T = 3.0
PLATE_T = 2.5
PLATE_TOP = 72.0

SERVO_L = 22.8
SERVO_W = 12.2
SERVO_H = 22.5
SERVO_TAB_L = 32.3
SERVO_TAB_T = 2.5
SERVO_TAB_Z = 15.9
SHAFT_OFF = 6.1
SHAFT_UP = 3.5

DECK2_Z = 74.0
PCB_W = 36.2
PCB_L = 56.4
RAIL_TOP = 140.0
JOINT_Z = 105.0
TOP_Z = 160.0
NOSE_L = 100.0
WIN_Z = (12.0, 30.0)
PIVOT_Z = 60.0

tube_r = TUBE_ID / 2


def ring(ir, orad, h=H, z0=None):
    z0 = -h / 2 if z0 is None else z0
    return cq.Workplane("XY").circle(orad).circle(ir).extrude(h).translate((0, 0, z0))


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


# gimbal: my fusion rings (inner holds the fan, tabs point up to the servos)
STEP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gimbal.step")
_rings = sorted(cq.importers.importStep(STEP).solids().vals(), key=lambda s: s.BoundingBox().xlen * s.BoundingBox().ylen)
inner, outer, mount = (cq.Workplane(obj=r) for r in _rings)

FAN_OD = 35.0
FAN_TOP = 6.0
edf = (cq.Workplane("XY").circle(FAN_OD / 2).circle(FAN_OD / 2 - 1.2).extrude(EDF_L)
       .union(cq.Workplane("XY").circle(8).extrude(25).translate((0, 0, 8)))
       .translate((0, 0, FAN_TOP - EDF_L)))

# linkage: rod hole on each tab, servo shaft, horn length, rod length
HOLE_A = (0.0, 20.0, 22.0)
HOLE_B = (27.0, 0.0, 17.0)
SHAFT_A = (0.0, 10.0, 47.5)
SHAFT_B = (17.0, 0.0, 47.5)
HORN_A, HORN_B = 10.0, 12.0
ROD_A, ROD_B = 29.5, 33.5
ROD_D = 2.0


def tilt(p, ax, ay):
    # ax = inner ring about x, ay = outer ring about y (the inner ring rides in the outer)
    x, y, z = p
    a = math.radians(ax)
    y, z = y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a)
    b = math.radians(ay)
    return (x * math.cos(b) + z * math.sin(b), y, -x * math.sin(b) + z * math.cos(b))


def horn_angle(which, ax=0.0, ay=0.0, start=None):
    # servo horn angle (deg above level, pointing outward) that keeps the rod the right length
    hole = tilt(HOLE_A, ax, ay) if which == "a" else tilt(HOLE_B, 0, ay)
    s, h, L = (SHAFT_A, HORN_A, ROD_A) if which == "a" else (SHAFT_B, HORN_B, ROD_B)
    best = None
    for i in range(-900, 901):
        t = math.radians(i / 10)
        tip = (s[0], s[1] + h * math.cos(t), s[2] + h * math.sin(t)) if which == "a" else (s[0] + h * math.cos(t), s[1], s[2] + h * math.sin(t))
        e = abs(math.dist(tip, hole) - L)
        if e < 0.05:
            ref = start if start is not None else 20.0
            if best is None or abs(i / 10 - ref) < abs(best[0] - ref):
                best = (i / 10, tip, hole)
    return best


def link(p0, p1, d=ROD_D):
    p0, p1 = cq.Vector(*p0), cq.Vector(*p1)
    return cq.Workplane().add(cq.Solid.makeCylinder(d / 2, (p1 - p0).Length, p0, (p1 - p0).normalized()))


def servo():
    # built with shaft along +z at origin, body below, long side along x
    body = box(-SHAFT_OFF, SERVO_L - SHAFT_OFF, -SERVO_W / 2, SERVO_W / 2, -SHAFT_UP - SERVO_H, -SHAFT_UP)
    tabs = box(-SHAFT_OFF - (SERVO_TAB_L - SERVO_L) / 2, SERVO_L - SHAFT_OFF + (SERVO_TAB_L - SERVO_L) / 2,
               -SERVO_W / 2, SERVO_W / 2,
               -SHAFT_UP - SERVO_H + SERVO_TAB_Z - SERVO_TAB_T, -SHAFT_UP - SERVO_H + SERVO_TAB_Z)
    shaft = cq.Workplane("XY").circle(2.5).extrude(SHAFT_UP).translate((0, 0, -SHAFT_UP))
    return body.union(tabs).union(shaft)


# a: shaft +x, body on the -x side, long side up. b: shaft +y, body on the -y side
servo_a = servo().rotate((0, 0, 0), (1, 0, 1), 180).translate(SHAFT_A)
servo_b = servo().rotate((0, 0, 0), (1, 1, 1), -120).translate(SHAFT_B)


def pose(ax=0.0, ay=0.0, sa=None, sb=None):
    # horns and rods for a given fan tilt
    ta, tip_a, hole_a = horn_angle("a", ax, ay, sa)
    tb, tip_b, hole_b = horn_angle("b", 0, ay, sb)
    return {"horn_a": link(SHAFT_A, tip_a, 5.0).union(link(SHAFT_A, (SHAFT_A[0] + 0.01, SHAFT_A[1], SHAFT_A[2]), 7.0)),
            "horn_b": link(SHAFT_B, tip_b, 5.0),
            "rod_a": link(hole_a, tip_a), "rod_b": link(hole_b, tip_b)}, ta, tb


_p, NEUTRAL_A, NEUTRAL_B = pose()
horn_a, horn_b, rod_a, rod_b = _p["horn_a"], _p["horn_b"], _p["rod_a"], _p["rod_b"]


# deck: servo plates on a disc, slots for the rods
def plate_for(sv, axis):
    bb = sv.val().BoundingBox()
    zc = (bb.zmin + bb.zmax) / 2
    zs = (bb.zmin + 4.75 - 0.2, bb.zmax - 4.75 + 0.2)
    if axis == "x":
        f = bb.xmin + SERVO_TAB_Z - SERVO_TAB_T
        p = box(f - PLATE_T, f, bb.ymin - 4, bb.ymax + 4, DECK_Z, PLATE_TOP)
        p = p.cut(box(f - PLATE_T - 1, f + 1, bb.ymin - 0.2, bb.ymax + 0.2, *zs))
        for z in (zc - 13.8, zc + 13.8):
            p = p.cut(cq.Workplane("YZ").circle(1).extrude(10, both=True).translate((f, (bb.ymin + bb.ymax) / 2, z)))
    else:
        f = bb.ymin + SERVO_TAB_Z - SERVO_TAB_T
        p = box(bb.xmin - 4, bb.xmax + 4, f - PLATE_T, f, DECK_Z, PLATE_TOP)
        p = p.cut(box(bb.xmin - 0.2, bb.xmax + 0.2, f - PLATE_T - 1, f + 1, *zs))
        for z in (zc - 13.8, zc + 13.8):
            p = p.cut(cq.Workplane("XZ").circle(1).extrude(10, both=True).translate(((bb.xmin + bb.xmax) / 2, f, z)))
    return p


deck = cq.Workplane("XY").circle(tube_r - 0.15).extrude(DECK_T).translate((0, 0, DECK_Z))
deck = deck.union(plate_for(servo_a, "x")).union(plate_for(servo_b, "y"))

# sled: second disc with rails for the pcb
sled = cq.Workplane("XY").circle(tube_r - 0.15).extrude(3).translate((0, 0, DECK2_Z))
for y0, y1, g0, g1 in ((17.6, 22, 17.0, 19.2), (-22, -17.6, -19.2, -17.0)):
    sled = sled.union(box(-4, 4, y0, y1, DECK2_Z + 3, RAIL_TOP)).cut(box(-0.9, 0.9, g0, g1, DECK2_Z + 5, RAIL_TOP + 1))
sled = sled.cut(cq.Workplane("XY").circle(6).extrude(10).translate((-20, -22, DECK2_Z - 2)))

pcb = box(-0.8, 0.8, -PCB_W / 2, PCB_W / 2, DECK2_Z + 6, DECK2_Z + 6 + PCB_L)
pico = box(0.8, 4.8, -10.5, 10.5, DECK2_Z + 8.5, DECK2_Z + 8.5 + 51)
imu = box(-30, -12.2, -12.7, 12.7, DECK2_Z + 3, DECK2_Z + 5)
esc = box(8, 32, -12, 12, DECK2_Z + 3, DECK2_Z + 11)

# tube: two prints joined by a coupler, intake windows above the gimbal
tube_low = ring(tube_r, tube_r + TUBE_WALL, h=JOINT_Z + H / 2, z0=-H / 2)
for k in range(6):
    w = box(tube_r - 1, tube_r + TUBE_WALL + 1, -12, 12, *WIN_Z).rotate((0, 0, 0), (0, 0, 1), 30 + 60 * k)
    tube_low = tube_low.cut(w)
tube_high = ring(tube_r, tube_r + TUBE_WALL, h=TOP_Z - JOINT_Z, z0=JOINT_Z)
tube_high = tube_high.cut(cq.Workplane("YZ").circle(6).extrude(50).translate((0, 0, TOP_Z - 12)).rotate((0, 0, 0), (0, 0, 1), 180))
coupler = ring(tube_r - 2.5, tube_r - 0.15, h=24, z0=JOINT_Z - 12)

def nose_loft(r0, tip):
    return (cq.Workplane("XY").circle(r0).workplane(offset=NOSE_L * 0.55).circle(r0 * 0.72)
            .workplane(offset=NOSE_L * 0.3).circle(r0 * 0.32).workplane(offset=NOSE_L * 0.15 - tip).circle(2)
            .loft(ruled=False).translate((0, 0, TOP_Z)))


nose = nose_loft(TUBE_OD / 2, 0).cut(nose_loft(TUBE_OD / 2 - 1.6, 4))
nose = nose.union(ring(tube_r - 0.15 - 1.6, tube_r - 0.15, h=12, z0=TOP_Z - 12))

print_parts = {"inner_ring": inner, "outer_ring": outer, "mount_ring": mount, "deck": deck, "sled": sled,
               "tube_low": tube_low, "tube_high": tube_high, "coupler": coupler, "nose": nose}
dummies = {"edf": edf, "servo_a": servo_a, "servo_b": servo_b, "horn_a": horn_a, "horn_b": horn_b,
           "rod_a": rod_a, "rod_b": rod_b, "pcb": pcb, "pico": pico, "imu": imu, "esc": esc}

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "out")
    os.makedirs(out, exist_ok=True)
    for n, p in print_parts.items():
        cq.exporters.export(p, os.path.join(out, f"{n}.stl"))
    print(f"neutral horn a {NEUTRAL_A:.0f} b {NEUTRAL_B:.0f}")
