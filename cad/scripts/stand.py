import math
import cadquery as cq

# sizes in mm
TUBE_OD = 84.0
CLEAR = 0.3
WALL = 3.0
H = 14.0
TOP = 18.0
SLIT = 2.0
TIE_W = 5.0
TIE_DEPTH = 1.0
PIN_D = 2.0
GAP = 0.5
BOSS_D = 7.0
ROOM = 1.5
STOP_DEG = 20.0
RIM = 5.0
WIN_W = 44.0
ARM_DEG = 225.0
ARM_L = 75.0
ARM_W = 16.0
ARM_H = 20.0
POST = 16.0
POST_FIT = 0.4
PEG = 8.0
PEG_L = 10.0
PIVOT_H = 210.0
BASE_L = 170.0
BASE_W = 110.0
BASE_T = 12.0
POCKET_T = 9.0

r_t = TUBE_OD / 2
collar_ir = r_t + CLEAR
collar_or = collar_ir + WALL
yoke_ir = math.hypot(collar_or, TOP) + ROOM
yoke_or = yoke_ir + WALL
frame_ir = max(math.hypot(yoke_or, H / 2), math.hypot(collar_or + 5, TOP)) + ROOM
frame_or = frame_ir + WALL
th = math.radians(STOP_DEG)
half = ((frame_ir - r_t * math.cos(th)) / math.sin(th)) * math.cos(th) - r_t * math.sin(th)


def ring(ir, orad, h=H, z0=None):
    z0 = -h / 2 if z0 is None else z0
    return cq.Workplane("XY").circle(orad).circle(ir).extrude(h).translate((0, 0, z0))


def bosses(r_from, r_to, axis):
    # posts on +/- axis
    out = None
    for sign in (1, -1):
        c = cq.Workplane("YZ").circle(BOSS_D / 2).extrude(r_to - r_from).translate((r_from, 0, 0))
        if sign < 0:
            c = c.rotate((0, 0, 0), (0, 0, 1), 180)
        if axis == "y":
            c = c.rotate((0, 0, 0), (0, 0, 1), 90)
        out = c if out is None else out.union(c)
    return out


def pin_hole(axis, span=200):
    return cq.Workplane("YZ" if axis == "x" else "XZ").circle(PIN_D / 2).extrude(span, both=True)


# collar: clamps the tube, tilts on x pins
collar = ring(collar_ir, collar_or, h=TOP + H / 2)
groove = ring(collar_or - TIE_DEPTH, collar_or + 1, h=TIE_W, z0=TOP - 2 - TIE_W)
slit = cq.Workplane("XY").box(SLIT, WALL * 3, 100).translate((0, (collar_ir + collar_or) / 2, 0))
collar = collar.cut(groove).cut(slit).union(bosses(collar_or - 0.5, yoke_ir - GAP, "x")).cut(pin_hole("x"))

# yoke: x pins inside, y pins outside
yoke = ring(yoke_ir, yoke_or).union(bosses(yoke_or - 0.5, frame_ir - GAP, "y"))
yoke = yoke.cut(pin_hole("x")).cut(pin_hole("y"))

# frame: tall sleeve = 20deg stop, windows for the x pins, arm to the post
frame = ring(frame_ir, frame_or, h=2 * half)
for a in (0, 180):
    win = (cq.Workplane("XY").box(WALL + 4, WIN_W, 2 * (half - RIM))
           .translate(((frame_ir + frame_or) / 2, 0, 0)).rotate((0, 0, 0), (0, 0, 1), a))
    frame = frame.cut(win)
frame = frame.cut(pin_hole("y"))

r0, r1 = frame_ir + 0.5, frame_or + ARM_L
blk = POST + 8
r_s = r1 - blk / 2
zb = -half + ARM_H / 2
arm = cq.Workplane("XY").box(r1 - r0, ARM_W, ARM_H).translate(((r0 + r1) / 2, 0, zb))
arm = arm.union(cq.Workplane("XY").box(blk, blk, ARM_H).translate((r_s, 0, zb)))
sock = POST + POST_FIT
arm = arm.cut(cq.Workplane("XY").box(sock, sock, ARM_H - 3).translate((r_s, 0, -half + (ARM_H - 3) / 2)))
arm = arm.cut(cq.Workplane("XY").circle(frame_ir).extrude(4 * half).translate((0, 0, -2 * half)))
frame = frame.union(arm.rotate((0, 0, 0), (0, 0, 1), ARM_DEG))

# post: two pieces, peg joint
top_z = -half + ARM_H - 3
bot_z = -PIVOT_H + 3
post_len = top_z - bot_z
seg = post_len / 2
hole = PEG + POST_FIT
post_low = (cq.Workplane("XY").box(POST, POST, seg).translate((0, 0, seg / 2))
            .union(cq.Workplane("XY").box(PEG, PEG, PEG_L).translate((0, 0, seg + PEG_L / 2))))
post_top = (cq.Workplane("XY").box(POST, POST, seg).translate((0, 0, seg / 2))
            .cut(cq.Workplane("XY").box(hole, hole, PEG_L + 1).translate((0, 0, (PEG_L + 1) / 2))))

# base: socket near the rocket end, pocket for weights
sx = BASE_L / 2 - blk / 2 - 3
base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T).translate((0, 0, BASE_T / 2))
base = base.union(cq.Workplane("XY").box(blk, blk, 20).translate((sx, 0, BASE_T + 10)))
base = base.cut(cq.Workplane("XY").box(sock, sock, BASE_T + 20).translate((sx, 0, 3 + (BASE_T + 20) / 2)))
pw = BASE_L - blk - 20
base = base.cut(cq.Workplane("XY").box(pw, BASE_W - 12, POCKET_T)
                .translate((-BASE_L / 2 + 6 + pw / 2, 0, BASE_T - POCKET_T / 2 + 0.01)))

if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "out")
    os.makedirs(out, exist_ok=True)
    parts = {"collar": collar, "yoke": yoke, "frame": frame,
             "post_low": post_low, "post_top": post_top, "base": base}
    for name, p in parts.items():
        cq.exporters.export(p, os.path.join(out, f"{name}.stl"))

    # assembled positions for the step
    ang = math.radians(ARM_DEG)
    px, py = r_s * math.cos(ang), r_s * math.sin(ang)
    tube = ring(r_t - 2, r_t, h=260, z0=-170)
    asm = (cq.Assembly(name="stand")
           .add(tube, name="tube", color=cq.Color(0.8, 0.8, 0.8, 0.4))
           .add(collar, name="collar", color=cq.Color(0.9, 0.4, 0.2))
           .add(yoke, name="yoke", color=cq.Color(0.3, 0.5, 0.9))
           .add(frame, name="frame", color=cq.Color(0.95, 0.75, 0.2))
           .add(post_low, name="post_low", loc=cq.Location((px, py, bot_z)), color=cq.Color(0.4, 0.4, 0.4))
           .add(post_top, name="post_top", loc=cq.Location((px, py, bot_z + seg)), color=cq.Color(0.5, 0.5, 0.5))
           .add(base, name="base", loc=cq.Location((px - sx * math.cos(ang + math.pi), py - sx * math.sin(ang + math.pi), -PIVOT_H),
                                                   (0, 0, 1), ARM_DEG + 180),
                color=cq.Color(0.3, 0.3, 0.3)))
    asm.export(os.path.join(out, "stand.step"))
    print(f"collar {collar_ir*2:.1f}-{collar_or*2:.1f}  yoke {yoke_ir*2:.1f}-{yoke_or*2:.1f}  "
          f"frame {frame_ir*2:.1f}-{frame_or*2:.1f} x {2*half:.1f} tall  post {seg:.1f} x2  arm reach {r_s:.1f}")
