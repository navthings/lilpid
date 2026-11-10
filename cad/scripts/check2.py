import math, itertools
import cadquery as cq
import demo as D, demo2 as M, stand as S

v = lambda w: w.val() if hasattr(w, "val") else w
hit = lambda a, b: v(a).intersect(v(b)).Volume()
ok = True

print("1) parts overlapping each other (should be none)")
allp = dict(M.print_parts); allp.update({k: x for k, x in M.dummies.items()})
skip = {("servo_a", "horn_a"), ("servo_b", "horn_b"), ("inner_ring", "rod_a"), ("outer_ring", "rod_b"), ("horn_a", "rod_a"), ("horn_b", "rod_b"),
        ("inner_ring", "outer_ring"), ("outer_ring", "mount_ring"), ("engine_bay", "mount_ring")}
for a, b in itertools.combinations(allp, 2):
    if (a, b) in skip or (b, a) in skip: continue
    A, B = v(allp[a]).BoundingBox(), v(allp[b]).BoundingBox()
    if A.xmax < B.xmin or B.xmax < A.xmin or A.ymax < B.ymin or B.ymax < A.ymin or A.zmax < B.zmin or B.zmax < A.zmin: continue
    h = hit(allp[a], allp[b])
    if h > 0.5:
        ok = False; print(f"   CLASH {a} x {b}: {h:.1f} mm3")

print("2) gimbal swinging to +-15 and +-20 deg on both axes at once, rods and horns moving with it")
fixed = {k: x for k, x in allp.items() if k not in ("inner_ring", "outer_ring", "edf", "rod_a", "rod_b", "horn_a", "horn_b")}
own = {"rod_a": ("inner", "horn_a", "servo_a"), "rod_b": ("outer", "horn_b", "servo_b"), "horn_a": ("servo_a", "rod_a"), "horn_b": ("servo_b", "rod_b")}
bad = 0
span = {"a": [0, 0], "b": [0, 0]}
for dx in (-20, -15, -8, 0, 8, 15, 20):
    for dy in (-20, -15, 0, 15, 20):
        try:
            links, ta, tb = D.pose(dx, dy)
        except TypeError:
            bad += 1; print(f"   LOCKED linkage at x{dx} y{dy}"); continue
        if abs(dx) <= 15 and abs(dy) <= 15:
            span["a"] = [min(span["a"][0], ta - D.NEUTRAL_A), max(span["a"][1], ta - D.NEUTRAL_A)]
            span["b"] = [min(span["b"][0], tb - D.NEUTRAL_B), max(span["b"][1], tb - D.NEUTRAL_B)]
        o = v(D.outer).rotate((0, 0, 0), (0, 1, 0), dy)
        mv = {"outer": o}
        for n, p in (("inner", D.inner), ("edf", D.edf)):
            mv[n] = v(p).rotate((0, 0, 0), (1, 0, 0), dx).rotate((0, 0, 0), (0, 1, 0), dy)
        mv.update(links)
        for n, q in mv.items():
            for m, f in fixed.items():
                if m in own.get(n, ()): continue
                if hit(q, f) > 0.5: bad += 1; print(f"   CLASH {n} vs {m} at x{dx} y{dy}")
        for n, m in itertools.combinations(mv, 2):
            if m in own.get(n, ()) or n in own.get(m, ()): continue
            if hit(mv[n], mv[m]) > 0.5: bad += 1; print(f"   CLASH {n} vs {m} at x{dx} y{dy}")
print(f"   servo a swings {span['a'][0]:+.0f} to {span['a'][1]:+.0f} deg, servo b {span['b'][0]:+.0f} to {span['b'][1]:+.0f} deg for +-15 of fan")
ok &= bad == 0

print("3) rocket leaning on the stand")
ang = math.radians(S.ARM_DEG)
px, py = S.r_s * math.cos(ang), S.r_s * math.sin(ang)
post = v(S.post_low).translate((px, py, S.bot_z)).fuse(v(S.post_top).translate((px, py, S.bot_z + S.seg)))
rocket = [v(p).translate((0, 0, -M.PIVOT_Z)) for p in list(M.print_parts.values()) + [D.edf]]
limit = None
for d in (10, 15, 18, 20, 22, 25):
    hits = set()
    for k in range(8):
        a = math.radians(45 * k)
        ax = (math.cos(a), math.sin(a), 0)
        for r in rocket:
            q = r.rotate((0, 0, 0), ax, d)
            if hit(q, S.frame) > 0.5: hits.add("frame")
            if hit(q, post) > 0.5: hits.add("post")
    print(f"   {d:2d} deg: {sorted(hits) or 'clear'}")
    if hits and limit is None: limit = d
    if "post" in hits and d <= 20: ok = False

print("4) every print fits the A1 mini (180 x 180 x 180)")
prints = dict(M.print_parts)
prints.update({f"stand_{n}": getattr(S, n) for n in ("collar", "yoke", "frame", "post_low", "post_top", "base")})
for n, p in prints.items():
    bb = v(p).BoundingBox()
    dims = sorted([bb.xlen, bb.ylen, bb.zlen])
    fits = dims[2] <= 180 or (dims[1] <= 180 and math.hypot(dims[2], dims[1]) <= 250 and dims[0] <= 180)
    tag = "ok" if dims[2] <= 180 else ("ok diagonal" if fits else "TOO BIG")
    if not fits: ok = False
    print(f"   {n:16s} {bb.xlen:5.0f} x {bb.ylen:5.0f} x {bb.zlen:5.0f}  {tag}")

print("5) steepest brace overhang")
chord = 2 * M.STRUT_R * math.sin(math.pi / M.N_STRUT)
for n, (z0, z1) in (("servo bay", (M.DECK[1] + M.RING_T, M.SLED[0] - M.RING_T)), ("avionics bay", (M.SLED[1] + M.RING_T, M.TOPRING[0]))):
    a = math.degrees(math.atan2(z1 - z0, chord))
    print(f"   {n}: braces at {a:.0f} deg from flat ({'ok' if a >= 45 else 'needs support'})")
    if a < 45: ok = False
print("ALL GOOD" if ok else "PROBLEMS ABOVE")
