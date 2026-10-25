"""Core: part registry + geometry helpers. Units mm / kg / EUR. World: X forward, Y left, Z up, ground z=0.
Every part is built directly in its assembled (world) position. Right side = -Y, left = +Y."""
import math, json
import numpy as np
from build123d import (Solid, Compound, Cylinder, Box, Plane, Location, Vector, Pos, Rot, Wire, Face,
                       Axis, Polygon, extrude, Align, Cone, Sphere, Torus, export_step, export_stl, Mode)

V = lambda x, y=None, z=None: Vector(*x) if y is None else Vector(x, y, z)
DENS = {"steel": 7.85e-6, "S355": 7.85e-6, "4130": 7.85e-6, "25CrMo4": 7.85e-6, "alu": 2.70e-6, "6061": 2.70e-6, "5052": 2.68e-6,
        "PETG": 1.27e-6, "PLA": 1.24e-6, "PC": 1.20e-6, "ABS": 1.05e-6, "POM": 1.41e-6, "HDPE": 0.95e-6,
        "coroplast": 0.22e-6, "foam": 0.05e-6, "rubber": 1.2e-6, "bronze": 8.8e-6, "brass": 8.5e-6, "nylon": 1.14e-6}  # kg/mm3
# tube stock price estimate EUR/m (cut-to-length, Spain) - ESTIMATES
TUBE_PRICE = {("steel", 25): 7.0, ("steel", 30): 9.0, ("steel", 20): 5.5, ("steel", 16): 5.0, ("steel", 12): 4.0}

class Part:
    def __init__(s, no, name, sub, **kw):
        s.no, s.name, s.sub = no, name, sub
        s.material = kw.get("material", "")
        s.stock = kw.get("stock", "")
        s.method = kw.get("method", "buy")          # cut / bend / weld / buy / print / laser / turn / drill
        s.supplier = kw.get("supplier", "")
        s.cost = kw.get("cost", 0.0)                # EUR each, ESTIMATE
        s.mass = kw.get("mass")                      # kg each (None -> from volume)
        s.status = kw.get("status", "")              # "" or "TO CONFIRM"
        s.notes = kw.get("notes", "")
        s.color = kw.get("color", (0.6, 0.6, 0.65))
        s.kind = kw.get("kind", "bought")            # tube / plate / bent_tube / sheet / turned / panel / bought / fastener / print
        s.draw = kw.get("draw", {})                  # data for 2D drawing
        s.fab = kw.get("fab", False)                 # fabricated by me -> needs drawing
        s.density = kw.get("density")                # kg/mm3 override
        s.insts = []
class Inst:
    def __init__(s, part, solid, sub, label, axis=None, meta=None):
        s.part, s.solid, s.sub, s.label, s.axis, s.meta = part, solid, sub, label, axis, meta or {}

class Registry:
    def __init__(s):
        s.parts, s.insts, s.fast_nos, s.log = {}, [], {}, []
    def part(s, no, name, sub, **kw):
        if no in s.parts: return s.parts[no]
        p = Part(no, name, sub, **kw); s.parts[no] = p; return p
    def add(s, no, solid, sub=None, label=None, **kw):
        """add an instance of part `no` (must exist, or give name+sub+kw to create)."""
        if no not in s.parts: s.part(no, kw.pop("name", no), sub or "", **kw)
        p = s.parts[no]
        i = Inst(p, solid, sub or p.sub, label or f"{no}-{len(p.insts)+1}")
        p.insts.append(i); s.insts.append(i); return i
    # ---- fasteners numbered TR-9xx
    def fastener_no(s, key, name, **kw):
        if key not in s.fast_nos:
            s.fast_nos[key] = f"TR-{901+len(s.fast_nos)}"
            s.part(s.fast_nos[key], name, "Fasteners", kind="fastener", **kw)
        return s.fast_nos[key]

# ---------------- geometry helpers
def frame_at(p, d):
    return Plane(origin=tuple(p), z_dir=tuple(d))
def cyl(p, q, r, ri=0.0):
    p, q = V(p), V(q); L = (q - p).length
    c = frame_at(p, q - p).location * Pos(0, 0, L / 2) * Cylinder(r, L)
    if ri > 0: c = c - frame_at(p, q - p).location * Pos(0, 0, L / 2) * Cylinder(ri, L + 2)
    return c
def solid_of(sh):
    """first solid of a possibly compound result"""
    ss = sh.solids() if hasattr(sh, "solids") else [sh]
    return ss[0] if len(ss) == 1 else Compound(list(ss))
def vol(sh): return sum(s.volume for s in sh.solids())
def box_at(c, size, rot=(0, 0, 0)):
    return Pos(*c) * Rot(*rot) * Box(*size)

def cut_cyl(sh, p, d, dia, length, back=0.0):
    """drill: remove cylinder starting `back` behind p along d"""
    d = V(d).normalized(); p0 = V(p) - d * back
    c = cyl(p0, p0 + d * (length + back), dia / 2)
    return sh - c

def tube_axis_len(p, q): return (V(q) - V(p)).length

def tube_solid(p, q, od, wall):
    return cyl(p, q, od / 2, od / 2 - wall)

def cope_cut(tube_full, p, q, end, mains):
    """cope the `end` ('p'|'q') of a tube against solid main tubes (list of (p,q,od)). tube_full is hollow solid."""
    res = tube_full
    for (mp, mq, mod) in mains:
        res = res - cyl(mp, mq, mod / 2)
    return res

def mitre_cut(sh, node, n, keep_side):
    """cut with plane through node with normal n, remove half-space on +n side"""
    n = V(n).normalized()
    big = Plane(origin=tuple(V(node)), z_dir=tuple(n)).location * Pos(0, 0, 500) * Box(2000, 2000, 1000)
    return sh - big

def reg_tube(reg, no, name, sub, p, q, od, wall, mains_p=(), mains_q=(), mitre_p=None, mitre_q=None,
             material="S355", method="cut+weld", supplier="steel tube stockist", price_m=None, color=(0.55, 0.57, 0.62),
             holes=(), label=None, notes="", fab=True):
    """Register one tube instance. mains_*: list of main tubes [(p,q,od)] the end is coped onto.
    mitre_*: (n, other_tube_id) plane normal to cut with. Records everything for cut list / drawing."""
    p, q = V(p), V(q)
    s = tube_solid(p, q, od, wall)
    ends = {}
    for tag, mains, mit, pt in (("p", mains_p, mitre_p, p), ("q", mains_q, mitre_q, q)):
        if mains:
            s = cope_cut(s, p, q, tag, mains)
            ends[tag] = dict(kind="cope", on=[m[2] for m in mains])
        elif mit is not None:
            s = mitre_cut(s, pt, mit, None)
            ends[tag] = dict(kind="mitre")
        else:
            ends[tag] = dict(kind="square")
    ax = (q - p).normalized()
    for (tpos, ang, dia) in holes:     # hole through both walls at axial position tpos, angle (deg about axis)
        perp = Plane(origin=tuple(p), z_dir=tuple(ax)).x_dir
        perp = Rot(0, 0, 0) and perp
        rot_dir = Plane(origin=tuple(p), z_dir=tuple(ax))
        dvec = (rot_dir.x_dir * math.cos(math.radians(ang)) + rot_dir.y_dir * math.sin(math.radians(ang)))
        c = p + ax * tpos
        s = s - cyl(c - dvec * (od / 2 + 2), c + dvec * (od / 2 + 2), dia / 2)
    # cut length = extent along the axis of the final solid, tip to tip
    bb_pts = [Vector(v.X, v.Y, v.Z) for v in s.vertices()]
    ts = [(v - p).dot(ax) for v in bb_pts] or [0, (q - p).length]
    cut_len = max(ts) - min(ts)
    if no not in reg.parts:
        key = ("steel", int(od))
        pm = price_m if price_m is not None else TUBE_PRICE.get(key, 6.0)
        reg.part(no, name, sub, material=material, stock=f"{od:g}x{wall:g} round tube", method=method, supplier=supplier,
                 cost=round(pm * (cut_len / 1000) * 1.1 + 1.0, 2), kind="tube", color=color, fab=fab, notes=notes,
                 draw=dict(type="tube", od=od, wall=wall, length=round(cut_len, 1), ends=ends, holes=list(holes), axis_len=round((q - p).length, 1)))
    inst = reg.add(no, s, sub, label, axis=(tuple(p), tuple(q)))
    inst.meta.update(p=tuple(p), q=tuple(q), od=od, wall=wall, cut_len=cut_len, ends=ends)
    return inst

def drill_inst(inst, p, d, dia, length, back=0.0):
    """cut a hole in an instance (only if cylinder touches it)"""
    d = V(d).normalized(); p0 = V(p) - d * back
    c = cyl(p0, p0 + d * (length + back), dia / 2)
    new = inst.solid - c
    inst.solid = new

# ---------------- plates / sheets
def plate_solid(outline, holes, thk, origin, xdir, normal, slots=()):
    """outline: list of (u,v) mm; holes: list of (u,v,dia); slots: (u,v,length,width,angle_deg).
    Plate lies in plane through `origin` spanned by xdir & (normal x xdir); extruded `thk` along +normal."""
    pl = Plane(origin=tuple(V(origin)), x_dir=tuple(V(xdir)), z_dir=tuple(V(normal)))
    face = Face(Wire.make_polygon([Vector(u, v, 0) for (u, v) in outline], close=True))
    s = pl.location * Solid.extrude(face, Vector(0, 0, thk))
    for (u, v, d) in holes:
        s = s - pl.location * Pos(u, v, thk / 2) * Cylinder(d / 2, thk + 2)
    for (u, v, L, W, a) in slots:
        sl = pl.location * Pos(u, v, thk / 2) * Rot(0, 0, a) * (Box(L - W, W, thk + 2) + Pos((L - W) / 2, 0, 0) * Cylinder(W / 2, thk + 2) + Pos(-(L - W) / 2, 0, 0) * Cylinder(W / 2, thk + 2))
        s = s - sl
    return s

def reg_plate(reg, no, name, sub, outline, holes, thk, origin, xdir, normal, material="S355", method="laser+weld",
              supplier="laser cutting service", cost=None, color=(0.7, 0.55, 0.3), slots=(), label=None, notes="", fab=True, bends=None):
    s = plate_solid(outline, holes, thk, origin, xdir, normal, slots)
    xs = [u for u, v in outline]; ys = [v for u, v in outline]
    if no not in reg.parts:
        area = abs(sum(outline[i][0] * outline[(i + 1) % len(outline)][1] - outline[(i + 1) % len(outline)][0] * outline[i][1] for i in range(len(outline)))) / 2
        reg.part(no, name, sub, material=material, stock=f"{thk:g} mm {material} sheet", method=method, supplier=supplier,
                 cost=cost if cost is not None else round(2.0 + area / 1e4 * 0.8 * thk / 3, 2), kind="plate", color=color, fab=fab, notes=notes,
                 draw=dict(type="plate", outline=outline, holes=holes, slots=list(slots), thk=thk, bbox=(max(xs) - min(xs), max(ys) - min(ys)), bends=bends))
    return reg.add(no, s, sub, label)

# ---------------- turned parts: profile = list of (radius, z) along axis, closed through axis
def turned_solid(profile, p, d):
    pts = [Vector(r, 0, z) for r, z in profile]
    pts = [Vector(0, 0, profile[0][1])] + pts + [Vector(0, 0, profile[-1][1])]
    face = Face(Wire.make_polygon(pts, close=True))
    s = Solid.revolve(face, 360, Axis.Z)
    return frame_at(p, d).location * s

def reg_turned(reg, no, name, sub, profile, p, d, material="S355", stock="", method="turn", supplier="own lathe / machine shop",
               cost=4.0, color=(0.75, 0.75, 0.2), label=None, bore=None, notes="", fab=True):
    s = turned_solid(profile, p, d)
    if bore:
        z0 = profile[0][1]; z1 = profile[-1][1]
        s = s - frame_at(p, d).location * Pos(0, 0, (z0 + z1) / 2) * Cylinder(bore / 2, z1 - z0 + 2)
    if no not in reg.parts:
        reg.part(no, name, sub, material=material, stock=stock, method=method, supplier=supplier, cost=cost, kind="turned", color=color,
                 fab=fab, notes=notes, draw=dict(type="turned", profile=profile, bore=bore))
    return reg.add(no, s, sub, label, axis=(tuple(V(p)), tuple(V(p) + V(d).normalized() * 100)))

# ---------------- bent sheet (profile polyline offset to thickness, extruded along width)
def offset_polyline(pts, thk):
    P = np.array(pts, float); n = len(P)
    nr = []
    for i in range(n - 1):
        t = P[i + 1] - P[i]; t /= np.linalg.norm(t); nr.append(np.array([-t[1], t[0]]))
    L, R = [], []
    for i in range(n):
        if i == 0: m = nr[0]; k = 1
        elif i == n - 1: m = nr[-1]; k = 1
        else:
            m = nr[i - 1] + nr[i]; m /= np.linalg.norm(m); k = 1 / max(0.3, float(np.dot(m, nr[i])))
        L.append(P[i] + m * thk / 2 * k); R.append(P[i] - m * thk / 2 * k)
    return [tuple(x) for x in L + R[::-1]]

def reg_sheet_bent(reg, no, name, sub, profile, width, thk, origin, uvec, vvec, wvec, material="5052", holes=(), method="laser+bend",
                   supplier="laser+press brake service", cost=6.0, color=(0.75, 0.75, 0.8), bend_r=2.0, label=None, notes="", fab=True):
    """profile: centreline polyline in (u,v) in the plane spanned by uvec,vvec, extruded `width` along wvec from origin.
    holes: (3D point, dir, dia) list cut through."""
    poly = offset_polyline(profile, thk)
    pl = Plane(origin=tuple(V(origin)), x_dir=tuple(V(uvec)), z_dir=tuple(V(wvec)))
    face = Face(Wire.make_polygon([Vector(u, v, 0) for u, v in poly], close=True))
    s = pl.location * Solid.extrude(face, Vector(0, 0, width))
    for (hp, hd, dia) in holes:
        s = cut_cyl(s, V(hp) - V(hd).normalized() * 40, hd, dia, 80)
    # flat pattern length: straight lengths + bend allowance (K=0.33)
    P = np.array(profile, float); segs = [float(np.linalg.norm(P[i + 1] - P[i])) for i in range(len(P) - 1)]
    angs = []
    for i in range(1, len(P) - 1):
        a = P[i] - P[i - 1]; b = P[i + 1] - P[i]
        angs.append(math.degrees(math.acos(max(-1, min(1, np.dot(a, b) / np.linalg.norm(a) / np.linalg.norm(b))))))
    K = 0.33; flat_len = sum(segs) - sum((bend_r + thk) * math.tan(math.radians(a) / 2) for a in angs) + sum(math.radians(a) * (bend_r + K * thk) for a in angs)
    if no not in reg.parts:
        reg.part(no, name, sub, material=material, stock=f"{thk:g} mm {material} sheet", method=method, supplier=supplier, cost=cost, kind="sheet",
                 color=color, fab=fab, notes=notes, draw=dict(type="sheet", profile=profile, width=width, thk=thk, bend_r=bend_r, segs=segs, angs=angs, flat_len=flat_len,
                                                            holes=[(tuple(V(h[0])), h[2]) for h in holes]))
    return reg.add(no, s, sub, label)

# ---------------- flat panels (coroplast / polycarbonate), polygon in 3D (planar-ish)
def reg_panel(reg, no, name, sub, pts3d, thk, material="coroplast", cost=None, color=(0.9, 0.9, 0.95), label=None, notes="", fab=True,
              holes3d=(), method="cut (knife / CNC / laser)", supplier="plastics sheet stockist", alpha=1.0, stock=None):
    pts = [V(p) for p in pts3d]
    # plane from Newell normal
    n = Vector(0, 0, 0)
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        n += Vector((a.Y - b.Y) * (a.Z + b.Z), (a.Z - b.Z) * (a.X + b.X), (a.X - b.X) * (a.Y + b.Y))
    n = n.normalized(); c = sum(pts, Vector(0, 0, 0)) * (1 / len(pts))
    pl = Plane(origin=tuple(c), z_dir=tuple(n))
    loc = [(float((p - c).dot(pl.x_dir)), float((p - c).dot(pl.y_dir))) for p in pts]
    dev = max(abs(float((p - c).dot(n))) for p in pts)     # non-planarity
    face = Face(Wire.make_polygon([Vector(u, v, 0) for u, v in loc], close=True))
    s = pl.location * Pos(0, 0, -thk / 2) * Solid.extrude(face, Vector(0, 0, thk))
    for (hp, dia) in holes3d:
        s = cut_cyl(s, V(hp) - n * 20, n, dia, 40)
    if no not in reg.parts:
        xs = [u for u, v in loc]; ys = [v for u, v in loc]
        area = abs(sum(loc[i][0] * loc[(i + 1) % len(loc)][1] - loc[(i + 1) % len(loc)][0] * loc[i][1] for i in range(len(loc)))) / 2
        price = {"coroplast": 4.0, "PC": 38.0, "ABS": 25.0, "alu": 30.0}.get(material, 20.0) * area / 1e6 * (thk / (4 if material == "coroplast" else 2))
        reg.part(no, name, sub, material=material, stock=stock or f"{thk:g} mm {material} sheet", method=method, supplier=supplier,
                 cost=cost if cost is not None else round(max(price * 1.3, 2.0), 2), kind="panel", color=color, fab=fab, notes=notes,
                 draw=dict(type="panel", outline=loc, thk=thk, bbox=(max(xs) - min(xs), max(ys) - min(ys)), nonplanar=dev,
                           holes=[(tuple(V(h[0])), h[1]) for h in holes3d]))
        reg.parts[no].color = color + (alpha,) if len(color) == 3 else color
    return reg.add(no, s, sub, label)

# ---------------- fasteners
STD_LEN = [8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 180, 200, 220, 240, 260, 280, 300]
HEX = {5: (8, 3.5, 4.7), 6: (10, 4.0, 5.2), 8: (13, 5.3, 6.8), 10: (16, 6.4, 8.4), 12: (18, 7.5, 10.8)}   # M: (AF, head k, nut m)
WASH = {5: (10, 1.0), 6: (12, 1.6), 8: (16, 1.6), 10: (20, 2.0), 12: (24, 2.5)}                          # OD, thk (DIN 125 approx)
BOLT_PRICE = {5: 0.08, 6: 0.10, 8: 0.15, 10: 0.25, 12: 0.40}

def hexprism(af, h):
    return Cylinder(af / math.sqrt(3), h, arc_size=360) if False else Solid.extrude(Face(Wire.make_polygon(
        [Vector(af / math.sqrt(3) * math.cos(math.radians(60 * i)), af / math.sqrt(3) * math.sin(math.radians(60 * i)), 0) for i in range(6)], close=True)), Vector(0, 0, h))

def bolt(reg, sub, p, d, M, grip, targets=(), nut=True, washers=(True, True), kind="hex", clearance=0.5, label=None, nyloc=True):
    """Bolt whose head seat is at point p, pointing along d (shank goes along d), clamping `grip` mm.
    Cuts clearance holes in `targets` (Inst list). Registers bolt, nut, washers under TR-9xx."""
    d = V(d).normalized(); p = V(p)
    af, k, m = HEX[M]; wod, wt = WASH[M]
    need = grip + (wt if washers[0] else 0) + (wt if washers[1] and nut else 0) + (m * 1.3 if nut else 2)
    L = next((l for l in STD_LEN if l >= need - 0.01), int(math.ceil(need)))
    for t in targets: drill_inst(t, p, d, M + clearance, grip, back=1.0)
    off = 0.0
    parts_added = []
    hw = None
    if kind == "hex":
        name = f"Hex bolt M{M}x{L} DIN 933 A2"
        key = ("bolt", M, L, kind)
    else:
        name = f"Button head screw M{M}x{L} ISO 7380"; key = ("bolt", M, L, kind)
    no = reg.fastener_no(key, name, material="stainless A2", stock=f"M{M}x{L}", supplier="hardware store", cost=round(BOLT_PRICE[M] * (1 + L / 60), 2),
                         mass=round(math.pi * (M / 2) ** 2 * L * 7.85e-6 + math.pi * (af / 2) ** 2 * k * 7.85e-6, 4))
    fr = frame_at(p, d).location
    head_z = -(wt if washers[0] else 0)
    body = fr * Pos(0, 0, head_z - k / 2 + 0.0) * Cylinder(af / 2 * 1.02, k) if kind != "hex" else fr * Pos(0, 0, head_z - k) * hexprism(af, k)
    shank = fr * Pos(0, 0, head_z + L / 2 - 0) * Cylinder(M / 2, L)
    bi = reg.add(no, body + shank, sub, label=label and label + "-bolt", axis=(tuple(p), tuple(p + d)))
    bi.meta["grip"] = grip
    if washers[0]:
        wno = reg.fastener_no(("washer", M), f"Flat washer M{M} DIN 125 A2", material="stainless A2", stock=f"M{M}", supplier="hardware store", cost=0.03,
                              mass=round(math.pi * ((wod / 2) ** 2 - ((M + 1) / 2) ** 2) * wt * 7.85e-6, 4))
        reg.add(wno, fr * Pos(0, 0, -wt / 2) * (Cylinder(wod / 2, wt) - Cylinder((M + 1) / 2, wt + 1)), sub)
    end = grip
    if washers[1] and nut:
        wno = reg.fastener_no(("washer", M), f"Flat washer M{M} DIN 125 A2")
        reg.add(wno, fr * Pos(0, 0, end + wt / 2) * (Cylinder(wod / 2, wt) - Cylinder((M + 1) / 2, wt + 1)), sub)
        end += wt
    if nut:
        nm = m * 1.3 if nyloc else m
        nno = reg.fastener_no(("nut", M, nyloc), f"{'Nyloc nut' if nyloc else 'Hex nut'} M{M} DIN {'985' if nyloc else '934'} A2", material="stainless A2", stock=f"M{M}", supplier="hardware store",
                              cost=round(BOLT_PRICE[M] * 0.8, 2), mass=round(math.pi * ((af / 2) ** 2 - (M / 2) ** 2) * nm * 7.85e-6, 4))
        reg.add(nno, fr * Pos(0, 0, end) * (hexprism(af, nm) - Cylinder(M / 2, 3 * nm)), sub)
    return bi

# ---------------- mass / bounding helpers
def inst_mass(i):
    p = i.part
    if p.mass is not None: return p.mass
    dens = p.density or DENS.get(p.material.split()[0] if p.material else "steel", 7.85e-6)
    ks = [k for k in DENS if k.lower() in p.material.lower()]
    if ks: dens = DENS[max(ks, key=len)]
    return vol(i.solid) * dens
def centroid(sh):
    c = sh.center(); return (c.X, c.Y, c.Z)

def export_all(reg, outdir):
    pass
