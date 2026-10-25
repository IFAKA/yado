"""Shared helpers for sub-assembly modules."""
import math
import numpy as np
from build123d import Pos, Rot, Cylinder, Box, Plane, Vector, Compound
from .core import *

def unit(v): v = np.array(v, float); return v / np.linalg.norm(v)

def tube(reg, no, name, sub, p, q, od, wall, mains_p=(), mains_q=(), group="frame", **kw):
    """reg_tube + record joint angles. mains are (p,q,od)."""
    kw.setdefault("label", None)
    inst = reg_tube(reg, no, name, sub, p, q, od, wall, mains_p=mains_p, mains_q=mains_q, **kw)
    inst.meta["group"] = group
    ax = unit(np.array(q, float) - np.array(p, float))
    first = len(reg.parts[no].insts) == 1
    if first:
        for tag, mains in (("p", mains_p), ("q", mains_q)):
            if mains:
                angs = []
                for (mp, mq, mod) in mains:
                    m = unit(np.array(mq, float) - np.array(mp, float))
                    angs.append(round(math.degrees(math.acos(min(1, abs(float(np.dot(ax, m)))))), 1))
                reg.parts[no].draw["ends"][tag]["angle_deg"] = angs
                reg.parts[no].draw["ends"][tag]["main_od"] = [m[2] for m in mains]
    return inst

def plate_on(reg, plane, no, name, sub, outline, holes, thk, o, group=None, **kw):
    """plane: 'xy' (u=x,v=y, extrude +z) | 'xz' (u=x,v=z, extrude +y) | 'yz' (u=y,v=z, extrude +x).
    o = world offset of the plane's near face along the normal axis (coordinate value). outline/holes given in (u,v) world coords."""
    if plane == "xy": orig, xd, nm, f = (0, 0, o), (1, 0, 0), (0, 0, 1), lambda u, v: (u, v)
    elif plane == "xz": orig, xd, nm, f = (0, o + thk, 0), (1, 0, 0), (0, -1, 0), lambda u, v: (u, v)
    else: orig, xd, nm, f = (o, 0, 0), (0, 1, 0), (1, 0, 0), lambda u, v: (u, v)
    inst = reg_plate(reg, no, name, sub, outline, holes, thk, orig, xd, nm, **kw)
    if group: inst.meta["group"] = group
    inst.meta["plane"] = plane; inst.meta["o"] = o; inst.meta["thk"] = thk
    return inst

def grp(inst, g): inst.meta["group"] = g; return inst

def wheel_solid(c, od, width, rim_d_iso, hub_len, axis=(0, 1, 0), spokes=24, flange=None, bore_r=5.0):
    """simple wheel: tyre torus, rim ring, hub, spokes. c centre, axis along hub."""
    ax = V(axis).normalized()
    fr = frame_at(c, ax).location
    R_t = od / 2 - width / 2
    tyre = fr * Torus(R_t, width / 2)
    rim_r = rim_d_iso / 2 + 2
    rim = fr * (Cylinder(rim_r + 3, 22) - Cylinder(rim_r - 4, 24))
    hub = fr * (Cylinder(18, hub_len) - Cylinder(bore_r, hub_len + 2))
    fl = flange if flange is not None else hub_len / 2 - 3
    hub = hub + fr * Pos(0, 0, -fl) * Cylinder(33, 5) + fr * Pos(0, 0, fl) * Cylinder(33, 5)
    sp = []
    for i in range(spokes):
        a = 2 * math.pi * i / spokes
        for sgn in (-1, 1):
            p = fr * Pos(14 * math.cos(a), 14 * math.sin(a), sgn * fl)
            q = fr * Pos((rim_r - 4) * math.cos(a), (rim_r - 4) * math.sin(a), sgn * 2)
            sp.append(cyl(p.position, q.position, 1.0))
    return tyre, rim, hub, sp

def sweep_poly_boxes(pts, w, h, y):
    """chain-like strip along polyline pts[(x,z)], plane y."""
    out = []
    for i in range(len(pts) - 1):
        a, b = np.array(pts[i]), np.array(pts[i + 1]); d = b - a; L = np.linalg.norm(d)
        if L < 1e-6: continue
        ang = math.degrees(math.atan2(-d[1], d[0]))   # rotation about +y mapping x->dir (x,z)
        m = (a + b) / 2
        out.append(Pos(m[0], y, m[1]) * Rot(0, ang, 0) * Box(L + 0.5, w, h))
    return out

def buy(reg, no, name, sub, solid, material="", stock="", cost=0.0, mass=None, status="TO CONFIRM", supplier="bike shop / online", color=(0.3, 0.3, 0.33), label=None, notes="", group=None, kind="bought"):
    if no not in reg.parts:
        reg.part(no, name, sub, material=material, stock=stock, method="buy", supplier=supplier, cost=cost, mass=mass, status=status, color=color, kind=kind, notes=notes)
    i = reg.add(no, solid, sub, label)
    if group: i.meta["group"] = group
    return i

def nut_at(reg, sub, p, d, M, nyloc=False):
    d = V(d).normalized(); af, k, m = HEX[M]
    nm = m * 1.3 if nyloc else m
    no = reg.fastener_no(("nut", M, nyloc), f"{'Nyloc nut' if nyloc else 'Hex nut'} M{M} DIN {'985' if nyloc else '934'} A2", material="stainless A2", stock=f"M{M}", supplier="hardware store",
                         cost=round(BOLT_PRICE[M] * 0.8, 2), mass=round(math.pi * ((af / 2) ** 2 - (M / 2) ** 2) * nm * 7.85e-6, 4))
    return reg.add(no, frame_at(p, d).location * (hexprism(af, nm) - Cylinder(M / 2, 3 * nm)), sub)

def washer_at(reg, sub, p, d, M):
    d = V(d).normalized(); wod, wt = WASH[M]
    no = reg.fastener_no(("washer", M), f"Flat washer M{M} DIN 125 A2", material="stainless A2", stock=f"M{M}", supplier="hardware store", cost=0.03,
                         mass=round(math.pi * ((wod / 2) ** 2 - ((M + 1) / 2) ** 2) * wt * 7.85e-6, 4))
    return reg.add(no, frame_at(p, d).location * Pos(0, 0, wt / 2) * (Cylinder(wod / 2, wt) - Cylinder((M + 1) / 2, wt + 1)), sub)

def rod_end(reg, sub, c, shank_dir, axis_dir, group=None, label=None):
    """M8 male spherical rod end: ball centre c, eye axis axis_dir, shank along shank_dir."""
    sd, ad = V(shank_dir).normalized(), V(axis_dir).normalized()
    ring = frame_at(c, ad).location * (Cylinder(12, 9) + Cylinder(7.6, 12) - Cylinder(4.1, 14))
    shank = cyl(V(c) + sd * 11, V(c) + sd * 37, 4.0) + cyl(V(c) + sd * 8, V(c) + sd * 17, 6.0)
    return buy(reg, "TR-240", "Rod end M8 male, spherical (SI8 type)", sub, ring + shank, material="steel/PTFE", stock="M8x1.25 male, 8 mm bore", cost=3.2, mass=0.035,
               status="TO CONFIRM", color=(0.7, 0.7, 0.72), label=label, group=group)

def rod_link(reg, sub, a, b, axis_a, axis_b, name_no, name, label, group=None):
    """tube Ø14x3 tie rod between ball centres a,b with rod ends + jam nuts. returns length."""
    a, b = V(a), V(b); dv = (b - a); cc = dv.length; u = dv.normalized()
    rod_end(reg, sub, a, u, axis_a, group=group, label=label + "-REa")
    rod_end(reg, sub, b, -u, axis_b, group=group, label=label + "-REb")
    Lt = cc - 44
    t = cyl(a + u * 22, b - u * 22, 7.0, 4.0)
    ti = reg.add(name_no, t, sub, label) if name_no in reg.parts else reg.add(name_no, t, sub, label, name=name, material="S355", stock="14x3 tube, M8 tapped both ends",
                 method="cut+tap", supplier="steel tube stockist", cost=2.0, kind="rodtube", fab=True, color=(0.6, 0.6, 0.65),
                 draw=dict(type="tube", od=14, wall=3, length=round(Lt, 1), ends={"p": {"kind": "square", "note": "tap M8 x 20 deep"}, "q": {"kind": "square", "note": "tap M8 x 20 deep"}}, holes=[], axis_len=round(Lt, 1)))
    ti.meta["group"] = group; ti.meta["length"] = Lt
    for e, uu in ((a, u), (b, -u)):
        nut_at(reg, sub, e + uu * 24, uu, 8)
    return cc

def rivet(reg, sub, p, d, grip, dia=4.0):
    """blind rivet: head seated at p, shank along d through `grip`, tail 3 mm beyond."""
    d = V(d).normalized(); p = V(p)
    L = 6 if grip <= 3 else (8 if grip <= 5 else (10 if grip <= 7 else (12 if grip <= 9 else 16)))
    no = reg.fastener_no(("rivet", dia, L), f"Blind rivet Ø{dia:g}x{L} alu/steel", material="alu/steel", stock=f"Ø{dia:g}x{L}", supplier="hardware store", cost=0.04, mass=0.002)
    fr = frame_at(p, d).location
    return reg.add(no, fr * Pos(0, 0, -0.8) * Cylinder(dia * 1.0, 1.6) + fr * Pos(0, 0, (grip + 3) / 2) * Cylinder(dia / 2, grip + 3.0), sub)

def hole_ang(ax, dirw):
    """angle (deg) for reg_tube holes so the hole axis points along world dir `dirw`, for a tube with axis `ax`."""
    pl = Plane(origin=(0, 0, 0), z_dir=tuple(unit(ax))); dv = V(unit(dirw))
    return math.degrees(math.atan2(dv.dot(pl.y_dir), dv.dot(pl.x_dir)))
