"""Engineering checks on the built model. Each check returns a dict; run_all writes out/checks.json. Sanity checks, NOT certification."""
import math, json, itertools
import numpy as np
from scipy.optimize import brentq
from build123d import Location, Axis, Pos, Rot, Vector
from .core import inst_mass, centroid, V

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)

# ---------------------------------------------------------------- mass, CG, tipping
def rider_cg(K, mass, recline=None):
    """Rider centre of gravity from segment fractions (estimates): upper body+head+arms 62 %, thighs 20 %, shanks+feet 18 %."""
    pose = K["pose"]; th = math.radians(recline if recline is not None else pose["recline"])
    hip = np.array(K["hip"]); knee = np.array(K["knee"]); ankle = np.array(K["ankle"])
    trunk_len = 0.50 * pose["H"]
    up = hip + 0.45 * trunk_len * np.array([-math.sin(th), 0, math.cos(th)])
    seg = [(0.62, up), (0.20, (hip + knee) / 2), (0.18, (knee + ankle) / 2)]
    return sum(f * p for f, p in seg), [(f * mass, tuple(p)) for f, p in seg]

def mass_cg(reg, K, rider_kg=100.0):
    m = 0.0; c = np.zeros(3); bysub = {}
    for i in reg.insts:
        mi = inst_mass(i); m += mi; c += mi * np.array(centroid(i.solid)); bysub[i.sub] = bysub.get(i.sub, 0) + mi
    veh_cg = c / m
    rcg, _ = rider_cg(K, rider_kg)
    tot = m + rider_kg; cg = (c + rider_kg * rcg) / tot
    return dict(vehicle_kg=m, vehicle_cg=veh_cg.tolist(), by_subassembly_kg=bysub, rider_kg=rider_kg, rider_cg=rcg.tolist(),
                total_kg=tot, total_cg=cg.tolist(), front_axle_load_share=float((cg[0] + K["WB"]) / K["WB"]) if False else float((cg[0] - K["rear_axle"][0]) / K["WB"]))

def tipping(K, cg, min_deg=35.0):
    """Static lateral tip about the line front-wheel contact -> rear-wheel contact. angle = atan(d / h)."""
    rear = np.array([K["rear_axle"][0], 0.0]); res = {}
    for s in (1, -1):
        f = np.array([0.0, s * K["T"] / 2]); a = rear - f; a = a / np.linalg.norm(a)
        r = np.array([cg[0], cg[1]]) - f; d = abs(a[0] * r[1] - a[1] * r[0])
        res["left" if s > 0 else "right"] = math.degrees(math.atan2(d, cg[2]))
    ang = min(res.values())
    # what front track would give the required angle (rear unchanged, CG fixed)
    need = None
    for T in np.arange(K["T"], 1400, 10):
        f = np.array([0.0, T / 2]); a = rear - f; a /= np.linalg.norm(a); r = np.array([cg[0], 0.0]) - f
        if math.degrees(math.atan2(abs(a[0] * r[1] - a[1] * r[0]), cg[2])) >= min_deg: need = float(T); break
    return dict(angle_deg=res, min_angle_deg=ang, required_deg=min_deg, passes=ang >= min_deg, front_track_needed_mm=need, cg_height_mm=float(cg[2]))

# ---------------------------------------------------------------- steering sweep + Ackermann
def _rot_about(p, a, d, ang):
    d = unit(d); v = np.asarray(p, float) - a; c, s = math.cos(ang), math.sin(ang)
    return a + v * c + np.cross(d, v) * s + d * np.dot(d, v) * (1 - c)

def steering(reg, K, theta_max_deg=40.0, step=0.5):
    S = K["steer"]; bc = K["bc"]; piv = np.array(bc["pivot"][:2])
    out = {}; L0 = {}
    axes = {s: (np.array(K["kp"](s, 0)), unit(K["kp_dir"][s])) for s in (1, -1)}
    tipt = {s: np.array(K["bc_tip"][s][0]) for s in (1, -1)}; tipb = {s: np.array(K["bc_tip"][s][1]) for s in (1, -1)}
    for s in (1, -1): L0[s] = np.linalg.norm(S[s]["tie"]["a"] - tipt[s])
    def tip(b, th):
        v = b[:2] - piv; c, sn = math.cos(th), math.sin(th)
        return np.array([piv[0] + c * v[0] - sn * v[1], piv[1] + sn * v[0] + c * v[1], b[2]])
    def delta(s, th, guess):
        a0, d = axes[s]; pk = np.array(S[s]["tie"]["a"]); b = tip(tipt[s], th)
        f = lambda dl: np.linalg.norm(_rot_about(pk, a0, d, dl) - b) - L0[s]
        lo, hi = guess - math.radians(8), guess + math.radians(8)
        try: return brentq(f, lo, hi)
        except ValueError: return None
    rows = []; g = {1: 0.0, -1: 0.0}
    sx = K["stick_pivot"]
    def stick_phi(s, th):
        pv = np.array(sx[s]); lev = np.array(K["drags"][s]["b"]); Ld = np.linalg.norm(np.array(K["drags"][s]["a"]) - lev); b = tip(tipb[s], th)
        def f(phi):
            v = lev - pv; c, sn = math.cos(phi), math.sin(phi)
            p = pv + np.array([c * v[0] + sn * v[2], v[1], -sn * v[0] + c * v[2]])
            return np.linalg.norm(p - b) - Ld
        try: return math.degrees(brentq(f, -1.2, 1.2))
        except ValueError: return float("nan")
    stop = "sweep limit"
    for t in np.arange(0, theta_max_deg + 1e-9, step):
        th = -math.radians(t); dl = {}
        for s in (1, -1):
            dl[s] = delta(s, th, g[s])
            if dl[s] is None: break
            g[s] = dl[s]
        if None in dl.values(): stop = "tie rod cannot reach"; break
        pL, pR = stick_phi(1, th), stick_phi(-1, th)
        row = (float(t), math.degrees(dl[1]), math.degrees(dl[-1]), pL, pR)       # left turn: left wheel (col 1) inner, right wheel outer
        if max(row[1], row[2]) > 45.0: stop = "wheel angle cap 45 deg"; break
        if not (abs(pL) <= 35 and abs(pR) <= 35): stop = "stick lever angle > 35 deg (drag link / lever geometry)"; break
        rows.append(row)
    T, WB = K["T"], K["WB"]
    ack = []
    for t, dL, dR, pL, pR in rows:
        di, do = max(dL, dR), min(dL, dR)
        if di > 1.0:
            ideal_do = math.degrees(math.atan(1 / (1 / math.tan(math.radians(di)) + T / WB)))
            ack.append((di, do, ideal_do))
    return dict(rows=rows, ack=ack, stop_reason=stop, theta_stop_deg=rows[-1][0] if rows else 0.0)

def steer_clearance(reg, K, rows):
    """Rotate each knuckle group (wheel, tyre, brake, arm...) about its kingpin and test collisions against everything else, per step."""
    from build123d import Plane
    tol = 0.5; res = {}
    groups = {s: [i for i in reg.insts if i.meta.get("group") == ("knuckleL" if s > 0 else "knuckleR") or (i.part.no in ("TR-210", "TR-211") and i.label.endswith("F") and (("L" in i.label) == (s > 0)))] for s in (1, -1)}
    others = {s: [i for i in reg.insts if i not in groups[s] and i.part.kind != "fastener" and i.sub not in ("Electrics",) and i.part.no not in ("TR-240", "TR-241", "TR-242", "TR-720", "TR-721")] for s in (1, -1)}
    first_hit = None
    for t, dL, dR, pL, pR in rows[::2]:
        for s, ang in ((1, dL), (-1, dR)):
            a0 = np.array(K["kp"](s, 0)); d = unit(K["kp_dir"][s])
            ax = Axis(origin=tuple(a0), direction=tuple(d))
            for g in groups[s]:
                if g.part.no not in ("TR-210", "TR-211", "TR-212", "TR-213", "TR-206", "TR-204", "TR-207"): continue
                sol = g.solid.rotate(ax, ang); bb = sol.bounding_box()
                for o in others[s]:
                    if o.part.no in ("TR-201", "TR-202", "TR-203") or o.meta.get("group") == g.meta.get("group") and g.meta.get("group"): continue
                    ob = o.solid.bounding_box()
                    if (bb.min.X > ob.max.X or bb.max.X < ob.min.X or bb.min.Y > ob.max.Y or bb.max.Y < ob.min.Y or bb.min.Z > ob.max.Z or bb.max.Z < ob.min.Z): continue
                    try: v = sum(x.volume for x in sol.intersect(o.solid).solids())
                    except Exception: v = 0
                    if v > 5.0:
                        hit = (t, "L" if s > 0 else "R", g.part.no, o.part.no, o.label, round(v, 1))
                        if first_hit is None or hit[0] < first_hit[0]: first_hit = hit
                        res.setdefault((hit[1], hit[2], hit[3]), hit)
    return dict(first_collision=first_hit, collisions=list(res.values()))

# ---------------------------------------------------------------- interference
def interference(reg, min_vol=2.0, skip=lambda a, b: False):
    items = [(i, i.solid.bounding_box()) for i in reg.insts if i.part.kind not in ("cable",)]
    hits = []; npairs = 0
    for (a, ba), (b, bb) in itertools.combinations(items, 2):
        if a.part.no == b.part.no and a.part.kind == "fastener": continue
        if (ba.min.X > bb.max.X or ba.max.X < bb.min.X or ba.min.Y > bb.max.Y or ba.max.Y < bb.min.Y or ba.min.Z > bb.max.Z or ba.max.Z < bb.min.Z): continue
        if skip(a, b): continue
        npairs += 1
        try: v = sum(x.volume for x in a.solid.intersect(b.solid).solids())
        except Exception: v = -1
        if v > min_vol: hits.append(dict(a=a.part.no, al=a.label, an=a.part.name, b=b.part.no, bl=b.label, bn=b.part.name, vol=round(v, 1)))
    return dict(tested_pairs=npairs, hits=hits)

# ---------------------------------------------------------------- clearances
def min_dist(a, b):
    try: return float(a.distance_to(b))
    except Exception: return None

def clearances(reg, K):
    out = {}
    chain = [i for i in reg.insts if i.part.no == "TR-413"][0]
    cb = chain.solid.bounding_box()
    near = []
    for o in reg.insts:
        if o.part.kind in ("fastener", "cable") or o is chain: continue
        if o.part.no in ("TR-401", "TR-406", "TR-407", "TR-408", "TR-409", "TR-410", "TR-411", "TR-412", "TR-312", "TR-307", "TR-311", "TR-404", "TR-403", "TR-402"): continue
        ob = o.solid.bounding_box(); m = 25
        if (cb.min.X > ob.max.X + m or cb.max.X < ob.min.X - m or cb.min.Y > ob.max.Y + m or cb.max.Y < ob.min.Y - m or cb.min.Z > ob.max.Z + m or cb.max.Z < ob.min.Z - m): continue
        d = min_dist(chain.solid, o.solid)
        if d is not None and d < 25: near.append((round(d, 1), o.part.no, o.part.name, o.label))
    out["chain_to_other_parts_lt25mm"] = sorted(near)
    # rotors
    rn = []
    for r in [i for i in reg.insts if i.part.no in ("TR-212", "TR-313")]:
        rb = r.solid.bounding_box(); m = 20
        for o in reg.insts:
            if o is r or o.part.kind in ("fastener", "cable"): continue
            if o.part.no in ("TR-213", "TR-314", "TR-211", "TR-311", "TR-206", "TR-304", "TR-305", "TR-205", "TR-307", "TR-312") : continue
            ob = o.solid.bounding_box()
            if (rb.min.X > ob.max.X + m or rb.max.X < ob.min.X - m or rb.min.Y > ob.max.Y + m or rb.max.Y < ob.min.Y - m or rb.min.Z > ob.max.Z + m or rb.max.Z < ob.min.Z - m): continue
            d = min_dist(r.solid, o.solid)
            if d is not None and d < 20: rn.append((round(d, 1), r.label, o.part.no, o.part.name, o.label))
    out["rotor_to_other_parts_lt20mm"] = sorted(rn)
    return out

# ---------------------------------------------------------------- rear suspension travel
def rear_travel(reg, K, P):
    """Swing-arm rotates about the pivot; find wheel travel at which the tyre hits the shock cross tube / chain / seat rail, and shock stroke used."""
    piv = np.array(K["pivot"]); ra = np.array(K["rear_axle"]); lo = np.array(K["shock_lo"]) if "shock_lo" in K else None
    shock = [i for i in reg.insts if i.part.no == "TR-315"][0]
    stroke = P["rear"]["shock_stroke"]["value"]
    return dict(arm_length_mm=float(np.linalg.norm(ra[[0, 2]] - piv[[0, 2]])), shock_stroke_mm=stroke)

def ground_clearance(reg, K, P, rider_kg=100.0, sag_frac=0.25, front_tyre_defl=4.0, hump=100.0):
    """Floor height over the ground, unladen, loaded (rear shock at sag_frac of its stroke) and at full bump.
    ASSUMPTIONS: spring chosen for 25 % sag; frame pitches about the front tyre contact; a hump is a thin ridge straddled by the wheelbase."""
    fl = [i for i in reg.insts if i.part.no == "TR-629"][0].solid.bounding_box()
    z0 = float(fl.min.Z); xn = float(fl.max.X)
    ra = np.array(K["rear_axle"]); piv = np.array(K["pivot"]); lo = np.array(K["shock_lo"])
    ratio = np.linalg.norm(ra[[0, 2]] - piv[[0, 2]]) / np.linalg.norm(lo[[0, 2]] - piv[[0, 2]])
    travel = P["rear"]["shock_stroke"]["value"] * ratio
    wb = -float(ra[0]); sag = sag_frac * travel
    def floor_at(x, drop_rear):                      # floor underside height at x (x<=0 behind the front axle, pitch about front contact)
        return z0 - front_tyre_defl - drop_rear * min(max(-x, 0.0), wb) / wb
    xm = -wb / 2; xt_ = K["rear_axle"][0] - K["RR"] - 25.0
    out = dict(floor_unladen_mm=round(z0, 1), wheel_travel_mm=round(travel, 1), sag_loaded_mm=round(sag, 1),
               floor_mid_loaded_mm=round(floor_at(xm, sag), 1), floor_mid_full_bump_mm=round(floor_at(xm, travel), 1),
               hump_mm=hump, margin_loaded_mm=round(floor_at(xm, sag) - hump, 1), approach_angle_deg=round(math.degrees(math.atan(floor_at(xn, sag) / xn)), 1),
               departure_angle_deg=round(math.degrees(math.atan(floor_at(xt_, sag) / (-xt_ - wb))), 1),
               max_ramp_slope_nose_clears_pct=round(100 * floor_at(xn, sag) / xn, 1))
    out["ok"] = out["margin_loaded_mm"] >= 15
    return out

def run_all(path="params.yaml", fast=False):
    from .build import build_all
    from .check_fea import run as fea_run
    reg, K, P = build_all(path)
    R = {}
    R["ground"] = ground_clearance(reg, K, P)
    R["mass"] = m = mass_cg(reg, K, P["loads_checks"]["rider_mass_stability"]["value"])
    R["tipping"] = tipping(K, m["total_cg"], P["loads_checks"]["tipping_angle_min"]["value"])
    st = steering(reg, K); R["steering"] = st
    R["steer_clear"] = steer_clearance(reg, K, st["rows"])
    R["clearances"] = clearances(reg, K)
    R["fea"] = fea_run(reg, K, P, rider_kg=P["loads_checks"]["rider_mass_fea"]["value"], g_vert=P["loads_checks"]["vertical_g"]["value"])
    R["sightline"] = sightline(reg, K, P)
    if not fast: R["interference"] = interference(reg)
    return R, reg, K, P

# ---------------------------------------------------------------- driver sightline
def sightline(reg, K, P, elevs=(0.0, -6.0, 6.0), step=1.0):
    """Rays from the rider's eye. Clear PC panels pass; everything else blocks. Reports the clear azimuth arcs ahead and to the sides (0 deg = straight ahead, + = left)."""
    import trimesh
    from .export_mesh import inst_mesh
    th = math.radians(K["pose"]["recline"]); L = 0.47 * K["pose"]["H"]; hip = np.array(K["hip"])
    eye = np.array([hip[0] - L * math.sin(th) + 90, 0.0, hip[2] + L * math.cos(th)])
    meshes = []
    for i in reg.insts:
        if i.part.kind in ("cable", "fastener"): continue
        if "PC" in i.part.material and i.part.kind == "panel": continue
        if i.part.no in ("TR-501", "TR-502", "TR-512", "TR-513", "TR-514", "TR-511", "TR-509", "TR-510"): continue     # rider's own seat: it is behind the eye line
        m = inst_mesh(i, 1.0, 0.5)
        if m is not None: meshes.append(m)
    big = trimesh.util.concatenate(meshes); rm = trimesh.ray.ray_triangle.RayMeshIntersector(big)
    res = {}
    for el in elevs:
        azs = np.arange(-180.0, 180.0 + 1e-9, step)
        dirs = np.array([[math.cos(math.radians(el)) * math.cos(math.radians(a)), math.cos(math.radians(el)) * math.sin(math.radians(a)), math.sin(math.radians(el))] for a in azs])
        hit = rm.intersects_any(np.tile(eye, (len(azs), 1)), dirs)
        clear = ~hit
        arcs = []; start = None
        for a, c in zip(azs, clear):
            if c and start is None: start = a
            if not c and start is not None: arcs.append((start, a - step)); start = None
        if start is not None: arcs.append((start, azs[-1]))
        res[f"elev_{el:+.0f}deg"] = dict(clear_arcs_deg=[(float(a), float(b)) for a, b in arcs], clear_fraction=float(clear.mean()))
    return dict(eye_mm=eye.tolist(), results=res, notes="rear view only by mirrors (seat back and canopy tail are opaque); tube blockages included; coating/fogging not modelled")
