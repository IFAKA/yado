"""Export GLB (metres, Y-up) + manifest.json for the web viewer. three(x,y,z) = (X, Z, -Y)/1000."""
import json, sys, numpy as np, trimesh
from .build import build_all
from .core import inst_mass, centroid
from .export_mesh import inst_mesh

SUBS = ["Frame", "Steering & front axle", "Rear swing-arm & wheel", "Drivetrain", "Seat", "Canopy", "Electrics", "Fasteners"]

def conv(p): return [round(p[0] / 1000, 4), round(p[2] / 1000, 4), round(-p[1] / 1000, 4)]

def export(outdir="web", path="params.yaml", overrides=None, k_overrides=None):
    reg, K, P = build_all(path, overrides)
    sc = trimesh.Scene(); parts = []
    M = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float) / 1000.0
    for n, inst in enumerate(reg.insts):
        m = inst_mesh(inst, 0.5, 0.4)
        if m is None: continue
        m.vertices = np.asarray(m.vertices) @ M.T
        col = list(inst.part.color) + [1.0] * (4 - len(inst.part.color)); col = [int(255 * c) for c in col[:4]]
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(col, (len(m.faces), 1)))
        name = f"i{n}"
        sc.add_geometry(m, node_name=name, geom_name=name)
        p = inst.part
        parts.append(dict(id=name, no=p.no, label=inst.label, name=p.name, sub=inst.sub if inst.sub in SUBS else "Frame", material=p.material, stock=p.stock,
                          method=p.method, supplier=p.supplier, status=p.status, kind=p.kind, mass=round(inst_mass(inst), 3), cost=p.cost,
                          c=conv(centroid(inst.solid)), alpha=col[3] / 255))
    sc.export(f"{outdir}/model.glb")
    subs = {}
    for q in parts: subs.setdefault(q["sub"], []).append(q["c"])
    import math
    th = math.radians(K["pose"]["recline"]); L = 0.47 * K["pose"]["H"]; hip = K["hip"]
    eye = [hip[0] - L * math.sin(th) + 90, 0.0, hip[2] + L * math.cos(th)]
    from .ergo import _knee
    pz = K["pose"]; Hm = pz["H"]; aoff = np.array([-0.115 * Hm, 0.075 * Hm]); bbr = np.asarray(pz["bb_rel"], float); ped1 = np.asarray(pz["pedal_rel"], float)
    h3 = lambda r: conv([hip[0] + r[0], 0.0, hip[2] + r[1]])
    legs = []
    for ped in (ped1, 2 * bbr - ped1):
        ank = ped + aoff; kn, _ = _knee(np.zeros(2), ank, pz["thigh"], pz["shank"]); legs.append(dict(knee=h3(kn), ankle=h3(ank), ped=h3(ped)))
    rider = dict(legs=legs, H=K["pose"]["H"] / 1000, rec=th, hip=conv(hip), knee=conv(K["knee"]), ankle=conv(K["ankle"]), grips=[conv(K["grip_c"][1]), conv(K["grip_c"][-1])])
    man = dict(rider=rider, eye=conv(eye), wb=K["WB"] / 1000, track=K["T"] / 1000, rear_x=-K["WB"] / 1000, parts=parts, subs={k: np.mean(v, axis=0).round(4).tolist() for k, v in subs.items()}, order=SUBS,
               mass=round(sum(q["mass"] for q in parts), 2), n=len(parts))
    json.dump(man, open(f"{outdir}/manifest.json", "w"))
    return man

if __name__ == "__main__":
    m = export(); print(m["n"], "instances", m["mass"], "kg")
