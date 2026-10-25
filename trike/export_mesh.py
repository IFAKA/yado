"""Tessellate registry instances -> trimesh Scene -> GLB (+ per-inst meshes for later use)."""
import numpy as np, trimesh
from .core import inst_mass

def inst_mesh(inst, tol=0.4, ang=0.35):
    sh = inst.solid
    try:
        v, t = sh.tessellate(tol, ang)
    except Exception as e:
        return None
    if not len(t): return None
    V = np.array([[p.X, p.Y, p.Z] for p in v], float); F = np.array(t, int)
    m = trimesh.Trimesh(V, F, process=False)
    return m

def build_scene(reg, tol=0.4, ang=0.35):
    sc = trimesh.Scene(); n = 0
    for i, inst in enumerate(reg.insts):
        m = inst_mesh(inst, tol, ang)
        if m is None: continue
        col = list(inst.part.color) + [1.0] * (4 - len(inst.part.color))
        col = [int(255 * c) for c in col[:4]]
        m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(col, (len(m.faces), 1)))
        name = f"{inst.part.no}|{inst.label}|{i}"
        sc.add_geometry(m, node_name=name, geom_name=name)
        inst.meta["mesh_name"] = name; n += 1
    return sc
