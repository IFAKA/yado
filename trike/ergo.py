"""Planar ergonomic model (estimates). Returns the pose for the chosen rider at the nominal seat position."""
import math, numpy as np
from scipy.optimize import brentq

def _seg(H): return 0.245 * H, 0.246 * H
def _knee(hip, ankle, a, b):
    d = min(np.linalg.norm(ankle - hip), a + b - 1e-6); ang = math.atan2(*(ankle - hip)[::-1])
    cA = (a * a + d * d - b * b) / (2 * a * d); t = ang + math.acos(max(-1, min(1, cA)))
    return hip + a * np.array([math.cos(t), math.sin(t)]), d

def user_pose(P, stature=None):
    v = lambda s, k: P[s][k]["value"]
    H = stature or v("riders", "user_stature"); crank = v("drive", "crank_length"); bbh = v("layout", "bb_height")
    hip_y = v("layout", "seat_pan_height") + v("layout", "hip_joint_above_pan"); flex = v("riders", "knee_flexion_bottom")
    a, b = _seg(H); aoff = np.array([-0.115 * H, 0.075 * H])
    def cycle(D):
        th = math.asin((bbh - hip_y) / D); bb = D * np.array([math.cos(th), math.sin(th)]); res = []
        for ph in np.radians(np.arange(0, 360, 3)):
            ped = bb + crank * np.array([math.cos(ph), math.sin(ph)]); ank = ped + aoff
            k, d = _knee(np.zeros(2), ank, a, b); res.append((ph, ped, ank, k, d))
        return bb, res
    target = math.sqrt(a * a + b * b - 2 * a * b * math.cos(math.radians(180 - flex)))
    D = brentq(lambda D: max(r[4] for r in cycle(D)[1]) - target, 400, 1500)
    bb, res = cycle(D); ph, ped, ank, k, d = max(res, key=lambda r: r[4])
    return dict(H=H, D=D, bb_rel=bb, hip_y=hip_y, thigh=a, shank=b, knee_rel=k, ankle_rel=ank, pedal_rel=ped, crank=crank,
                recline=v("layout", "backrest_recline_set"), toe=v("canopy", "toe_beyond_pedal_axle") if "canopy" in P and "toe_beyond_pedal_axle" in P["canopy"] else 90)
