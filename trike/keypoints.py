"""All key coordinates derived from params.yaml (the parametric core). World mm: X fwd, Y left(+), Z up, origin on ground under front axle."""
import math, yaml, numpy as np
from .core import V
from .ergo import user_pose

def load_params(path="params.yaml"):
    return yaml.safe_load(open(path))

def keypoints(P):
    v = lambda s, k: P[s][k]["value"]
    K = {}
    WB, T = v("layout", "wheelbase"), v("layout", "front_track_cc")
    FR, RR = v("wheels", "front_tyre_od") / 2, v("wheels", "rear_tyre_od") / 2
    K.update(WB=WB, T=T, FR=FR, RR=RR, tyre_w=v("wheels", "tyre_width"))
    K["front_wheel"] = {+1: (0.0, T / 2, FR), -1: (0.0, -T / 2, FR)}
    K["rear_axle"] = (-WB, 0.0, RR)
    pose = user_pose(P); K["pose"] = pose
    hip = np.array([v("layout", "hip_x_from_front_axle"), 0.0, pose["hip_y"]]); K["hip"] = tuple(hip)
    bb_nom = hip + np.array([pose["bb_rel"][0], 0.0, pose["bb_rel"][1] - 0.0])
    bb_nom[2] = v("layout", "bb_height"); K["bb"] = tuple(bb_nom)
    K["ankle"] = tuple(hip + np.array([pose["ankle_rel"][0], 0, pose["ankle_rel"][1]])); K["knee"] = tuple(hip + np.array([pose["knee_rel"][0], 0, pose["knee_rel"][1]]))
    K["pedal_far"] = tuple(hip + np.array([pose["pedal_rel"][0], 0, pose["pedal_rel"][1]]))
    zf = v("front", "frame_plane_z"); K["zf"] = zf
    # kingpin axis: through ground point G (x=+trail, y=T/2-scrub), tilted by caster (top back) and KPI (top inboard)
    c, k = math.radians(v("front", "caster")), math.radians(v("front", "kingpin_inclination")); scrub = v("front", "scrub_radius")
    trail = FR * math.tan(c) * 0 + 25.0
    K["trail"] = trail
    def kp(side, z):
        return (trail - math.tan(c) * z, side * (T / 2 - scrub - math.tan(k) * z), z)
    K["kp"] = kp
    K["kp_dir"] = {s: tuple(V(kp(s, 1) ) - V(kp(s, 0))) for s in (1, -1)}
    K["ht_z"] = (v("front", "headtube_z0"), v("front", "headtube_z1"))
    K["U"] = {s: kp(s, zf) for s in (1, -1)}                     # ends of upper cross tube
    K["F"] = (K["U"][1][0], 0.0, zf)                             # keel front node (centre of U)
    # seat (nominal) and rails
    K["pan_len"] = 440.0; K["seat_w"] = 450.0; K["pan_x"] = (hip[0] - 120, hip[0] + 320)
    K["X1"] = hip[0] + 300; K["X2"] = hip[0] - 120                 # seat cross tubes x
    K["cross_half"] = {1: 270.0, 2: 222.0}; K["rail_y"] = 195.0; K["rail_z"] = 366.0
    K["pivot"] = (K["X2"] + v("rear", "swingarm_pivot_dx_from_x2"), 0.0, zf)
    # boom / sleeve
    stroke, emin = v("boom", "stroke"), v("boom", "min_engagement")
    xr = K["F"][0]; x_si = xr + 15 + 5
    BBmin = bb_nom[0] - stroke / 2; Lb = BBmin - x_si
    K.update(boom_stroke=stroke, boom_len=Lb, sleeve_rear=xr, sleeve_front=x_si + stroke + emin, bb_z=bb_nom[2])
    K["nose_x"] = bb_nom[0] + stroke / 2 + pose["crank"] + pose["toe"] + v("canopy", "nose_clearance_ahead_of_foot")
    K["tail_x"] = hip[0] - 0.50 * pose["H"] * math.sin(math.radians(pose["recline"])) - 190
    return K

if __name__ == "__main__":
    K = keypoints(load_params())
    for k_, v_ in K.items():
        if k_ not in ("kp", "pose"): print(k_, v_)
