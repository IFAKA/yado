"""Rear trailing swing-arm, shock, rear wheel + disc (TR-3xx)."""
import math
import numpy as np
from .h import *

SUB = "Rear swing-arm & wheel"
CASS = [36, 32, 28, 24, 21, 18, 15, 13, 11]          # 9-speed 11-36 (TO CONFIRM)
CASS_Y0, CASS_DY = -30.0, -4.5                       # centre of the 36T sprocket, pitch

def rad_of(N): return 12.7 / (2 * math.sin(math.pi / N))

def build(reg, K, P):
    SS0 = P["chain"]["single_speed"]["value"]; RIG = P["rear"]["rigid"]["value"]
    ax, _, az = K["rear_axle"]; px, _, zf = K["pivot"]
    C = (25, 1.5); B = (30, 2); D20 = (20, 1.5)
    # ---- pivot tube + bushes + bolt
    pt = ((px, -98.0, zf), (px, 98.0, zf))
    tube(reg, "TR-301", "Swing-arm pivot tube", SUB, *pt, *B, material="25CrMo4", group="swing", label="SWPT")
    for s in (1, -1):
        reg_turned(reg, "TR-308", "Pivot end plug (clamped, does not turn)" if RIG else "Pivot bush (pressed in tube end)", SUB, [(16, 0), (16, 2), (13, 2), (13, 22)], (px, s * 100.0, zf), (0, -s, 0), material="steel" if RIG else "bronze / PA", stock="OD26 x 22 flanged",
                   method="turn / buy (igus)", cost=2.5, bore=10.5, label=f"PB{'L' if s>0 else 'R'}", notes="press fit into 26 mm bore").meta["group"] = "swing"
    bolt(reg, SUB, (px, -107.0, zf), (0, 1, 0), 10, 214, targets=(), nut=True, washers=(True, True), label="PIV")
    # ---- arms
    dz = az - 250.0
    arm_end = (ax + 28.0, 252.0 + dz)
    for s in (1, -1):
        y = s * 85.0
        tube(reg, "TR-302", "Swing-arm tube", SUB, (px, y, zf), (arm_end[0], y, arm_end[1]), *C, mains_p=[(pt[0], pt[1], 30)], material="25CrMo4", group="swing",
             label=f"SWA{'L' if s>0 else 'R'}")
    # ---- dropouts (left has the caliper tab), hanger
    dl = [(ax - 30, 222 + dz), (ax + 62, 222 + dz), (ax + 62, 282 + dz), (ax - 12, 282 + dz), (ax - 12, 350 + dz), (ax - 68, 350 + dz), (ax - 68, 282 + dz), (ax - 30, 282 + dz)]
    ang = math.radians(120.0); R_cal = 80.0
    cc = (ax + R_cal * math.cos(ang), az + R_cal * math.sin(ang)); tg = (-math.sin(ang), math.cos(ang))
    cb = [(cc[0] + k_ * 25.5 * tg[0], cc[1] + k_ * 25.5 * tg[1]) for k_ in (-1, 1)]
    plate_on(reg, "xz", "TR-304", "Rear dropout, left (with caliper tab)", SUB, dl, [(ax, az, 10.5)] + [(u, v, 5.5) for u, v in cb], 6, 67.5, material="S355", group="swing", label="DOL")
    dr = [(ax - 30, 222 + dz), (ax + 62, 222 + dz), (ax + 62, 282 + dz), (ax - 30, 282 + dz)]
    plate_on(reg, "xz", "TR-305", "Rear dropout, right", SUB, dr, [(ax, az, 10.5)], 6, -73.5, material="S355", group="swing", label="DOR")
    hg = [(ax - 14, 264 + dz), (ax + 14, 264 + dz), (ax + 14, 236 + dz), (ax - 8, 196 + dz), (ax - 36, 196 + dz), (ax - 36, 230 + dz)]
    if not SS0: plate_on(reg, "xz", "TR-306", "Derailleur hanger (replaceable)", SUB, hg, [(ax, az, 10.5), (ax - 22, 206 + dz, 10.0)], 3, -76.5, material="6061", method="laser (alu)", group="hanger", label="HGR", cost=4.0)
    K["rd_mount"] = (ax - 22.0, -79.5, 206.0 + dz)
    # ---- axle with nuts
    reg_turned(reg, "TR-307", "Rear axle Ø10 (threaded both ends)", SUB, [(5, 0), (5, 180)], (ax, -90.0, az), (0, 1, 0), material="8.8 steel", stock="M10 rod 180 mm", method="cut + thread",
               cost=2.0, label="RAX", notes="Ø10 solid axle, nuts both sides (hub bearings 10 mm ID: TO CONFIRM)").meta["group"] = "axle"
    washer_at(reg, SUB, (ax, 73.5, az), (0, 1, 0), 10); nut_at(reg, SUB, (ax, 75.5, az), (0, 1, 0), 10, nyloc=True)
    washer_at(reg, SUB, (ax, -76.5, az), (0, -1, 0), 10); nut_at(reg, SUB, (ax, -78.5, az), (0, -1, 0), 10, nyloc=True)
    # ---- wheel
    c = (ax, 0.0, az)
    tyre, rim, hub, sp = wheel_solid(c, K["RR"] * 2, K["tyre_w"], 305, 135.0, flange=36.0, spokes=28)
    fr = frame_at(c, (0, 1, 0)).location
    adaptor = fr * Pos(0, 0, 44) * (Cylinder(28, 16) - Cylinder(18, 18))
    buy(reg, "TR-310", "Tyre 16x1.75 (47-305) + tube", SUB, tyre, material="rubber", stock="47-305", cost=16.0, mass=0.6, color=(0.08, 0.08, 0.08), label="TYR")
    buy(reg, "TR-311", 'Rear wheelset 16" ISO 305, 135 mm OLD (TO CONFIRM: hub availability), 9-sp freehub, disc mount', SUB, Compound([rim, hub, adaptor] + sp), material="alu / steel", stock="305 rim, 28 H, 135 OLD",
        cost=62.0, mass=1.5, color=(0.78, 0.78, 0.8), label="WHR")
    casp = []
    SS = P["chain"]["single_speed"]["value"]; cass = [P["chain"]["cog_teeth"]["value"]] if SS else CASS
    for i, N in enumerate(cass):
        yc = CASS_Y0 + CASS_DY * i
        casp.append(frame_at((ax, yc, az), (0, 1, 0)).location * (Cylinder(rad_of(N), 2.0) - Cylinder(19, 3)))
    buy(reg, "TR-312", f"Cog {cass[0]}T single speed" if SS else "Cassette 9-speed 11-36", SUB, Compound(casp), material="steel", stock="HG cog" if SS else "HG 9s 11-36", cost=22.0, mass=0.45, color=(0.55, 0.55, 0.6), label="CAS")
    rotor = reg_turned(reg, "TR-313", "Brake rotor 160 mm 6-bolt (rear)", SUB, [(80, 0), (80, 1.8)], (ax, 54.0, az), (0, 1, 0), material="stainless", stock="160 mm 6-bolt", method="buy",
                       supplier="bike shop / online", cost=9.0, color=(0.85, 0.85, 0.88), bore=70.0, label="RORR", fab=False)
    rotor.part.kind = "bought"; rotor.part.status = "TO CONFIRM"
    cal = Pos(cc[0], 53.0, cc[1]) * Box(58, 26, 62)
    cal = cal - fr * Pos(0, 0, 54.9) * Cylinder(82.5, 3.8)
    buy(reg, "TR-314", "Mechanical disc caliper (flat mount, with pads)", SUB, cal, material="alu / steel", stock="BB5/BB7 type", cost=16.0, mass=0.28, color=(0.15, 0.15, 0.2), label="CAR")
    for (u, v) in cb:
        bolt(reg, SUB, (u, 73.5, v), (0, -1, 0), 5, 19, targets=(), nut=False, washers=(True, False))
    K["rear_cal_c"] = cc
    # ---- shock + lower mount
    lo, hi = np.array(K["shock_lo"]), np.array(K["shock_hi"])
    t = (px - lo[0]) / (px - arm_end[0]); zc = zf + (arm_end[1] - zf) * t
    tube(reg, "TR-303", "Shock cross tube (between arms)", SUB, (lo[0], -85.0, zc), (lo[0], 85.0, zc), *D20,
         mains_p=[((px, -85.0, zf), (arm_end[0], -85.0, arm_end[1]), 25)], mains_q=[((px, 85.0, zf), (arm_end[0], 85.0, arm_end[1]), 25)], material="25CrMo4", group="swing", label="SWXT")
    for yy in (10.0, -14.0):
        plate_on(reg, "xz", "TR-309", "Shock lower clevis plate", SUB, [(lo[0] - 16, zc - 16), (lo[0] + 16, zc - 16), (lo[0] + 16, lo[2] + 6), (lo[0], lo[2] + 14), (lo[0] - 16, lo[2] + 6)],
                 [(lo[0], zc, 20.3), (lo[0], lo[2], 8.5)], 4, yy, material="S355", group="swing", label=f"SLC{int(yy)}")
    u = (hi - lo) / np.linalg.norm(hi - lo)
    if P["rear"]["rigid"]["value"]:
        tube(reg, "TR-316", "Rigid strut (replaces shock; same pin centres)", SUB, tuple(lo), tuple(hi), 20, 1.5, material="25CrMo4", group="swing", label="SHK")
    else:
        body = cyl(lo + u * 12, lo + u * 62, 17.0) + cyl(lo + u * 62, hi - u * 12, 6.0) + (cyl(lo + u * 40, hi - u * 14, 23.0) - cyl(lo + u * 40, hi - u * 14, 19.0))
        eyes = frame_at(lo, (0, 1, 0)).location * Cylinder(9, 20) + frame_at(hi, (0, 1, 0)).location * Cylinder(9, 20)
        buy(reg, "TR-315", "Coil shock 165x38 (eyelet, 8 mm bushes)", SUB, body + eyes, material="steel/alu", stock="165 mm EYE-EYE, 38 mm stroke, spring rate TO SELECT", cost=28.0, mass=0.45,
            color=(0.9, 0.2, 0.15), label="SHK")
    bolt(reg, SUB, (lo[0], -14.0, lo[2]), (0, 1, 0), 8, 28, targets=(), nut=True, washers=(True, True), label="SHL")
    bolt(reg, SUB, (hi[0], -14.0, hi[2]), (0, 1, 0), 8, 28, targets=(), nut=True, washers=(True, True), label="SHU")
    return dict(arm_end=arm_end)
