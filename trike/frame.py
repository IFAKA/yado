"""Main frame (TR-1xx): headtubes, U cross tube, keel, cross tubes, riser, boom sleeve + boom, braces, seat rails, battery tray hangers, rear pivot lugs."""
import math
import numpy as np
from .h import *

SUB = "Frame"

def build(reg, K, P):
    zf = K["zf"]; kp = K["kp"]; F = K["F"]; xr = F[0]
    X1, X2 = K["X1"], K["X2"]
    A = (25, 1.5); B = (30, 2); C = (20, 1.5)
    ht = {}
    for s in (1, -1):
        ht[s] = (kp(s, K["ht_z"][0]), kp(s, K["ht_z"][1]))
        tube(reg, "TR-101", "Head tube (kingpin housing)", SUB, *ht[s], *B, material="25CrMo4", holes=(), label=f"HT{'L' if s>0 else 'R'}")
    htm = {s: (ht[s][0], ht[s][1], 30) for s in (1, -1)}
    # U cross tube joining the two headtubes at z=zf
    tube(reg, "TR-102", "Front cross tube (U)", SUB, K["U"][1], K["U"][-1], *B, mains_p=[htm[1]], mains_q=[htm[-1]], material="25CrMo4", label="UX")
    # cross tubes
    ch1, ch2 = K["cross_half"][1], K["cross_half"][2]
    tube(reg, "TR-103", "Seat cross tube X1 (front)", SUB, (X1, -ch1, zf), (X1, ch1, zf), *B, material="25CrMo4", label="X1")
    tube(reg, "TR-104", "Seat cross tube X2 (rear)", SUB, (X2, -ch2, zf), (X2, ch2, zf), *B, material="25CrMo4", label="X2")
    x1m, x2m = ((X1, -ch1, zf), (X1, ch1, zf), 30), ((X2, -ch2, zf), (X2, ch2, zf), 30)
    uxm = (K["U"][1], K["U"][-1], 30)
    # keel
    tube(reg, "TR-105", "Keel front section", SUB, (xr, 0, zf), (X1, 0, zf), *B, mains_p=[uxm], mains_q=[x1m], material="25CrMo4", label="K1")
    tube(reg, "TR-106", "Keel rear section", SUB, (X1, 0, zf), (X2, 0, zf), *B, mains_p=[x1m], mains_q=[x2m], material="25CrMo4", label="K2")
    # sleeve + riser
    zb = K["bb_z"]; xs0 = xr - 15; xs1 = K["sleeve_front"]
    sl = ((xs0, 0, zb), (xs1, 0, zb), 30)
    sleeve = tube(reg, "TR-107", "Boom sleeve", SUB, sl[0], sl[1], *B, material="25CrMo4", label="SL")
    tube(reg, "TR-108", "Riser (neck)", SUB, (xr, 0, zf), (xr, 0, zb), *B, mains_p=[uxm], mains_q=[sl], material="25CrMo4", label="RS")
    # braces: sleeve front -> U cross tube at y=+-150
    bx = xr + 120
    for s in (1, -1):
        tube(reg, "TR-109", "Brace (sleeve to U cross tube)", SUB, (bx, 0, zb), (xr, s * 150, zf), *C, mains_p=[sl], mains_q=[uxm], material="25CrMo4",
             label=f"BR{'L' if s>0 else 'R'}")
    # ---- pinch clamp on sleeve front (slit on top + 2 lugs + M6 bolt)
    slit = Pos(xs1 - 22, 0, zb + 11) * Box(46, 3.0, 24)
    sleeve.solid = sleeve.solid - slit
    xc = xs1 - 20; zc = zb + 15 + 14
    for s in (1, -1):
        y0 = 1.5 if s > 0 else -1.5 - 6
        L = plate_on(reg, "xz", "TR-110", "Pinch clamp lug", SUB, [(xc - 14, zb + 13), (xc + 14, zb + 13), (xc + 14, zc + 10), (xc - 14, zc + 10)],
                     [(xc, zc, 6.5)], 6, y0, material="S355", method="laser+weld", group="frame", label=f"PCL{'L' if s>0 else 'R'}")
        L.solid = L.solid  # holes already cut
    bolt(reg, SUB, (xc, 7.5, zc), (0, -1, 0), 6, 15, targets=(), nut=True)
    # ---- boom (sliding): tube 25x1.5 inside the sleeve, with BB shell welded at the front
    bbx = K["bb"][0]; xb0 = xs1 - 75 - 130 + (bbx - (K["bb"][0] - K["boom_stroke"] / 2) - K["boom_stroke"] / 2) * 0   # nominal rear end
    x_rear_nom = (K["sleeve_rear"] + 20) + K["boom_stroke"] / 2        # boom rear end at mid stroke
    bq = (bbx - 22, 0, zb)
    bp = (bq[0] - K["boom_len"], 0, zb)
    bi = tube(reg, "TR-111", "Boom (sliding)", SUB, bp, bq, *A, material="25CrMo4", group="boom", label="BOOM", holes=[(150.0, 90, 6.4)])
    K["boom_p"], K["boom_q"] = bp, bq
    # BB shell (turned/cut tube 40x3.5 , 68 long) along y at BB
    shell = cyl((bbx, -34, zb), (bbx, 34, zb), 21, 17.5)
    si = reg.add(reg.part("TR-112", "BB shell (motor mount)", SUB, material="S355", stock="42x3.5 tube, 68 mm", method="cut+weld", supplier="steel tube stockist",
                          cost=2.0, kind="tube", fab=True, draw=dict(type="tube", od=42, wall=3.5, length=68, ends={"p": {"kind": "square"}, "q": {"kind": "square"}}, holes=[], axis_len=68)).no, shell, SUB, "BBSH")
    si.meta["group"] = "boom"
    # boom tube is welded into the side of the BB shell: sits at the shell's mid-length, so cut boom end to shell outer radius
    bi.solid = bi.solid - cyl((bbx, -40, zb), (bbx, 40, zb), 21)
    # ---- seat rails + posts
    ry, rz = K["rail_y"], K["rail_z"]; rx0, rx1 = K["hip"][0] - 200, K["hip"][0] + 400
    for s in (1, -1):
        rail = (rx0, s * ry, rz), (rx1, s * ry, rz)
        tube(reg, "TR-113", "Seat rail", SUB, *rail, *A, material="25CrMo4", label=f"RAIL{'L' if s>0 else 'R'}",
             holes=[(30.0, 90, 6.4), (rx1 - rx0 - 30.0, 90, 6.4)])
        for (X, ch, nm) in ((X1, ch1, 1), (X2, ch2, 2)):
            xm = x1m if nm == 1 else x2m
            tube(reg, "TR-114", "Seat rail post", SUB, (X, s * ry, zf + 10), (X, s * ry, rz), *A,
                 mains_p=[xm], mains_q=[(rail[0], rail[1], 25)], material="25CrMo4", label=f"POST{nm}{'L' if s>0 else 'R'}")
        # side truss: lower longeron between the two cross tubes + V diagonals up to the rail
        lo_t = ((X1, s * ry, zf), (X2, s * ry, zf), 25)
        tube(reg, "TR-115", "Side longeron", SUB, *lo_t[:2], *A, mains_p=[x1m], mains_q=[x2m], material="25CrMo4", label=f"LGN{'L' if s>0 else 'R'}")
        xm_ = (X1 + X2) / 2
        for (Xe, nm) in ((X1, 1), (X2, 2)):
            tube(reg, "TR-116", "Side diagonal", SUB, (xm_, s * ry, zf), (Xe, s * ry, rz), *C, mains_p=[lo_t], mains_q=[(rail[0], rail[1], 25)], material="25CrMo4", label=f"DIA{nm}{'L' if s>0 else 'R'}")
    for s in (1, -1):
        tube(reg, "TR-118", "Rail front stay", SUB, (rx1, s * ry, rz), (xr, s * 140.0, zf), *A, mains_p=[(( rx0, s * ry, rz), (rx1, s * ry, rz), 25)], mains_q=[K["U"] and (K["U"][1], K["U"][-1], 30)],
             material="25CrMo4", label=f"RST{'L' if s>0 else 'R'}")
    K["rail_x"] = (rx0, rx1)
    # ---- battery / controller tray hung from X1, X2 (hanger plates welded round the tubes; 3 mm alu tray with side flanges, M5 bolts)
    tz = K["tray_z"] = 188.0
    ya, yb = -150.0, 125.0                      # outer faces of tray flanges
    K["tray_y"] = (ya, yb); K["tray_x"] = (X2 - 15, X1 + 15)
    tray = reg_sheet_bent(reg, "TR-120", "Battery / controller tray", SUB, [(ya + 1.5, tz + 26), (ya + 1.5, tz + 1.5), (yb - 1.5, tz + 1.5), (yb - 1.5, tz + 26)], X1 - X2 + 30, 3,
                          (0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), material="5052", cost=9.0, bend_r=3.0)
    # reg_sheet_bent lies in plane (u=y, v=z) extruded along +x from origin: shift to start at X2-15
    tray.solid = Pos(X2 - 15, 0, 0) * tray.solid
    tray.meta["group"] = "tray"
    hang = []
    for X in (X1, X2):
        for (y, side) in ((ya, -1), (yb, 1)):
            y0 = (y + 3.0) if side < 0 else (y - 3.0 - 4.0)         # hanger plate just inside the flange
            xm = x1m if X == X1 else x2m
            h = plate_on(reg, "xz", "TR-121", "Tray hanger plate", SUB, [(X - 32, tz + 4), (X + 32, tz + 4), (X + 32, zf + 20), (X - 32, zf + 20)],
                         [(X, zf, 30.3)], 4, y0, material="S355", group="frame", label=f"HNG{int(X)}{side}")
            hang.append((h, X, y, side))
            for dx in (-18, 18):
                zc_ = tz + 14
                hd = (0, 1, 0) if side < 0 else (0, -1, 0)
                bolt(reg, SUB, (X + dx, y, zc_), hd, 5, 7, targets=(tray, h), nut=True, washers=(True, True))
    # ---- chain idler plate (welded flat to keel and riser, slotted over the U cross tube)
    idl = [(-170.0, 325.0), (xr + 6.0, 195.0)]
    ypl = -20.0
    ip = plate_on(reg, "xz", "TR-122", "Chain idler plate", SUB, [(-205, 150), (-205, 340), (xr + 40, 340), (xr + 40, 150)],
                  [(xr, zf, 30.3), (idl[0][0], idl[0][1], 8.4), (idl[1][0], idl[1][1], 8.4), (idl[1][0] + 20.0, idl[1][1] + 35.0, 8.4)], 5, ypl, material="S355", group="frame", label="IDLPL")
    K["idler_c"] = idl; K["idler_plate"] = ip
    # ---- rear swing-arm pivot lugs + shock bridge (welded to X2 from behind)
    px = K["pivot"][0]
    K["shock_lo"] = np.array([K["rear_axle"][0] + K["RR"] + 25.0, 0.0, K["zf"] + 10.0]); K["shock_hi"] = K["shock_lo"] + 165 * np.array([math.cos(math.radians(75)), 0, math.sin(math.radians(75))])
    sh = K["shock_hi"]; zbr = sh[2] - 22.0
    for s in (1, -1):
        yl = 101.0 if s > 0 else -107.0
        outline = [(X2 - 13, zf - 18), (px - 30, zf - 18), (px - 62, zf + 5), (px - 62, zbr + 24), (sh[0] - 20, zbr + 24), (sh[0] + 6, zbr + 4), (X2 - 13, zf + 18)]
        lug = plate_on(reg, "xz", "TR-123", "Swing-arm pivot lug", SUB, outline, [(px, zf, 10.5), (sh[0], zbr, 20.3)], 6, yl, material="S355", group="frame",
                       label=f"LUG{'L' if s>0 else 'R'}")
    K["lug_y"] = (101.0, -101.0)
    # shock bridge tube between lugs (carries upper shock clevis)
    zbr = sh[2] - 22.0
    br_ = ((sh[0], -101.0, zbr), (sh[0], 101.0, zbr), 20)
    tube(reg, "TR-124", "Shock bridge tube", SUB, br_[0], br_[1], *B, material="25CrMo4", label="SHBR")
    K["bridge"] = (sh[0], zbr)
    for s in (1, -1):
        rl = ((K["rail_x"][0], s * K["rail_y"], K["rail_z"]), (K["rail_x"][1], s * K["rail_y"], K["rail_z"]), 25)
        tube(reg, "TR-117", "Shock bridge stay", SUB, (sh[0], s * 101.0, zbr), (K["rail_x"][0] + 20.0, s * K["rail_y"], K["rail_z"]), *A, mains_p=[br_ if s > 0 else br_], mains_q=[rl],
             material="25CrMo4", label=f"STAY{'L' if s>0 else 'R'}")
    for yy in (10.0, -14.0):
        plate_on(reg, "xz", "TR-125", "Shock upper clevis plate", SUB, [(sh[0] - 16, zbr - 12), (sh[0] + 16, zbr - 12), (sh[0] + 16, sh[2] + 6), (sh[0], sh[2] + 14), (sh[0] - 16, sh[2] + 6)],
                 [(sh[0], zbr, 20.3), (sh[0], sh[2], 8.5)], 4, yy, material="S355", group="frame", label=f"SUC{int(yy)}")
    return dict(ht=ht, sleeve=sl)
