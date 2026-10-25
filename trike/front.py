"""Front axle & steering (TR-2xx): kingpins, knuckles, wheels, brakes, steering arms, bellcrank, tie rods, drag links, side sticks."""
import math
import numpy as np
from .h import *

SUB = "Steering & front axle"

def build(reg, K, P):
    FR, T, TW = K["FR"], K["T"], K["tyre_w"]
    kp = K["kp"]
    S = {}
    rail_y, rail_z = K["rail_y"], K["rail_z"]
    for s in (1, -1):
        side = "L" if s > 0 else "R"
        g = f"knuckle{side}"
        d = unit(np.array(kp(s, 1)) - np.array(kp(s, 0)))           # pin axis up
        ht0 = np.array(kp(s, K["ht_z"][0])); ht1 = np.array(kp(s, K["ht_z"][1]))
        HTL = float(np.linalg.norm(ht1 - ht0))
        at = lambda t: ht0 + d * t                                    # point on pin axis, t from HT bottom
        # ---- bushes, thrust washers
        for t0, nm in ((1.0, "lo"), (HTL - 26.0, "hi")):
            b = reg_turned(reg, "TR-202", "Kingpin bush (POM)", SUB, [(13, 0), (13, 25)], at(t0), d, material="POM", stock="OD26 ID20 x 25", method="buy (igus / POM tube)",
                           supplier="plastics / igus", cost=1.2, color=(0.95, 0.9, 0.7), bore=20.0, label=f"BU{side}{nm}", fab=False)
            b.part.kind = "bought"; b.meta["group"] = g
        for t0, nm in ((-2.0, "lo"), (HTL, "hi")):
            w = reg_turned(reg, "TR-203", "Kingpin thrust washer (POM)", SUB, [(17, 0), (17, 2)], at(t0), d, material="POM", stock="OD34 ID20 x 2", method="cut from POM sheet",
                           supplier="plastics / igus", cost=0.6, color=(0.95, 0.9, 0.7), bore=20.0, label=f"TW{side}{nm}", fab=False)
            w.part.kind = "bought"; w.meta["group"] = g
        # ---- kingpin (turned): collar at bottom, free top
        prof = [(10, -30), (10, -10), (17, -10), (17, -2), (10, -2), (10, HTL + 16)]
        pin = reg_turned(reg, "TR-201", "Kingpin (Ø20, turned)", SUB, prof, at(0), d, material="25CrMo4", stock="Ø35 bar, 150 mm", method="turn", cost=7.0, label=f"KP{side}")
        pin.meta["group"] = g
        pin_top = at(HTL + 16)
        # ---- knuckle web + stub axle
        zaxle = FR
        ya = lambda yy: s * yy
        web_out = [(ya(298), 184.0), (ya(335), 184.0), (ya(335), 232.0), (ya(298), 232.0)]
        web_out = [(min(a, b2), 0) for a, b2 in []] or web_out
        web = plate_on(reg, "yz", "TR-204", "Knuckle web plate", SUB, web_out, [], 8, -4.0, material="S355", group=g, label=f"KW{side}", cost=2.5)
        K.setdefault("axle_c", {})[s] = (0.0, s * 375.0, zaxle)
        ax_prof = [(7.5, 0), (7.5, 147), (6.0, 147), (6.0, 172)]       # r vs distance from y=s*298 outward
        axle = reg_turned(reg, "TR-205", "Stub axle Ø15 (M12 end)", SUB, [(7.5, 0), (7.5, 86), (6.0, 86), (6.0, 104)], (0, s * 296.0, zaxle), (0, s, 0), material="S355",
                          stock="Ø16 bar, 110 mm", method="turn", cost=3.5, label=f"AX{side}")
        axle.meta["group"] = g
        # ---- wheel: tyre, wheelset
        c = (0.0, s * 375.0, zaxle)
        tyre, rim, hub, sp = wheel_solid(c, K["FR"] * 2, TW, 305, 56.0, flange=25.0, bore_r=7.8)
        hub = hub - cyl((0, s * 383.0, zaxle), (0, s * 406.0, zaxle), 13.0) + cyl((0, s * 324.0, zaxle), (0, s * 347.0, zaxle), 26.0, 7.8)
        # hub flanges
        fr = frame_at(c, (0, 1, 0)).location
        buy(reg, "TR-210", 'Tyre 16x1.75 (47-305) + tube', SUB, tyre, material="rubber", stock="47-305", cost=14.0, mass=0.45, color=(0.08, 0.08, 0.08), label=f"TY{side}F")
        buy(reg, "TR-211", 'Front wheelset 16" ISO 305, 56 mm hub, disc mount', SUB, Compound([rim, hub] + sp), material="alu / steel", stock="305 rim, 28 H, 56 mm OLD hub, 20 mm pocket", cost=38.0, mass=0.95,
            color=(0.78, 0.78, 0.8), label=f"WH{side}F")
        # bearings are inside the hub; locknut + washers on the axle tip
        tip = np.array((0, s * (296.0 + 160.0), zaxle))
        washer_at(reg, SUB, np.array((0, s * 385.0, zaxle)), (0, s, 0), 12)
        nut_at(reg, SUB, np.array((0, s * 387.5, zaxle)), (0, s, 0), 12, nyloc=True)
        # ---- brake disc (on hub inboard flange), caliper + bracket
        rotor = reg_turned(reg, "TR-212", "Brake rotor 160 mm 6-bolt", SUB, [(80, 0), (80, 1.8)], (0, s * 322.2, zaxle), (0, s, 0), material="stainless", stock="160 mm 6-bolt", method="buy",
                           supplier="bike shop / online", cost=9.0, color=(0.85, 0.85, 0.88), bore=34.0, label=f"RO{side}F", fab=False)
        rotor.part.kind = "bought"; rotor.part.status = "TO CONFIRM"; rotor.meta["group"] = g
        # (rotor bolts: 6x M5x10 Torx, counted as one line "TR-5xx rotor bolts" in the BOM)
        rb = reg.fastener_no(("rotor_bolt", 5, 10), "Rotor bolt M5x10 Torx (6 per rotor)", material="steel 10.9", stock="M5x10", supplier="supplied with rotor/hub", cost=0.15, mass=0.004)
        for k_ in range(6):
            a = math.radians(60 * k_ + 30)
            p0 = V(22 * math.sin(a), s * 322.2, zaxle + 22 * math.cos(a))
            reg.add(rb, cyl(V(p0.X, s * 322.2, p0.Z), V(p0.X, s * 330.0, p0.Z), 2.5) + cyl(V(p0.X, s * 319.0, p0.Z), V(p0.X, s * 322.2, p0.Z), 4.0), SUB)
        # caliper bracket (plate in xz at inboard side) welded to web
        bx_out = [(-4, 176), (-4, 228), (-96, 226), (-104, 218), (-104, 182), (-96, 176)]
        br = plate_on(reg, "xz", "TR-206", "Caliper bracket", SUB, bx_out, [(-66.0, zaxle + 25.5, 6.5), (-66.0, zaxle - 25.5, 6.5)], 5, (302.0 if s > 0 else -307.0),
                      material="S355", group=g, label=f"CB{side}", cost=2.0)
        cal = Pos(-66, s * 323, zaxle) * Box(58, 32, 62)
        cal = cal - Pos(-66, s * 323.1, zaxle) * Box(70, 3.8, 80)
        cal_i = buy(reg, "TR-213", "Mechanical disc caliper (flat/post mount, with pads)", SUB, cal, material="alu / steel", stock="BB5/BB7 type", cost=16.0, mass=0.28,
                    color=(0.15, 0.15, 0.2), label=f"CA{side}F", group=g)
        yb = s * (307.0 if s > 0 else 307.0)
        for zz in (zaxle + 25.5, zaxle - 25.5):
            bolt(reg, SUB, (-66.0, s * 302.0 if s > 0 else -307.0 + 0.0, zz) if False else (-66.0, s * (302.0 if s > 0 else 307.0), zz), (0, s, 0), 6, 5, targets=(), nut=False, washers=(True, False))
        # ---- steering arm (plate normal to the pin axis, on top of the upper thrust washer)
        tz = HTL + 2.0
        origin = at(tz)
        inward = np.array((0, -s, 0.0)); xd = unit(inward - d * np.dot(inward, d))
        armL = 90.0
        outline = [(-24, 0), (-17, -17), (0, -24), (armL, -12), (armL + 12, 0), (armL, 12), (0, 24), (-17, 17)]
        arm = reg_plate(reg, "TR-207", "Steering arm (12 mm plate)", SUB, outline, [(0, 0, 20.3), (armL, 0, 8.5)], 12.0, tuple(origin), tuple(xd), tuple(d), material="S355",
                        method="laser", cost=3.5, label=f"SA{side}")
        arm.meta["group"] = g
        pick = origin + xd * armL + d * 12.0          # top face at pickup
        S[s] = dict(pin_axis=d, ht0=ht0, pin_top=pin_top, pickup=pick, arm_origin=origin, arm_d=d, arm_L=armL, xd=xd)
        # retaining bolt in pin top
        bolt(reg, SUB, origin + d * 12.0, -d, 10, 14, targets=(pin,), nut=False, washers=(True, False))
        # pin<->arm anti-rotation roll pin (counted, hidden in hub)
        rp = reg.fastener_no(("rollpin", 5, 30), "Roll pin Ø5x30 (spring steel)", material="spring steel", stock="Ø5x30", supplier="hardware store", cost=0.10, mass=0.004)
        pt = origin + d * 6.0
        reg.add(rp, cyl(V(pt) - V(xd) * 0 + V(0, 0, 0) - V(unit(np.cross(d, xd))) * 15, V(pt) + V(unit(np.cross(d, xd))) * 15, 2.5), SUB)
    K["steer"] = S
    # ================= bellcrank
    xb, zb_bc = -60.0, 330.0
    K["bc"] = dict(pivot=(xb, 0.0, zb_bc), tip_dx=-90.0, tip_y=225.0)
    stud = reg_turned(reg, "TR-220", "Bellcrank pivot stud", SUB, [(8, 0), (8, 26), (5, 26), (5, 58)], (xb, 0, 302), (0, 0, 1), material="S355", stock="Ø16 bar, 60 mm", method="turn",
                      cost=2.5, label="BCSTUD")
    stud.meta["group"] = "frame"
    pts = [(-90.0, -225.0), (0.0, 0.0), (-90.0, 225.0)]
    poly = offset_polyline([(a + xb, b) for a, b in pts], 26.0)
    bc = reg_plate(reg, "TR-221", "Steering bellcrank (6 mm plate)", SUB, poly, [(xb, 0, 14.0), (xb - 90.0, 225.0, 8.5), (xb - 90.0, -225.0, 8.5)], 6.0, (0, 0, zb_bc), (1, 0, 0), (0, 0, 1),
                   material="S355", method="laser", cost=4.0, label="BC")
    # bronze bush in bellcrank
    reg_turned(reg, "TR-222", "Bellcrank bush (bronze/oilite)", SUB, [(7, zb_bc - 1.0 + 0), (7, zb_bc + 7.0)], (xb, 0, 0), (0, 0, 1), material="bronze", stock="OD14 ID10 x 8", method="buy", supplier="bearing shop", cost=1.0,
               color=(0.8, 0.55, 0.2), bore=10.0, label="BCB", fab=False).part.kind = "bought"
    washer_at(reg, SUB, (xb, 0, zb_bc + 7.0), (0, 0, 1), 10)
    nut_at(reg, SUB, (xb, 0, zb_bc + 7.0 + 2.0), (0, 0, 1), 10, nyloc=True)
    # ---- tie rods: arm pickup <-> bellcrank tip (top ring, z = plate top + 6)
    ties = {}
    for s in (1, -1):
        tip_top = np.array((xb - 90.0, s * 225.0, zb_bc + 6.0 + 6.0))      # ring centre above plate (bolt through rod end sits on plate top)
        tip_bot = np.array((xb - 90.0, s * 225.0, zb_bc - 6.0))
        pk = S[s]["pickup"] + S[s]["arm_d"] * 6.0
        side = "L" if s > 0 else "R"
        cc = rod_link(reg, SUB, pk, tip_top, S[s]["arm_d"], (0, 0, 1), "TR-241", "Tie rod (Ø14x3 tube)", f"TIE{side}", group="steer")
        ties[s] = dict(a=pk, b=tip_top, cc=cc)
        # bolts at bellcrank tip: vertical M8 through rod end(top) / plate / drag link end(bottom)
        K.setdefault("bc_tip", {})[s] = (tip_top, tip_bot)
        K["steer"][s]["tie"] = ties[s]
        # bolt at the steering arm: along pin axis d from above, nut below plate
        bolt(reg, SUB, pk + S[s]["arm_d"] * 6.0 + S[s]["arm_d"] * 0.0, -S[s]["arm_d"], 8, 6 + 12 + 12 * 0, targets=(), nut=True, washers=(True, True))
        bolt(reg, SUB, tip_top + np.array((0, 0, 6.0)), (0, 0, -1), 8, 12 + 6 + 12, targets=(), nut=True, washers=(True, True))
    # ================= sticks
    sx = -400.0
    K["stick_pivot"] = {s: (sx, s * 270.0, 430.0) for s in (1, -1)}
    drags = {}
    rails = {s: ((K["rail_x"][0], s * rail_y, rail_z), (K["rail_x"][1], s * rail_y, rail_z), 25) for s in (1, -1)}
    for s in (1, -1):
        side = "L" if s > 0 else "R"; g = f"stick{side}"
        # bracket tube
        tube(reg, "TR-130", "Stick bracket tube", SUB, (sx, s * rail_y, rail_z), (sx, s * 282.0, rail_z), 20, 1.5, mains_p=[rails[s]], material="25CrMo4", label=f"SBT{side}")
        y1, y2 = (252.0, 288.0) if s > 0 else (-288.0, -252.0)
        zb_ = rail_z + 10 + 2.0
        prof = [(y1, 452.0), (y1, zb_), (y2, zb_), (y2, 452.0)]
        holes = [((sx, y1, 430.0), (0, 1, 0), 10.5), ((sx, y2, 430.0), (0, 1, 0), 10.5)]
        ub = reg_sheet_bent(reg, "TR-131", "Stick pivot bracket (U, 4 mm)", SUB, prof, 36.0, 4.0, (sx - 18, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), material="S355", holes=holes,
                            method="laser+bend", cost=4.0, bend_r=4.0, label=f"SUB{side}")
        ub.meta["group"] = "frame"
        # sleeve
        sleeve = reg_turned(reg, "TR-132", "Stick pivot sleeve", SUB, [(10, -15), (10, 15)], (sx, s * 270.0, 430.0), (0, 1, 0), material="S355", stock="Ø20 bar, 30 mm", method="turn", cost=1.5,
                            bore=10.1, label=f"SSL{side}")
        sleeve.meta["group"] = g
        pv = np.array((sx, s * 270.0, 430.0))
        sleeve_m = (tuple(pv - np.array((0, 15.0, 0))), tuple(pv + np.array((0, 15.0, 0))), 20)
        top = pv + np.array((math.tan(math.radians(8)) * 220, 0, 220.0))
        lev = np.array((sx, s * 270.0, 342.0))
        tube(reg, "TR-133", "Stick (grip tube)", SUB, tuple(pv), tuple(top), 16, 1.5, mains_p=[sleeve_m], material="25CrMo4", group=g, label=f"STK{side}")
        tube(reg, "TR-134", "Stick lever (lower)", SUB, tuple(pv), tuple(lev), 16, 1.5, mains_p=[sleeve_m], material="25CrMo4", group=g, label=f"LEV{side}")
        # lever tip: lug plate (horizontal) carrying drag-link rod end
        # pivot bolt
        bolt(reg, SUB, (sx, s * 290.0, 430.0), (0, -s, 0), 10, 4 + 1 + 30 + 1 + 4, targets=(), nut=True, washers=(True, True))
        # grip + brake lever
        gd = unit(top - pv)
        grip = reg_turned(reg, "TR-701" if False else "TR-135", "Hand grip (rubber)", SUB, [(15, 0), (15, 110)], tuple(top - gd * 90), tuple(gd), material="rubber", stock="Ø32 x 110 grip", method="buy",
                          supplier="bike shop", cost=3.0, color=(0.05, 0.05, 0.05), label=f"GR{side}", fab=False)
        grip.part.kind = "bought"; grip.meta["group"] = g
        blever = Pos(top[0] + 70 * gd[0] - 20, top[1], top[2] - 55) * Box(95, 14, 12)
        buy(reg, "TR-136", "Brake lever (mechanical, with barrel adjuster)", SUB, blever, material="alu", stock="22.2 mm bar clamp, 2-finger", cost=9.0, mass=0.1, color=(0.2, 0.2, 0.25),
            label=f"BL{side}", group=g)
        K.setdefault("grip_c", {})[s] = tuple(top - gd * 40)
        # drag link: bellcrank tip (below plate) <-> lever bottom
        tb = K["bc_tip"][s][1]
        cc = rod_link(reg, SUB, tb, lev, (0, 0, 1), (0, 1, 0) if False else (0, 0, 1), "TR-242", "Drag link (Ø14x3 tube)", f"DRG{side}", group="steer")
        drags[s] = dict(a=tb, b=lev, cc=cc)
        bolt(reg, SUB, tuple(lev + np.array((0, 0, 6.0))), (0, 0, -1), 8, 12, targets=(), nut=True, washers=(True, True))
    K["drags"] = drags
    return S
