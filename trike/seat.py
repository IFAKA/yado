"""Seat (TR-5xx): sliding sled (pan + U-bolt clamps), backrest frame with adjustable recline struts, board, foam, head pad."""
import math
import numpy as np
from .h import *

SUB = "Seat"
ALU = dict(material="6061", supplier="aluminium tube stockist", price_m=3.2)

def build(reg, K, P):
    slide = K.get("seat_slide", 0.0)
    x0, x1 = K["pan_x"][0] + slide, K["pan_x"][1] + slide
    ry, rz = K["rail_y"], K["rail_z"]
    zt = rz + 12.5                                  # rail top 378.5
    # ---- UHMW slide strips on the rails
    for s in (1, -1):
        buy(reg, "TR-502", "Slide strip UHMW 25x3 (bonded under pan)", SUB, Pos((x0 + 60 + x1) / 2, s * ry, zt + 1.5) * Box(x1 - x0 - 60, 25, 3), material="HDPE", stock="25x3 UHMW tape/strip",
            cost=2.5, status="", color=(0.9, 0.9, 0.85), label=f"UH{'L' if s>0 else 'R'}", notes="bonded with PU adhesive")
    # ---- pan (2 mm alu, bent lips) with holes for the U-bolt legs
    uxs = (x0 + 80.0, x1 - 50.0)
    holes = [((xu, s * ry + k_ * 15.5, 380.0), (0, 0, 1), 6.5) for xu in uxs for s in (1, -1) for k_ in (-1, 1)]
    pan = reg_sheet_bent(reg, "TR-501", "Seat pan (2 mm, bent lips)", SUB, [(-224, 395.0), (-224, zt + 4), (224, zt + 4), (224, 395.0)], x1 - x0, 2.0, (x0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0),
                         material="5052", holes=holes, cost=14.0, bend_r=2.0, label="PAN")
    pan.meta["group"] = "seat"
    zp = zt + 3 + 2                                   # pan top
    # ---- U-bolts
    for xu in uxs:
        for s in (1, -1):
            yc = s * ry
            legs = cyl((xu, yc - 15.5, rz), (xu, yc - 15.5, zp + 12), 3.0) + cyl((xu, yc + 15.5, rz), (xu, yc + 15.5, zp + 12), 3.0)
            tor = frame_at((xu, yc, rz), (1, 0, 0)).location * Torus(15.5, 3.0)
            tor = tor - Pos(xu, yc, rz + 30) * Box(20, 70, 60)
            buy(reg, "TR-503", "U-bolt M6 for Ø25 tube (leg spacing 31)", SUB, legs + tor, material="stainless A2", stock="M6, ID 25", cost=1.6, mass=0.06, status="", color=(0.75, 0.75, 0.8), label=f"UB{int(xu)}{'L' if s>0 else 'R'}")
            for k_ in (-1, 1):
                washer_at(reg, SUB, (xu, yc + k_ * 15.5, zp), (0, 0, 1), 6)
                nut_at(reg, SUB, (xu, yc + k_ * 15.5, zp + 1.6), (0, 0, 1), 6, nyloc=True)
    # ---- cushion (EVA foam) with holes over the nut stacks
    hcut = [((xu, s * ry + k_ * 15.5, zp + 8), 18.0) for xu in uxs for s in (1, -1) for k_ in (-1, 1)]
    cu_x0 = x0 + 100
    reg_panel(reg, "TR-512", "Seat cushion (EVA foam 16 mm)", SUB, [(cu_x0, -222, zp + 8), (x1, -222, zp + 8), (x1, 222, zp + 8), (cu_x0, 222, zp + 8)], 16.0, material="foam", cost=9.0,
              color=(0.15, 0.15, 0.17), label="CUSH", holes3d=[h for h in hcut if h[0][0] > cu_x0 - 9], stock="EVA foam 16 mm", method="cut", supplier="foam / upholstery shop", notes="top surface z=400 at the hip point (spec 400)")
    # ---- hinge brackets (welded: base plate + 2 ears), riveted to pan
    th = math.radians(K["pose"]["recline"]); db = np.array([-math.sin(th), 0, math.cos(th)]); nf = np.array([math.cos(th), 0, math.sin(th)])
    hx = x0 + 30.0; hz = zp + 11.5 + 4.0          # pivot height
    pa = (x0 - 80.0, zp + 8.0)                     # strut anchor
    Hp = {s: np.array((hx, s * 200.0, hz)) for s in (1, -1)}
    ear_out = [(x0 + 5, zp + 4), (x0 + 55, zp + 4), (x0 + 55, hz + 16), (x0 + 5, hz + 16), (x0 - 60, hz + 12), (x0 - 100, pa[1] + 14), (x0 - 100, zp + 4)]
    for s in (1, -1):
        y0 = s * 200.0
        base = plate_on(reg, "xy", "TR-506", "Hinge bracket base plate (4 mm)", SUB, [(x0 - 100, y0 - 18), (x0 + 60, y0 - 18), (x0 + 60, y0 + 18), (x0 - 100, y0 + 18)], [], 4, zp, material="S355", group=f"hinge{s}", label=f"HB{'L' if s>0 else 'R'}", cost=2.5)
        for (yy, nm) in ((y0 - 18, "i"), (y0 + 14, "o")):
            hl = [(hx, hz, 8.5)] + ([(pa[0], pa[1], 6.5)] if (yy > y0) else [])
            plate_on(reg, "xz", "TR-507", "Hinge bracket ear (4 mm)", SUB, ear_out, hl, 4, yy, material="S355", group=f"hinge{s}", label=f"HE{'L' if s>0 else 'R'}{nm}")
        for (rx, ry_) in ((x0 + 15, y0 - 8), (x0 + 45, y0 + 8)):
            rivet(reg, SUB, (rx, ry_, zp + 4), (0, 0, -1), 6, 4.8)
            drill_inst(pan, (rx, ry_, zp + 4), (0, 0, -1), 4.9, 8.0)
        for (rx, ry_) in ((x0 + 15, y0 + 8), (x0 + 45, y0 - 8)):
            rivet(reg, SUB, (rx, ry_, zp + 4), (0, 0, -1), 6, 4.8)
            drill_inst(pan, (rx, ry_, zp + 4), (0, 0, -1), 4.9, 8.0)
    # ---- backrest frame
    TL = 760.0
    tcs = (150.0, 400.0, 650.0)
    side = {}
    for s in (1, -1):
        p0 = Hp[s] - db * 14.0; q0 = Hp[s] + db * TL
        ang = hole_ang(q0 - p0, (0, 1, 0))
        t = tube(reg, "TR-509", "Backrest side tube 20x1.5 (alu)", SUB, tuple(p0), tuple(q0), 20, 1.5, holes=[(14.0, ang, 8.5), (14.0 + 380.0, ang, 6.5)], group=f"back{s}", label=f"BK{'L' if s>0 else 'R'}", **ALU)
        side[s] = (tuple(p0), tuple(q0))
    for j, tc in enumerate(tcs):
        c = Hp[1] + db * tc; cl = Hp[-1] + db * tc
        mp = [(side[-1][0], side[-1][1], 20)]; mq = [(side[1][0], side[1][1], 20)]
        ang = hole_ang((0, 1, 0), nf)
        hl = [(200 + yy, ang, 5.5) for yy in (-100.0, 100.0)] + ([(200 + yy, ang, 5.5) for yy in (-30.0, 30.0)] if j == 2 else [])
        tube(reg, "TR-510", "Backrest cross tube 20x1.5 (alu)", SUB, tuple(cl), tuple(c), 20, 1.5, mains_p=mp, mains_q=mq, holes=hl, group="back", label=f"BX{j+1}", **ALU)
    # board (HDPE 6 mm) and foam
    def sheet(t0, t1, hw, off):
        return [tuple(Hp[1] * 0 + np.array((hx, 0, hz)) + db * tt + nf * off + np.array((0, yy, 0))) for tt, yy in ((t0, -hw), (t0, hw), (t1, hw), (t1, -hw))]
    off_b = 13.0
    bh = [(tuple(np.array((hx, 0, hz)) + db * tc + nf * (off_b + 3) + np.array((0, yy, 0))), 5.5) for tc in tcs for yy in (-100.0, 100.0)]
    reg_panel(reg, "TR-511", "Backrest board (HDPE 6 mm)", SUB, sheet(80, 740, 190, off_b), 6.0, material="HDPE", cost=11.0, color=(0.85, 0.85, 0.8), label="BOARD", holes3d=bh + [
        (tuple(np.array((hx, 0, hz)) + db * 650 + nf * 16 + np.array((0, yy, 0))), 5.5) for yy in (-30.0, 30.0)], stock="HDPE 6 mm sheet", method="cut + drill", supplier="plastics sheet stockist")
    reg_panel(reg, "TR-513", "Backrest foam (EVA 20 mm)", SUB, sheet(100, 740, 180, off_b + 3 + 10), 20.0, material="foam", cost=7.0, color=(0.15, 0.15, 0.17), label="BFOAM", stock="EVA foam 20 mm",
              method="cut", supplier="foam / upholstery shop", holes3d=[(tuple(np.array((hx, 0, hz)) + db * tc + nf * 23 + np.array((0, yy, 0))), 14.0) for tc in tcs for yy in (-100.0, 100.0)])
    reg_panel(reg, "TR-514", "Head support pad (EVA 30 mm)", SUB, sheet(600, 760, 90, off_b + 3 + 20 + 15), 30.0, material="foam", cost=5.0, color=(0.15, 0.15, 0.17), label="HPAD", stock="EVA foam 30 mm",
              method="cut", supplier="foam / upholstery shop")
    for j, tc in enumerate(tcs):
        for yy in (-100.0, 100.0):
            pt = np.array((hx, 0, hz)) + db * tc + nf * (off_b + 6) + np.array((0, yy, 0))
            bolt(reg, SUB, tuple(pt), tuple(-nf), 5, 6 + 20, targets=(), nut=True, washers=(True, True), kind="button")
    for yy in (-30.0, 30.0):
        pt = np.array((hx, 0, hz)) + db * 650 + nf * (off_b + 6 + 20 + 15 + 15) + np.array((0, yy, 0))
        bolt(reg, SUB, tuple(pt), tuple(-nf), 5, 30 + 6 + 20 + 20, targets=(), nut=True, washers=(True, True), kind="button")
    # ---- hinge bolt + recline strut (flat bar with holes for 30..45 deg)
    for s in (1, -1):
        y0 = s * 200.0
        yi, yo = ((y0 - 18, y0 + 18) if s > 0 else (y0 + 18, y0 - 18))
        bolt(reg, SUB, (hx, yi if s > 0 else yi, hz), (0, 1 if s > 0 else -1, 0), 8, 36, targets=(), nut=True, washers=(True, True), label=f"HG{'L' if s>0 else 'R'}")
        # strut bar between the tube and the outer ear (outer ear is on the +/- y side)
        sy = s * (200.0 + 10.5 + 1.5)
        def bk(deg):
            r = math.radians(deg); d_ = np.array([-math.sin(r), math.cos(r)])
            return np.array((hx, hz)) + d_ * 380.0
        Pa = np.array(pa)
        Dn = float(np.linalg.norm(bk(K["pose"]["recline"]) - Pa))
        Ds = [float(np.linalg.norm(bk(a) - Pa)) for a in (30, 35, 40, 45)]
        bar_len = max(Ds) - min(Ds) + 40
        ux = (Pa - bk(K["pose"]["recline"])) / Dn
        a_pt = bk(K["pose"]["recline"])
        # hole k at distance Dk from the top pin; bar extends 20 beyond ends
        offs = [d_ - min(Ds) for d_ in Ds]
        t0 = -15.0; t1 = Dn + 15.0
        px = lambda t: a_pt + ux * t
        w = 10.0; n_ = np.array([-ux[1], ux[0]])
        outl = [tuple(px(t0) + n_ * w), tuple(px(t1) + n_ * w), tuple(px(t1) - n_ * w), tuple(px(t0) - n_ * w)]
        hh = [(a_pt[0], a_pt[1], 6.5)] + [(px(Dk)[0], px(Dk)[1], 6.5) for Dk in Ds]
        o_ = (200.0 + 10.5) if s > 0 else -(200.0 + 10.5 + 3.0)
        plate_on(reg, "xz", "TR-515", "Recline strut bar 20x3 (holes for 30/35/40/45 deg)", SUB, outl, hh, 3, o_, material="S355", group=f"strut{s}", label=f"STR{'L' if s>0 else 'R'}", cost=1.8)
        bolt(reg, SUB, (a_pt[0], (200.0 + 14.0 + 4.0) if s > 0 else -(200.0 + 14.0 + 4.0), a_pt[1]) if False else (a_pt[0], s * (200.0 - 12.0), a_pt[1]), (0, s, 0), 6, 24.0, targets=(), nut=True, washers=(True, True), label=f"SP{'L' if s>0 else 'R'}")
        ptx = px(Ds[2])
        bolt(reg, SUB, (pa[0], s * (200.0 + 18.0), pa[1]), (0, -s, 0), 6, 4 + 0.5 + 3, targets=(), nut=True, washers=(True, True), label=f"SA{'L' if s>0 else 'R'}")
    K["seat_pts"] = dict(hinge=(hx, hz), back_dir=tuple(db), anchor=pa, x0=x0, x1=x1, zp=zp)
    return {}
