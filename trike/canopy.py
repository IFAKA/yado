"""Canopy (TR-6xx): 20x1.5 aluminium tube skeleton in two side planes, panels riveted outside, hinged lid, rail clamps."""
import math
import numpy as np
from .h import *

SUB = "Canopy"
ALU = dict(material="6061", supplier="aluminium tube stockist", price_m=3.2)
PANEL = dict(material="coroplast 4 mm", cost=None, color=(0.85, 0.87, 0.9))

def _off(poly, n, d):
    return [tuple(np.array(p, float) + np.array(n, float) * d) for p in poly]

def rivet_line(reg, a, b, n_in, grip=5.5, pitch=280.0, inset=0.0):
    a = np.array(a, float); b = np.array(b, float)
    L = np.linalg.norm(b - a); n = max(1, int(round(L / pitch)))
    for i in range(n + 1):
        t = (i + 0.5) / (n + 1) if False else i / n
        if i in (0, n):
            t = 0.02 + 0.96 * t
        rivet(reg, SUB, tuple(a + (b - a) * t), n_in, grip)

def build(reg, K, P):
    TAIL = K["rear_axle"][0] - K["RR"] - 25.0; NOSE = float(round(K["nose_x"]))
    y = 330.0
    pr = {"A": (NOSE, 440.0), "B": (NOSE, 880.0), "C": (-350.0, 1250.0), "D": (-1050.0, 1250.0), "E": (TAIL, 900.0), "F": (TAIL, 570.0), "G": (-900.0, 570.0), "H": (-700.0, 440.0)}
    loop = ["A", "B", "C", "D", "E", "F", "G", "H"]
    sides = {}
    for s in (1, -1):
        for i, k in enumerate(loop):
            a, b = pr[k], pr[loop[(i + 1) % 8]]
            p = (a[0], s * y, a[1]); q = (b[0], s * y, b[1])
            tube(reg, "TR-601", "Canopy side tube 20x1.5 (alu)", SUB, p, q, 20, 1.5, group="canopy", label=f"CS{k}{loop[(i+1)%8]}{'L' if s>0 else 'R'}", **ALU)
    # cross tubes between the two side planes at the corners
    for k in ("A", "B", "C", "D", "E", "G", "H"):
        x, z = pr[k]
        mp = [((x, -y, z), (x, -y, z), 20)]
        tube(reg, "TR-602", "Canopy cross tube 20x1.5 (alu)", SUB, (x, -y, z), (x, y, z), 20, 1.5, group="canopy", label=f"CX{k}", **ALU)
    # ---- support posts from frame rails / front boom up to the canopy bottom rail
    ry, rz = K["rail_y"], K["rail_z"]
    posts = [(-120.0), (-720.0)]
    for xp in posts:
        zr = rz + 12.5 + 5.0
        zb = 440.0 if xp > -700 else 440.0
        for s in (1, -1):
            tube(reg, "TR-603", "Canopy post 20x1.5 (alu)", SUB, (xp, s * (ry + 0.0), rz + 22.5), (xp, s * y, zb), 20, 1.5, group="canopy", label=f"PO{int(abs(xp))}{'L' if s>0 else 'R'}", **ALU)
    # window rear pillar: bottom rail (G-F line) to top rail (C-D line) at x=-1000, edge support for the side window
    for s_ in (1, -1):
        tube(reg, "TR-607", "Canopy window pillar 20x1.5 (alu)", SUB, (-1000.0, s_ * y, 570.0 + 10.0), (-1000.0, s_ * y, 1250.0 - 10.0), 20, 1.5, group="canopy", label=f"WP{'L' if s_>0 else 'R'}", **ALU)
    # front support: cross tube at x=130 and a vertical strut to the boom
    xf = 130.0
    tube(reg, "TR-604", "Canopy front cross tube 20x1.5 (alu)", SUB, (xf, -y, 440.0), (xf, y, 440.0), 20, 1.5, group="canopy", label="CXF", **ALU)
    bp, bq = np.array(K["boom_p"], float), np.array(K["boom_q"], float)
    t = (xf - bp[0]) / (bq[0] - bp[0]); bz = bp[2] + (bq[2] - bp[2]) * t; by = bp[1] + (bq[1] - bp[1]) * t
    tube(reg, "TR-605", "Canopy front strut 20x1.5 (alu)", SUB, (xf, by, bz + 12.5 + 10), (xf, 0, 440.0 - 10), 20, 1.5, group="canopy", label="STF", **ALU)
    # side planes A-H rail must reach the front cross tube: add the x=130 point on the bottom rail
    # ---- rail clamps (cross-tube clamps Ø25 to Ø20) at the post bases and the strut base
    def clamp(c, ax_main, label):
        c = np.array(c, float)
        body = frame_at(tuple(c), (0, 1, 0)).location * (Cylinder(16, 26) - Cylinder(12.5, 28))
        body2 = Pos(*c) * Pos(0, 0, 22.5) * Box(30, 26, 8)
        buy(reg, "TR-606", "Tube clamp 25/20 cross (alu, 2-bolt)", SUB, body + body2, material="alu", stock="25/20 cross clamp", cost=4.5, mass=0.06, label=label, notes="bolt-on, avoids welding alu to steel")
        for dx in (-9, 9):
            bolt(reg, SUB, (c[0] + dx, c[1], c[2] + 22.5 + 4), (0, 0, -1), 5, 20, targets=(), nut=True, washers=(True, False), label=label + "b")
    for xp in posts:
        for s in (1, -1):
            clamp((xp, s * ry, rz + 0.0), "y", f"CL{int(abs(xp))}{'L' if s>0 else 'R'}")
    # ---- panels (riveted outside tubes)
    off = 10.0 + 2.0
    XW = -1000.0; ZW = 800.0                      # window: upper front part of the side wall (clear PC), rider sees traffic at junctions
    low = [pr["A"], (NOSE, ZW), (XW, ZW), (XW, 1250.0), pr["D"], pr["E"], pr["F"], pr["G"], pr["H"]]
    win = [(NOSE, ZW), pr["B"], pr["C"], (XW, 1250.0), (XW, ZW)]
    for s in (1, -1):
        sd = 'L' if s > 0 else 'R'
        reg_panel(reg, "TR-611", "Canopy side wall, lower (coroplast 4 mm)", SUB, [(p[0], s * (y + off), p[1]) for p in low], 4.0, material="coroplast 4 mm", cost=7.0, color=(0.88, 0.9, 0.95), label=f"SW{sd}", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.45)
        reg_panel(reg, "TR-618", "Canopy side window (polycarbonate 2 mm)", SUB, [(p[0], s * (y + off), p[1]) for p in win], 2.0, material="PC 2 mm", color=(0.85, 0.92, 1.0), label=f"WIN{sd}", stock="PC 2 mm clear", method="cut", supplier="plastics sheet stockist", alpha=0.3, notes="scratch-resistant coating and anti-fog TO CONFIRM")
        for i, k in enumerate(loop):
            a, b = pr[k], pr[loop[(i + 1) % 8]]
            rivet_line(reg, (a[0], s * (y + 10), a[1]), (b[0], s * (y + 10), b[1]), (0, -s, 0), 5.5)
        # side mirror: stalk bolted through window + tube, head outside (rear view; seat back blocks the rear)
        xm, zm = 420.0, 880.0 + (NOSE - 420.0) / (NOSE + 350.0) * 370.0 + 0.0
        c = (xm, s * 374.0, zm + 8)
        body = cyl((xm, s * (y + off + 2), zm), (xm, s * 374.0, zm + 8), 4.0) + frame_at(c, (1, 0, 0)).location * Cylinder(28, 8)
        buy(reg, "TR-624", "Side mirror, convex 56 mm, bolt-on stalk", SUB, body, material="ABS / glass", stock="bike/trike mirror 56 mm", cost=8.0, mass=0.08, label=f"MIR{sd}", notes="field of view TO CONFIRM; may need larger head")
        bolt(reg, SUB, (xm, s * (y + off + 6), zm), (0, -s, 0), 5, 30, targets=(), nut=True, washers=(True, True), label=f"MIRb{sd}")
    reg_panel(reg, "TR-612", "Canopy nose panel (polycarbonate 2 mm, 2 vents)", SUB, [(NOSE + off, -y, 440.0), (NOSE + off, y, 440.0), (NOSE + off, y, 880.0), (NOSE + off, -y, 880.0)], 2.0, material="PC 2 mm", color=(0.85, 0.92, 1.0),
              label="NOSE", stock="PC 2 mm clear", method="cut", supplier="plastics sheet stockist", holes3d=[((NOSE + off, yy, 480.0), 50.0) for yy in (-150.0, 150.0)], alpha=0.3)
    for yy in np.linspace(-y, y, 5):
        rivet(reg, SUB, (NOSE + 10, float(yy), 450.0), (-1, 0, 0), 5.5)
    for yy in np.linspace(-y, y, 5)[1:-1]:
        rivet(reg, SUB, (NOSE + 10, float(yy), 870.0), (-1, 0, 0), 5.5)
    def top_panel(no, name, ka, kb, label, mat="coroplast 4 mm", cost=6.0, hinge=False, alpha=0.5, thk=4.0):
        a, b = np.array(pr[ka]), np.array(pr[kb]); d = (b - a) / np.linalg.norm(b - a); n = np.array((-d[1], d[0]))  # outward normal in xz (up/out)
        if n[1] < 0: n = -n
        o = off
        pa = (a[0] + n[0] * o, a[1] + n[1] * o); pb = (b[0] + n[0] * o, b[1] + n[1] * o)
        reg_panel(reg, no, name, SUB, [(pa[0], -y, pa[1]), (pa[0], y, pa[1]), (pb[0], y, pb[1]), (pb[0], -y, pb[1])], thk, material=mat, cost=cost, color=(0.85, 0.92, 1.0) if "PC" in mat else (0.88, 0.9, 0.95),
                  label=label, stock=mat, method="cut", supplier="plastics sheet stockist", alpha=alpha)
        return a, b, n
    # lid (3 mm polycarbonate) B-C, hinged at C
    a, b, n = top_panel("TR-613", "Canopy lid (polycarbonate 3 mm)", "B", "C", "LID", mat="PC 3 mm", cost=26.0, thk=3.0, alpha=0.3)
    top_panel("TR-614", "Canopy roof panel (coroplast 4 mm)", "C", "D", "ROOF")
    top_panel("TR-615", "Canopy tail slope panel (coroplast 4 mm)", "D", "E", "TAILS")
    top_panel("TR-616", "Canopy tail panel (coroplast 4 mm)", "E", "F", "TAILV")
    top_panel("TR-617", "Canopy rear deck (coroplast 4 mm)", "F", "G", "DECK")
    for ka, kb in (("C", "D"), ("D", "E"), ("E", "F"), ("F", "G")):
        a_, b_ = np.array(pr[ka]), np.array(pr[kb]); d = (b_ - a_) / np.linalg.norm(b_ - a_); nn = np.array((-d[1], d[0])); nn = nn if nn[1] >= 0 else -nn
        if ka == "F": nn = np.array((0, 1.0))
        for yy in (-y, y):
            rivet_line(reg, (a_[0], yy, a_[1]), (b_[0], yy, b_[1]), (-nn[0], 0, -nn[1]), 5.0)
    # lid hardware: 2 hinges at C, 2 latches at B, 2 gas struts
    nl = np.array((0.0, 1.0)); dl = np.array(pr["C"]) - np.array(pr["B"]); dl = dl / np.linalg.norm(dl); nl = np.array((-dl[1], dl[0])); nl = nl if nl[1] >= 0 else -nl
    for i, yy in enumerate((-200.0, 200.0)):
        ph = (pr["C"][0] + nl[0] * 14, yy, pr["C"][1] + nl[1] * 14)
        buy(reg, "TR-621", "Lid hinge 60x40 (stainless)", SUB, Pos(*ph) * Box(40, 60, 4), material="stainless A2", stock="piano/butt hinge 60x40", cost=3.5, mass=0.05, label=f"HG{i}")
        for dy in (-20, 20):
            rivet(reg, SUB, (ph[0] + 10, yy + dy, ph[2] - 0), (nl[0] * 0, 0, -1), 6)
        bolt(reg, SUB, (ph[0] - 10, yy, ph[2] + 2), (0, 0, -1), 5, 12, targets=(), nut=True, washers=(True, False), label=f"HGb{i}")
    for i, yy in enumerate((-280.0, 280.0)):
        pl = (pr["B"][0] + nl[0] * 14 + 8, yy, pr["B"][1] + nl[1] * 14 - 2)
        buy(reg, "TR-622", "Lid latch (draw / toggle, stainless)", SUB, Pos(*pl) * Box(30, 20, 8), material="stainless A2", stock="toggle latch 50 mm", cost=3.2, mass=0.04, label=f"LT{i}")
        rivet(reg, SUB, (pl[0] + 8, yy, pl[2] + 4), (0, 0, -1), 5)
    for i, s in enumerate((1, -1)):
        pb = (pr["B"][0] - 10, s * (y - 40), pr["B"][1] + 30)
        pc = (-100.0, s * (y - 40), 1210.0)
        buy(reg, "TR-623", "Gas strut 100 N, 250 mm (lid)", SUB, cyl(pb, pc, 5.0), material="steel", stock="gas spring 250 mm, 100 N", cost=7.0, mass=0.15, label=f"GS{i}", notes="force and ball-stud sizing TO CONFIRM")
    # ---- belly pan: floor, lower skirts, tail wall, nose lower. Front wheels poke out through openings |x|<215.
    ZF = 140.0; zt = ZF + 12.0; xw = 225.0; xt = TAIL + 10.0; xn = NOSE - 10.0; yf = 338.0
    for s in (1, -1):
        sd = 'L' if s > 0 else 'R'
        tube(reg, "TR-625", "Canopy floor rail 20x1.5 (alu)", SUB, (xt, s * y, zt), (-xw, s * y, zt), 20, 1.5, group="canopy", label=f"FRR{sd}", **ALU)
        tube(reg, "TR-625", "Canopy floor rail 20x1.5 (alu)", SUB, (xw, s * y, zt), (xn, s * y, zt), 20, 1.5, group="canopy", label=f"FRF{sd}", **ALU)
        tube(reg, "TR-626", "Canopy floor stringer 20x1.5 (alu)", SUB, (xt, s * 180.0, zt), (xn, s * 180.0, zt), 20, 1.5, group="canopy", label=f"FST{sd}", **ALU)
        for xp, zup in ((-xw, 430.0), (xw, 430.0), (-700.0, 430.0), (xn, 430.0), (xt, 560.0)):
            tube(reg, "TR-627", "Canopy floor post 20x1.5 (alu)", SUB, (xp, s * y, zt + 10), (xp, s * y, zup), 20, 1.5, group="canopy", label=f"FP{int(abs(xp))}{sd}", **ALU)
    for xc in (xt, -xw, xw, xn):
        # the chain runs through y -48 at x=-225: that cross tube is split, leaving the 220 mm slot open
        segs = [(-y, -110.0), (110.0, y)] if xc == -xw else [(-y, y)]
        for j, (ya, yb) in enumerate(segs):
            tube(reg, "TR-628", "Canopy floor cross tube 20x1.5 (alu)", SUB, (xc, ya, zt), (xc, yb, zt), 20, 1.5, group="canopy", label=f"FCX{int(abs(xc))}{'F' if xc > 0 else 'R'}{j}", **ALU)
    zp = ZF
    fl = [(TAIL, -yf), (-215, -yf), (-215, -200), (215, -200), (215, -yf), (NOSE, -yf), (NOSE, yf), (215, yf), (215, 200), (-215, 200), (-215, yf),
          (TAIL, yf), (TAIL, 110), (-60, 110), (-60, -110), (TAIL, -110)]
    reg_panel(reg, "TR-629", "Canopy floor (coroplast 4 mm, rear wheel slot, front wheel notches)", SUB, [(px, py, zp) for px, py in fl], 4.0, material="coroplast 4 mm", cost=14.0,
              color=(0.88, 0.9, 0.95), label="FLOOR", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.6,
              notes="ground clearance 140 mm at the floor; front wheel notches |x|<215; chain and rear wheel slot 220 mm wide, x < -215")
    zlo = ZF - 2.0
    skl = [(-215, zlo), (-215, 440), (-700, 440), (-900, 570), (TAIL, 570), (TAIL, zlo)]
    skf = [(215, zlo), (NOSE, zlo), (NOSE, 440), (215, 440)]
    for s in (1, -1):
        sd = 'L' if s > 0 else 'R'
        yy = s * (y + off)
        reg_panel(reg, "TR-630", "Canopy lower skirt, rear (coroplast 4 mm)", SUB, [(px, yy, pz) for px, pz in skl], 4.0, material="coroplast 4 mm", cost=8.0, color=(0.88, 0.9, 0.95), label=f"SKR{sd}", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.45)
        reg_panel(reg, "TR-631", "Canopy lower skirt, front (coroplast 4 mm)", SUB, [(px, yy, pz) for px, pz in skf], 4.0, material="coroplast 4 mm", cost=5.0, color=(0.88, 0.9, 0.95), label=f"SKF{sd}", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.45)
        for a_, b_ in (((xt, zt), (-xw, zt)), ((xw, zt), (xn, zt))):
            rivet_line(reg, (a_[0], s * (y + 10), a_[1]), (b_[0], s * (y + 10), b_[1]), (0, -s, 0), 5.5)
        for xr0, xr1 in ((TAIL + 20.0, -250.0), (250.0, NOSE - 20.0)):
            rivet_line(reg, (xr0, s * y, zp - 2.0), (xr1, s * y, zp - 2.0), (0, 0, 1), 5.5)
            rivet_line(reg, (xr0, s * 180.0, zp - 2.0), (xr1, s * 180.0, zp - 2.0), (0, 0, 1), 5.5)
    reg_panel(reg, "TR-632", "Canopy tail wall (coroplast 4 mm)", SUB, [(TAIL - off, -y, zlo), (TAIL - off, y, zlo), (TAIL - off, y, 570.0), (TAIL - off, -y, 570.0)], 4.0, material="coroplast 4 mm", cost=6.0,
              color=(0.88, 0.9, 0.95), label="TAILW", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.45)
    reg_panel(reg, "TR-633", "Canopy nose lower (coroplast 4 mm)", SUB, [(NOSE + off, -y, zlo), (NOSE + off, y, zlo), (NOSE + off, y, 440.0), (NOSE + off, -y, 440.0)], 4.0, material="coroplast 4 mm", cost=5.0,
              color=(0.88, 0.9, 0.95), label="NOSEL", stock="coroplast 4 mm", method="cut", supplier="plastics sheet stockist", alpha=0.45)
    for yy in np.linspace(-y, y, 5):
        rivet(reg, SUB, (NOSE + 10, float(yy), zt), (-1, 0, 0), 5.5)
        rivet(reg, SUB, (TAIL - 10, float(yy), zt), (1, 0, 0), 5.5)
    K["canopy_pts"] = pr
    return {}
