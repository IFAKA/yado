"""Drivetrain (TR-4xx): Bafang BBS01B mid-motor, cranks, chainring, pedals, idlers, derailleur, chain."""
import math
import numpy as np
from .h import *
from .rear import CASS, CASS_Y0, CASS_DY, rad_of

SUB = "Drivetrain"
CHAIN_Y = -48.0

def stadium(c1, r1, c2, r2, n=10):
    a = math.atan2(c2[1] - c1[1], c2[0] - c1[0]); pts = []
    for i in range(n + 1): t = a + math.pi / 2 + math.pi * i / n; pts.append((c2[0] + r2 * math.cos(t), c2[1] + r2 * math.sin(t)))
    for i in range(n + 1): t = a - math.pi / 2 + math.pi * i / n; pts.append((c1[0] + r1 * math.cos(t), c1[1] + r1 * math.sin(t)))
    return pts

def chain_path(pul):
    """pul: list of (cx, cz, r, s) in chain order (s=+1 CCW, -1 CW). Returns segments, arcs, total length."""
    n = len(pul); edges = []
    for i in range(n):
        A, B = pul[i], pul[(i + 1) % n]
        d = np.array([B[0] - A[0], B[1] - A[1]]); k = B[3] * B[2] - A[3] * A[2]
        phi = math.atan2(d[1], d[0]); th = phi - math.asin(k / np.linalg.norm(d))
        u = np.array([math.cos(th), math.sin(th)]); nL = np.array([-u[1], u[0]])
        pa = np.array(A[:2]) - A[3] * A[2] * nL; pb = np.array(B[:2]) - B[3] * B[2] * nL
        edges.append((pa, pb))
    segs, arcs, L = [], [], 0.0
    for i in range(n):
        segs.append(edges[i]); L += float(np.linalg.norm(edges[i][1] - edges[i][0]))
        c = pul[(i + 1) % n]; arr = edges[i][1]; dep = edges[(i + 1) % n][0]
        a0 = math.atan2(arr[1] - c[1], arr[0] - c[0]); a1 = math.atan2(dep[1] - c[1], dep[0] - c[0])
        dl = ((a1 - a0) * c[3]) % (2 * math.pi)
        m = max(2, int(dl / math.radians(8)))
        pts = [np.array([c[0] + c[2] * math.cos(a0 + c[3] * dl * j / m), c[1] + c[2] * math.sin(a0 + c[3] * dl * j / m)]) for j in range(m + 1)]
        arcs.append(pts); L += dl * c[2]
    return segs, arcs, L

def build(reg, K, P):
    ax, _, az = K["rear_axle"]; bbx, _, bbz = K["bb"]
    # ---- motor Bafang BBS01B (coaxial with BB shell, body on the left) + lock ring
    mo = cyl((bbx, 34.0, bbz), (bbx, 112.0, bbz), 49.0) + cyl((bbx, -34.0, bbz), (bbx, 34.0, bbz), 17.0)
    buy(reg, "TR-401", "Mid-drive motor Bafang BBS01B 250 W 36 V (cadence sensor)", SUB, mo, material="alu / steel", stock="BBS01B, 68-73 mm shell", cost=190.0, mass=P["drive"]["motor_mass_kg"]["value"],
        color=(0.12, 0.12, 0.14), label="MOT", notes="Rated power, dimensions and mounting TO CONFIRM from the datasheet / nameplate")
    buy(reg, "TR-402", "Motor lock ring + rotation stop", SUB, cyl((bbx, -40.0, bbz), (bbx, -34.0, bbz), 26.0, 17.5), material="alu", stock="supplied with motor", cost=0.0, mass=0.08, color=(0.3, 0.3, 0.33), label="LKR")
    spin = reg_turned(reg, "TR-403", "BB spindle (part of motor)", SUB, [(9.5, 0), (9.5, 205)], (bbx, -78.0, bbz), (0, 1, 0), material="steel", stock="supplied with motor", method="buy", cost=0.0, color=(0.6, 0.6, 0.65), label="SPN", fab=False)
    spin.part.kind = "bought"; spin.part.mass = 0.3; spin.part.status = "TO CONFIRM"
    # ---- cranks
    cl = P["drive"]["crank_length"]["value"]
    pf = np.array(K["pedal_far"]) - np.array(K["bb"]); thR = math.atan2(pf[2], pf[0]); K["crank_angle"] = math.degrees(thR)
    cr = {}
    for s, (y0, th) in ((-1, (-72.0, thR)), (1, (117.0, thR + math.pi))):
        e = (bbx + cl * math.cos(th), bbz + cl * math.sin(th))
        pl = plate_on(reg, "xz", "TR-404", "Crank arm 170 mm (R/L)", SUB, stadium((bbx, bbz), 16, e, 11), [(bbx, bbz, 19.0), (e[0], e[1], 14.0)], 10, y0, material="6061", group=f"crank{s}",
                      label="CRR" if s < 0 else "CRL", cost=14.0)
        pl.part.kind = "bought"; pl.part.fab = False; pl.part.method = "buy"; pl.part.supplier = "bike shop / online"; pl.part.status = "TO CONFIRM"; pl.part.mass = 0.3
        cr[s] = e
        yo = y0 if s < 0 else y0 + 10
        # pedal: spindle + platform body
        yc = (y0 - 5 - 50) if s < 0 else (y0 + 10 + 5 + 50)
        body = Pos(e[0], yc, e[1]) * Box(95, 100, 20)
        sp = cyl((e[0], y0 + (0 if s < 0 else 10), e[1]), (e[0], yc, e[1]), 7.0)
        buy(reg, "TR-405", "Pedal, platform (9/16\" thread)", SUB, body + sp, material="alu / PA", stock="platform 95x100 with built-in reflector (front and rear)", cost=14.0, mass=0.35, notes="pedal reflectors: sources disagree whether required; fitted to be safe", color=(0.2, 0.2, 0.22), label="PDR" if s < 0 else "PDL", group=f"crank{s}")
        # crank bolt M14 head on the spindle end (counted as part of motor kit)
    # ---- chainring 42T + spider arms + bolts
    N = P["chain"]["chainring_teeth"]["value"]; Rr = rad_of(N)
    ring = frame_at((bbx, CHAIN_Y + 1.25, bbz), (0, 1, 0)).location * Pos(0, 0, 0) * (Cylinder(Rr + 2, 2.5) - Cylinder(52, 4))
    buy(reg, "TR-406", f"Chainring {N}T 130 BCD (narrow-wide not required)", SUB, ring, material="alu", stock=f"{N}T 1/2x11/128", cost=22.0, mass=0.18, color=(0.7, 0.7, 0.75), label="RNG", group="crank-1")
    arms = []
    for k in range(5):
        a = thR + 2 * math.pi * k / 5
        arms.append(Pos(bbx + 40 * math.cos(a), -55.6, bbz + 40 * math.sin(a)) * Rot(0, -math.degrees(a), 0) * Box(44, 12.8, 10))
    buy(reg, "TR-407", "Crank spider (right crank, integral)", SUB, Compound(arms), material="alu", stock="part of right crank", cost=0.0, mass=0.1, color=(0.7, 0.7, 0.75), label="SPD", group="crank-1")
    for k in range(5):
        a = thR + 2 * math.pi * k / 5
        bolt(reg, SUB, (bbx + 57 * math.cos(a), CHAIN_Y + 1.25, bbz + 57 * math.sin(a)), (0, -1, 0), 8, 15.0, targets=(), nut=False, washers=(False, False), label=f"CRB{k}")
    # ---- idlers on the idler plate
    idl = K["idler_c"]; ri = rad_of(12)
    for j, (cx, cz) in enumerate(idl):
        wh = frame_at((cx, CHAIN_Y - 3.5, cz), (0, 1, 0)).location * (Cylinder(ri + 1.5, 7.0) - Cylinder(4.5, 9))
        buy(reg, "TR-408", "Chain idler wheel 12T, Ø8 bore (bush)", SUB, wh, material="PA / alu", stock="12T idler, 7 mm wide", cost=7.0, mass=0.03, color=(0.8, 0.5, 0.1), label=f"ID{j+1}", group=f"idl{j}")
        reg_turned(reg, "TR-409", "Idler spacer Ø14/Ø8.4 x 24.5", SUB, [(7, 0), (7, 24.5)], (cx, CHAIN_Y + 3.5 - 24.5 + 0.0 + 24.5 * 0 - 0, cz), (0, 1, 0), material="S355", stock="Ø14 bar", cost=0.8, bore=8.4, label=f"IS{j+1}").meta["group"] = f"idl{j}"
        bolt(reg, SUB, (cx, CHAIN_Y - 3.5, cz), (0, 1, 0), 8, 36.5, targets=(), nut=True, washers=(True, True), label=f"IB{j+1}")
    # ---- rear derailleur (simplified body, parallelogram, cage, 2 jockey wheels)
    rm = np.array(K["rd_mount"]); Rcog = rad_of(21)
    gx, gz = ax - 10.0, az - Rcog - 22.4 - 8.0                 # guide pulley
    tx, tz = ax - 32.0, gz - 70.0                              # tensioner pulley
    body = Pos(rm[0], -84.0, rm[2]) * Box(30, 12, 30) + cyl((rm[0], -90.0, rm[2]), (rm[0], -73.5, rm[2]), 5.0)
    l1 = cyl((rm[0], -84.0, rm[2] + 8), (gx, -57.0, gz + 14), 5.0); l2 = cyl((rm[0], -84.0, rm[2] - 8), (gx, -57.0, gz - 2), 5.0)
    buy(reg, "TR-410", "Rear derailleur 9-speed, medium cage (body + linkage)", SUB, body + l1 + l2, material="alu / steel", stock="9s, 36T capacity", cost=28.0, mass=0.26, color=(0.25, 0.25, 0.3), label="RD", group="rd")
    for nm, o in (("outer", -54.0), ("inner", -45.0)):
        pl = plate_on(reg, "xz", "TR-411", "Derailleur cage plate", SUB, stadium((gx, gz), 14, (tx, tz), 14), [(gx, gz, 5.2), (tx, tz, 5.2)], 3, o, material="steel", group="rd", label=f"CG{nm[0]}")
        pl.part.kind = "bought"; pl.part.fab = False; pl.part.method = "buy"; pl.part.supplier = "bike shop / online"; pl.part.mass = 0.04
    for j, (cx, cz) in enumerate(((gx, gz), (tx, tz))):
        buy(reg, "TR-412", "Jockey wheel 11T", SUB, frame_at((cx, -51.0, cz), (0, 1, 0)).location * (Cylinder(22.4 + 1.5, 6) - Cylinder(2.6, 8)) if False else
            frame_at((cx, -48.0, cz), (0, 1, 0)).location * (Cylinder(22.4 + 1.0, 6) - Cylinder(2.6, 8)), material="PA / alu", stock="11T, 5 mm bore", cost=6.0, mass=0.02, color=(0.2, 0.2, 0.2), label=f"JW{j+1}", group="rd")
        bolt(reg, SUB, (cx, -54.0, cz), (0, 1, 0), 5, 12.0, targets=(), nut=True, washers=(False, False), label=f"JB{j+1}")
    # ---- chain
    pul = [(bbx, bbz, Rr, 1), (idl[0][0], idl[0][1], ri, 1), (ax, az, Rcog, 1), (gx, gz, 22.4, -1), (tx, tz, 22.4, 1), (idl[1][0], idl[1][1], ri, 1)]
    segs, arcs, L = chain_path(pul)
    links = int(math.ceil(L / 12.7 / 2) * 2)
    boxes = []
    for (a, b) in segs: boxes += sweep_poly_boxes([tuple(a), tuple(b)], 6.6, 8.5, CHAIN_Y)
    for pts in arcs: boxes += sweep_poly_boxes([tuple(p) for p in pts], 6.6, 8.5, CHAIN_Y)
    buy(reg, "TR-413", f"Chain 9-speed 1/2x11/128, {links} links (2 chains + connector)", SUB, Compound(boxes), material="steel", stock=f"{links} links", cost=26.0, mass=round(links * 0.0023, 3),
        color=(0.35, 0.33, 0.3), label="CHN", status="TO CONFIRM", notes="link count from the modelled path at nominal boom position; re-measure on the real bike")
    K["chain"] = dict(links=links, length=L, pul=pul, segs=[(tuple(a), tuple(b)) for a, b in segs], rd=((gx, gz), (tx, tz)))
    return dict(links=links)
