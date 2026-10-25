"""Electrics (TR-7xx): battery, controller, display, lights, switches, sensors, bell, reflectors, cables, EPAC labels/caps."""
import math
import numpy as np
from .h import *

SUB = "Electrics"

def cable(reg, no, name, pts, dia, color, label, cost, notes=""):
    """cable run as a chain of cylinders; length computed -> mass/cost by registered part."""
    L = sum(float(np.linalg.norm(np.array(pts[i + 1]) - np.array(pts[i]))) for i in range(len(pts) - 1))
    solid = None
    for i in range(len(pts) - 1):
        c = cyl(tuple(pts[i]), tuple(pts[i + 1]), dia / 2)
        solid = c if solid is None else solid + c
    i = buy(reg, no, name, SUB, solid, material="copper/PVC", stock=f"{L:.0f} mm + 10% slack", cost=cost, mass=0.0003 * L * 1.1, color=color, label=label, notes=notes or "route length from model; add slack TO CONFIRM", kind="cable")
    return i

def build(reg, K, P):
    TAIL = K["rear_axle"][0] - K["RR"] - 25.0; RAX = K["rear_axle"][0]; NOSE = float(round(K["nose_x"]))
    tz = K["tray_z"]; (ya, yb) = K["tray_y"]; (tx0, tx1) = K["tray_x"]
    xm = (tx0 + tx1) / 2
    # battery (left) and controller (right) on the tray
    bz0 = tz + 4.0
    buy(reg, "TR-701", "Battery 36 V 10-14 Ah (case 380x90x80)", SUB, Pos(xm, 70.0, bz0 + 40) * Box(380, 90, 80), material="Li-ion pack in ABS case", stock="380x90x80, 2.8 kg", cost=330.0, mass=2.8, color=(0.1, 0.1, 0.12), label="BAT",
        notes="envelope TO CONFIRM; fixed by two straps; cell protection/BMS and charger by supplier")
    buy(reg, "TR-702", "Controller 36 V, 7 A limit (PAS, no throttle)", SUB, Pos(xm, -70.0, bz0 + 15) * Box(150, 60, 30), material="alu case", stock="150x60x30", cost=45.0, mass=0.45, color=(0.55, 0.55, 0.6), label="CTRL",
        notes="current limit set to 7 A via programming cable; rating TO CONFIRM")
    for (cx, cy, w, h, lbl) in ((xm - 110, 70.0, 90, 80, "S1"), (xm + 110, 70.0, 90, 80, "S2")):
        buy(reg, "TR-703", "Battery strap (nylon 25 mm, buckle)", SUB, Pos(cx, cy, bz0 + 40) * Box(22, w + 6, h + 6) - Pos(cx, cy, bz0 + 40) * Box(30, w, h), material="nylon", stock="25 mm webbing", cost=1.5, mass=0.03, label=lbl, status="")
    for (cx, cy) in ((xm - 110, 70.0 - 55.0), (xm - 110, 70.0 + 55.0), (xm + 110, 70.0 - 55.0), (xm + 110, 70.0 + 55.0)):
        pass
    # tray tie-down eyelets for the straps: M5 bolt through tray with nyloc
    for cx in (xm - 110, xm + 110):
        for cy in (70.0 - 55.0, 70.0 + 55.0):
            bolt(reg, SUB, (cx, cy, bz0 + 6), (0, 0, -1), 5, 7.0, targets=(), nut=True, washers=(True, True), label="SB")
    for cy in (-70.0 - 40, -70.0 + 40):
        for cx in (xm - 55, xm + 55):
            bolt(reg, SUB, (cx, cy, bz0 + 30 + 4), (0, 0, -1), 5, 34.0, targets=(), nut=True, washers=(True, True), label="CB")
    # display + brake cut-off switches + bell on the sticks
    gl, gr = np.array(K["grip_c"][1]), np.array(K["grip_c"][-1])
    buy(reg, "TR-704", "Display (assist level, speed, battery)", SUB, Pos(*(gl + np.array((-60.0, 0.0, 20.0)))) * Box(60, 40, 25), material="ABS", stock="60x40x25", cost=30.0, mass=0.1, color=(0.1, 0.1, 0.12), label="DSP", notes="Bafang-compatible; TO CONFIRM")
    buy(reg, "TR-705", "Bell (alu, 22 mm clamp)", SUB, Pos(*(gr + np.array((-55.0, 0.0, 15.0)))) * Cylinder(25, 20), material="alu", stock="Ø50 bell", cost=5.0, mass=0.08, color=(0.7, 0.7, 0.75), label="BELL", status="")
    for s in (1, -1):
        lv = np.array(K["grip_c"][s]) + np.array((15.0, 0, -30.0))
        buy(reg, "TR-706", "Brake cut-off switch (cable inline)", SUB, Pos(*lv) * Box(30, 14, 12), material="ABS", stock="inline cable switch", cost=4.0, mass=0.02, color=(0.2, 0.2, 0.2), label=f"BSW{'L' if s>0 else 'R'}", notes="cuts motor on braking; wiring TO CONFIRM")
    # lights, reflectors (canopy panels)
    off = 12.0
    buy(reg, "TR-707", "Headlamp (LED, 30 lux, StVZO type)", SUB, Pos(NOSE + off + 6, 0.0, 780.0) * Box(12, 90, 40), material="alu/PC", stock="front lamp", cost=22.0, mass=0.15, color=(0.95, 0.95, 0.8), label="HLAMP", notes="approval marking TO CONFIRM")
    for dy in (-30, 30):
        rivet(reg, SUB, (NOSE + off, dy, 790.0), (1, 0, 0), 5)
    buy(reg, "TR-708", "Rear light (LED, red, always on)", SUB, Pos(TAIL - off - 6, 0.0, 800.0) * Box(12, 80, 35), material="ABS", stock="rear lamp", cost=12.0, mass=0.08, color=(0.8, 0.05, 0.05), label="RLAMP", notes="approval marking TO CONFIRM")
    for dy in (-30, 30):
        rivet(reg, SUB, (TAIL - off, dy, 800.0), (-1, 0, 0), 5)
    buy(reg, "TR-709", "Front reflector (white)", SUB, Pos(NOSE + off + 4, 0.0, 520.0) * Box(8, 50, 40), material="PMMA", stock="E-mark reflector", cost=1.5, mass=0.01, color=(1, 1, 1), label="RFw", status="")
    buy(reg, "TR-710", "Rear reflector (red)", SUB, Pos(TAIL - off - 4, 0.0, 620.0) * Box(8, 50, 40), material="PMMA", stock="E-mark reflector", cost=1.5, mass=0.01, color=(0.8, 0.05, 0.05), label="RFr", status="")
    for s in (1, -1):
        buy(reg, "TR-711", "Side reflector (amber)", SUB, Pos(-300.0, s * (330 + off + 4), 500.0) * Box(40, 8, 20), material="PMMA", stock="E-mark reflector", cost=1.2, mass=0.01, color=(1.0, 0.65, 0.1), label=f"RFs{'L' if s>0 else 'R'}", status="")
        rivet(reg, SUB, (-300.0, s * (330 + off), 500.0), (0, -s, 0), 5)
    # speed sensor on the rear left dropout, magnet on the disc-side spoke
    ax = K["axle"] if "axle" in K else None
    buy(reg, "TR-712", "Speed sensor + spoke magnet", SUB, Pos(RAX + 60, 95.0, 190.0) * Box(30, 8, 10), material="ABS", stock="spoke magnet + reed", cost=6.0, mass=0.02, color=(0.2, 0.2, 0.2), label="SPD", notes="needed for the 25 km/h cut-off; TO CONFIRM position")
    # wheel reflectors (clip-on, amber): sources disagree whether required for cycles; fitted to cover both readings
    for lbl, c in (("RFwF1", (0.0, 375.0, 310.0)), ("RFwF2", (0.0, -375.0, 310.0)), ("RFwR", (RAX, 0.0, 360.0))):
        buy(reg, "TR-716", "Wheel reflector (amber, spoke clip)", SUB, Pos(*c) * Box(30, 6, 18), material="PMMA", stock="E-mark spoke reflector", cost=1.0, mass=0.01, color=(1.0, 0.65, 0.1), label=lbl, status="")
    # EPAC plate and port caps
    mb = np.array((299.1, 0.0, 550.0))
    buy(reg, "TR-713", "Motor power label (250 W, riveted)", SUB, Pos(mb[0], 90.0, mb[2] + 52.0) * Box(40, 1, 20), material="alu", stock="40x20 plate", cost=2.0, mass=0.003, color=(0.8, 0.8, 0.8), label="LBL", status="", notes="text: 250 W continuous rated; value TO CONFIRM")
    buy(reg, "TR-714", "Throttle port cap (blind plug)", SUB, Pos(*(gr + np.array((0.0, 0.0, -90.0)))) * Cylinder(5, 12), material="rubber", stock="waterproof connector cap", cost=0.8, mass=0.002, color=(0.1, 0.1, 0.1), label="CAP", status="", notes="throttle input unplugged and capped")
    buy(reg, "TR-715", "Programming port (capped)", SUB, Pos(xm + 60.0, -70.0, bz0 + 36.0) * Cylinder(6, 10), material="rubber", stock="Bafang programming plug + cap", cost=1.0, mass=0.003, color=(0.1, 0.1, 0.1), label="PRG", status="", notes="USB programming cable (not included): speed limit 25 km/h, current 7 A")
    # cables (battery -> controller -> motor; display; lights)
    mp = (299.1 - 20.0, 100.0, 550.0 - 40.0)
    cable(reg, "TR-720", "Power cable battery - controller", [(xm + 190, 70.0, bz0 + 40), (xm + 210, 0.0, bz0 + 30), (xm + 140, -70.0, bz0 + 15)], 6, (0.8, 0.1, 0.1), "W1", 6.0)
    cable(reg, "TR-721", "Motor cable controller - motor", [(xm + 150, -70.0, bz0 + 15), (xm + 400, -75.0, 215.0), (200.0, -75.0, 235.0), (260.0, -60.0, 450.0), (mp[0], -40.0, 480.0)], 7, (0.1, 0.1, 0.1), "W2", 8.0)
    cable(reg, "TR-722", "Display cable", [(xm + 150, -60.0, bz0 + 30), (-100.0, -140.0, 360.0), tuple(np.array(gl) + np.array((-60.0, 0.0, 10.0)))], 4, (0.1, 0.1, 0.1), "W3", 4.0)
    cable(reg, "TR-723", "Light cable", [(xm + 150, -50.0, bz0 + 30), (-100.0, -280.0, 440.0), (NOSE, -280.0, 460.0), (NOSE, -30.0, 780.0)], 4, (0.1, 0.1, 0.1), "W4", 5.0)
    # zip ties (one number, 14 pcs)
    for i, p in enumerate(((xm, 70, 120), (200.0, 0, 240.0))):
        pass
    buy(reg, "TR-724", "Cable tie 200 mm (pack of 25)", SUB, Pos(xm + 190, 0, bz0 + 4) * Box(5, 3, 3), material="nylon", stock="2.5x200 mm", cost=1.5, mass=0.05, label="ZT", status="", notes="~20 used")
    return {}
