"""Main-frame beam FEA from the registry: rider 150 kg x 3 g. Sanity check only."""
import math
import numpy as np
from .fea import Frame, cantilever_check, YIELD

HAZ = 0.6        # welded heat-affected-zone knock-down applied to yield for the 'with HAZ' column

def run(reg, K, P, rider_kg=150.0, g_vert=3.0):
    fr = Frame(); G = 9.81
    for i in reg.insts:
        m = i.meta
        if i.part.kind == "tube" and "od" in m and m.get("group") in ("frame",):
            fr.add(m["p"], m["q"], m["od"], m["wall"], i.part.material, f"{i.part.no} {i.label}", coped=(m["ends"]["p"]["kind"] == "cope", m["ends"]["q"]["kind"] == "cope"))
    # sleeve (frame tube SL, already above) + boom stub outside the sleeve
    bq = np.array(K["boom_q"], float)
    sl = [i for i in reg.insts if i.label == "SL"][0]; xs1 = sl.meta["q"][0]
    boom = [i for i in reg.insts if i.label == "BOOM"][0]
    fr.add((xs1, 0, bq[2]), tuple(boom.meta["q"]), 25, boom.meta["wall"], "25CrMo4", "TR-111 BOOM (outside sleeve)")
    px, zf = K["pivot"][0], K["zf"]
    # extra nodes (rider load points)
    for y in (195.0, -195.0):
        fr.node((-520.0, y, 366.0))
    # lugs = stiff links
    LUG = (50.0, 8.0)
    for s in (1, -1):
        fr.add((-640.0, s * 98.0, zf), (px, s * 98.0, zf), *LUG, "S355", f"lug link {'L' if s>0 else 'R'}")
        fr.add((K["bridge"][0], s * 101.0, K["bridge"][1]), (px, s * 98.0, zf), *LUG, "S355", f"lug-bridge link {'L' if s>0 else 'R'}")
    fr.node((K["bridge"][0], 0.0, K["bridge"][1]))
    fr.snap_ends()
    fr.split_on_nodes(tol=1.5)
    # --- loads
    Wr = rider_kg * g_vert * G
    veh = (sum(1 for _ in [0]) and 56.0) * g_vert * G
    L_ = {}
    def add(p, f):
        n = fr.node(p); L_[n] = np.array(L_.get(n, np.zeros(6))) + np.array(f, float)
    for y in (195.0, -195.0): add((-520.0, y, 366.0), (0, 0, -0.35 * Wr, 0, 0, 0))                 # seat (70 %)
    for y in (195.0, -195.0): add((-640.0, y, 366.0), (0, 0, -0.10 * Wr, 0, 0, 0))                 # backrest hinge (20 %)
    add(tuple(boom.meta["q"]), (0, 0, -0.10 * Wr, 0, 0, 0))                                         # feet on pedals (10 %)
    for X in (-220.0, -640.0): add((X, 0.0, zf), (0, 0, -0.5 * veh, 0, 0, 0))                       # rest of vehicle, lumped
    tot = Wr + veh; xc = (0.35 * Wr * 2 * -520 + 0.10 * Wr * 2 * -640 + 0.10 * Wr * boom.meta["q"][0] + 0.5 * veh * (-220 - 640)) / tot
    R = tot * (-xc) / float(-K["rear_axle"][0])                                                                        # rear wheel ground reaction
    # swing-arm statics about the pivot
    A = np.array(K["rear_axle"], dtype=float); piv = np.array((px, 0.0, zf)); lo = np.array(K["shock_lo"]); hi = np.array(K["shock_hi"]); dirs = (hi - lo) / np.linalg.norm(hi - lo)
    r_w = A - piv; r_s = lo - piv
    # torque_y of wheel force R*z: -r_x*R ; torque_y of shock force on arm (-Fs*dir): -(r_z*dirs_x... ) solve Fs
    tw = -r_w[0] * R
    ts = r_s[2] * (-dirs[0]) - r_s[0] * (-dirs[2])
    Fs = -tw / ts
    piv_f = np.array((0, 0, R)) - Fs * dirs
    for s in (1, -1): add((px, s * 98.0, zf), (*(piv_f / 2), 0, 0, 0))
    add((K["bridge"][0], 0.0, K["bridge"][1]), (*(Fs * dirs), 0, 0, 0))
    # --- front wheel reactions enter the head tubes as a vertical force plus the couple from the wheel/kingpin offset (balanced load set)
    ht = {s: [i for i in reg.insts if i.label == f"HT{'L' if s>0 else 'R'}"][0] for s in (1, -1)}
    Ff = (tot - R) / 2.0
    for s in (1, -1):
        pp, qq = np.array(ht[s].meta["p"], float), np.array(ht[s].meta["q"], float)
        mid = (pp + qq) / 2; off = s * 375.0 - mid[1]; dz = qq[2] - pp[2]
        add(tuple(pp), (0, 0, Ff / 2, 0, 0, 0)); add(tuple(qq), (0, 0, Ff / 2, 0, 0, 0))
        fy = -s * Ff * abs(off) / dz           # lateral couple (top/bottom bearing) from F * offset
        add(tuple(pp), (0, -fy, 0, 0, 0, 0)); add(tuple(qq), (0, fy, 0, 0, 0, 0))
    n0 = fr.node((K["F"][0], 0.0, zf))
    fixed = {n0: [0, 1, 2, 3, 4, 5]}
    deg = {}
    for e in fr.elems:
        deg[e[0]] = deg.get(e[0], 0) + 1; deg[e[1]] = deg.get(e[1], 0) + 1
    floating = [fr.nodes[n].tolist() for n in range(len(fr.nodes)) if n not in deg]
    u, res = fr.solve(L_, fixed)
    resid = float(np.linalg.norm(fr.reaction[:3]))
    # reactions
    rows = []
    for r in res:
        y = YIELD.get(r["mat"], 355.0)
        rows.append(dict(element=r["name"], od=r["od"], wall=r["wall"], length=round(r["L"], 1), vm=round(r["vm"], 1), yield_MPa=y, ratio=round(r["vm"] / y, 3),
                         ratio_haz=round(r["vm"] / (y * HAZ), 3), lug_link=("link" in r["name"])))
    rows.sort(key=lambda d: -d["ratio"])
    tubes = [r for r in rows if not r["lug_link"]]
    zdisp = {n: u[6 * n + 2] for n in range(len(fr.nodes))}
    nmax = min(zdisp, key=zdisp.get)
    out = dict(rider_kg=rider_kg, g=g_vert, total_vertical_N=tot, rear_reaction_N=R, front_reaction_each_N=Ff, residual_reaction_N=resid, shock_force_N=Fs, shock_to_wheel_ratio=Fs / R, pivot_force_N=[float(x) for x in piv_f],
               n_nodes=len(fr.nodes), n_elems=len(fr.elems), max=tubes[0], top=tubes[:8], max_defl_mm=float(zdisp[nmax]), max_defl_at=[float(x) for x in fr.nodes[nmax]],
               floating_nodes=floating, cantilever=cantilever_check(), rows=rows, haz=HAZ)
    return out
