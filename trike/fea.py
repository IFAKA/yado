"""Small 3D Euler-Bernoulli frame solver (6 dof/node) + validation against a hand-calculated cantilever.
SANITY CHECK ONLY: linear, welded joints rigid, no buckling, no weld HAZ, no fatigue, loads simplified."""
import math
import numpy as np

E = 210e3; G = 80.0e3      # MPa
YIELD = {"S355": 355.0, "25CrMo4": 435.0}

def sec(od, wall):
    ri = od / 2 - wall
    A = math.pi * ((od / 2) ** 2 - ri ** 2); I = math.pi / 4 * ((od / 2) ** 4 - ri ** 4)
    return A, I, 2 * I

class Frame:
    def __init__(s):
        s.nodes = []; s.elems = []; s.coped = []     # elem: (i,j,od,wall,mat,name)
    def node(s, p, tol=1.5):
        p = np.array(p, float)
        for i, q in enumerate(s.nodes):
            if np.linalg.norm(q - p) < tol: return i
        s.nodes.append(p); return len(s.nodes) - 1
    def add(s, p, q, od, wall, mat, name, coped=(False, False)):
        i, j = s.node(p), s.node(q)
        if i != j:
            s.elems.append([i, j, od, wall, mat, name]); s.coped.append(coped)
    def snap_ends(s, extra=3.0):
        """a coped tube end sits on the surface of the main tube: move that end node onto the main tube's axis."""
        deg = {}
        for e in s.elems:
            deg[e[0]] = deg.get(e[0], 0) + 1; deg[e[1]] = deg.get(e[1], 0) + 1
        for k, e in enumerate(s.elems):
            for pos in (0, 1):
                n = e[pos]
                if deg[n] > 1 or not s.coped[k][pos]: continue
                p = s.nodes[n]; best = None
                for k2, e2 in enumerate(s.elems):
                    if k2 == k: continue
                    a, b = s.nodes[e2[0]], s.nodes[e2[1]]; d = b - a; L = np.linalg.norm(d); u = d / L
                    t = float(np.dot(p - a, u))
                    if t < -1.0 or t > L + 1.0: continue
                    q = a + u * min(max(t, 0.0), L); dist = np.linalg.norm(p - q)
                    if dist <= e2[2] / 2 + extra and (best is None or dist < best[0]): best = (dist, q)
                if best is not None:
                    s.nodes[n] = best[1]
    def split_on_nodes(s, tol=2.5):
        """T-junctions: split every element at nodes lying on its axis."""
        changed = True
        while changed:
            changed = False
            for k, (i, j, od, wall, mat, name) in enumerate(s.elems):
                a, b = s.nodes[i], s.nodes[j]; d = b - a; L = np.linalg.norm(d); u = d / L
                for n, q in enumerate(s.nodes):
                    if n in (i, j): continue
                    t = np.dot(q - a, u)
                    if 5.0 < t < L - 5.0 and np.linalg.norm(q - (a + u * t)) < tol:
                        c0 = s.coped[k]; s.elems[k] = [i, n, od, wall, mat, name]; s.coped[k] = (c0[0], False); s.elems.append([n, j, od, wall, mat, name]); s.coped.append((False, c0[1])); changed = True; break
                if changed: break
    def solve(s, loads, fixed):
        """loads: {node: (Fx,Fy,Fz,Mx,My,Mz)} N,Nmm ; fixed: {node: [dof idx]}"""
        n = len(s.nodes); K = np.zeros((6 * n, 6 * n)); Ts = []
        for (i, j, od, wall, mat, name) in s.elems:
            a, b = s.nodes[i], s.nodes[j]; L = np.linalg.norm(b - a); ex = (b - a) / L
            ref = np.array((0, 0, 1.0)) if abs(ex[2]) < 0.99 else np.array((1.0, 0, 0))
            ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
            R = np.array([ex, ey, ez]); T = np.zeros((12, 12))
            for k in range(4): T[3 * k:3 * k + 3, 3 * k:3 * k + 3] = R
            A, I, J = sec(od, wall)
            k = np.zeros((12, 12))
            EA = E * A / L; GJ = G * J / L; a1 = 12 * E * I / L ** 3; a2 = 6 * E * I / L ** 2; a3 = 4 * E * I / L; a4 = 2 * E * I / L
            k[0, 0] = k[6, 6] = EA; k[0, 6] = k[6, 0] = -EA
            k[3, 3] = k[9, 9] = GJ; k[3, 9] = k[9, 3] = -GJ
            for (v, th, sg) in ((1, 5, 1), (2, 4, -1)):   # bending in local y (v, rz) and z (w, ry)
                k[v, v] = k[v + 6, v + 6] = a1; k[v, v + 6] = k[v + 6, v] = -a1
                k[v, th] = k[th, v] = sg * a2; k[v, th + 6] = k[th + 6, v] = sg * a2
                k[v + 6, th] = k[th, v + 6] = -sg * a2; k[v + 6, th + 6] = k[th + 6, v + 6] = -sg * a2
                k[th, th] = k[th + 6, th + 6] = a3; k[th, th + 6] = k[th + 6, th] = a4
            kg = T.T @ k @ T
            idx = list(range(6 * i, 6 * i + 6)) + list(range(6 * j, 6 * j + 6))
            K[np.ix_(idx, idx)] += kg; Ts.append((T, k, idx, L, A, I, J, od))
        F = np.zeros(6 * n)
        for nd, f in loads.items(): F[6 * nd:6 * nd + 6] += f
        fx = sorted(set(6 * nd + d for nd, ds in fixed.items() for d in ds))
        free = [d for d in range(6 * n) if d not in fx]
        # singular-safe: tiny regularisation on rotational dofs that nothing constrains (pure-torsion free ends)
        Kf = K[np.ix_(free, free)] + np.eye(len(free)) * 1e-6
        u = np.zeros(6 * n); u[free] = np.linalg.solve(Kf, F[free])
        s.reaction = (K @ u - F)[fx].reshape(-1) if fx else None; s.fixed_dofs = fx
        res = []
        for e, (T, k, idx, L, A, I, J, od) in zip(s.elems, Ts):
            f = k @ (T @ u[idx])                      # local end forces
            c = od / 2; worst = 0; where = None
            for end, o in ((0, 0), (1, 6)):
                N = f[o]; My = f[o + 4]; Mz = f[o + 5]; Tq = f[o + 3]
                sg = abs(N) / A + math.hypot(My, Mz) * c / I; tau = abs(Tq) * c / J
                vm = math.sqrt(sg ** 2 + 3 * tau ** 2)
                if vm > worst: worst, where = vm, end
            res.append(dict(name=e[5], mat=e[4], vm=worst, end=where, i=e[0], j=e[1], L=L, od=e[2], wall=e[3]))
        return u, res

def cantilever_check():
    """Hand calc: tip load P on a cantilever Ø25x1.5, L=500: delta=PL^3/(3EI), sigma=P L c / I. Solver vs formula."""
    f = Frame(); P = 1000.0; L = 500.0; od, wall = 25.0, 1.5
    f.add((0, 0, 0), (L, 0, 0), od, wall, "S355", "cant")
    f.split_on_nodes()
    # subdivide into 5 elements to also test assembly
    f2 = Frame()
    for k in range(5): f2.add((L * k / 5, 0, 0), (L * (k + 1) / 5, 0, 0), od, wall, "S355", f"c{k}")
    u, res = f2.solve({5: (0, 0, -P, 0, 0, 0)}, {0: [0, 1, 2, 3, 4, 5]})
    A, I, J = sec(od, wall)
    d_hand = P * L ** 3 / (3 * E * I); s_hand = P * L * (od / 2) / I
    d_fe = abs(u[6 * 5 + 2]); s_fe = max(r["vm"] for r in res)
    return dict(P=P, L=L, section=f"{od:g}x{wall:g}", I=I, delta_hand=d_hand, delta_fe=d_fe, delta_err_pct=100 * (d_fe - d_hand) / d_hand,
                sigma_hand=s_hand, sigma_fe=s_fe, sigma_err_pct=100 * (s_fe - s_hand) / s_hand)
