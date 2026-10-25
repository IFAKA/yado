"""Stage 1: ergonomic check for 5th..95th rider range. Planar model, x forward, y up, mm."""
import sys, csv, math, yaml, numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import brentq

P = yaml.safe_load(open(sys.argv[1])); out = sys.argv[2]
v = lambda sec, k: P[sec][k]["value"]
crank = v("drive","crank_length"); bbh = v("layout","bb_height")
hip_y = v("layout","seat_pan_height")+v("layout","hip_joint_above_pan")
flex = v("riders","knee_flexion_bottom"); gap = v("riders","roof_gap_above_head")
slide = v("layout","seat_slide_pm"); margin = v("layout","boom_margin")
rec_set = v("layout","backrest_recline_set")
WB = v("layout","wheelbase"); hip_x0 = v("layout","hip_x_from_front_axle")
ankle_off = lambda H: np.array([-0.115*H, 0.075*H])   # pedal axle -> ankle (foot toe-up, estimate)

def seg(H): return 0.245*H, 0.246*H
def knee_pos(hip, ankle, a, b, up=True):
    d = np.linalg.norm(ankle-hip); d = min(d, a+b-1e-6)
    ang = math.atan2(*(ankle-hip)[::-1]); cA = (a*a+d*d-b*b)/(2*a*d)
    t = ang + (1 if up else -1)*math.acos(max(-1,min(1,cA)))
    return hip + a*np.array([math.cos(t), math.sin(t)]), d
def interior(a,b,d): return math.degrees(math.acos((a*a+b*b-d*d)/(2*a*b)))

def cycle(H, D):
    a,b = seg(H); dy = bbh-hip_y; th = math.asin(dy/D)
    bb = D*np.array([math.cos(th), math.sin(th)]); res=[]
    for ph in np.radians(np.arange(0,360,3)):
        ped = bb + crank*np.array([math.cos(ph), math.sin(ph)]); ank = ped+ankle_off(H)
        k,d = knee_pos(np.zeros(2), ank, a, b); res.append((ph,ped,ank,k,d))
    return bb, res

def solve_D(H):
    a,b = seg(H); target = math.sqrt(a*a+b*b-2*a*b*math.cos(math.radians(180-flex)))
    f = lambda D: max(r[4] for r in cycle(H,D)[1]) - target
    return brentq(f, 400, 1500)

rows=[]; data={}
for name,H in [("small (1550)",v("riders","range_stature_min")),("mid (1725)",(v("riders","range_stature_min")+v("riders","range_stature_max"))/2),("tall (1900)",v("riders","range_stature_max")),("USER (1720)",v("riders","user_stature"))]:
    D = solve_D(H); a,b = seg(H); bb,res = cycle(H,D)
    kmax = max(r[3][1] for r in res)+hip_y; kmin_int = min(interior(a,b,r[4]) for r in res)
    head = {rc: hip_y+0.50*H*math.cos(math.radians(rc)) for rc in (30,35,40,45)}
    shoulder = 0.285*H + v("riders","clothing_shoulder_allow")
    rows.append(dict(rider=name, stature=H, thigh=round(a), shank=round(b), hip_to_BB=round(D),
        knee_int_top_deg=round(kmin_int), knee_peak_h=round(kmax), shoulder_need=round(shoulder),
        seat_width_need=round(0.215*H+40), head_top_30=round(head[30]), head_top_35=round(head[35]),
        head_top_40=round(head[40]), head_top_45=round(head[45]),
        roof_need_at_set=round(head[rec_set]+gap), buttock_knee=round(0.30*H)))
    data[name]=(H,D,bb,res)
Dmin,Dmax = rows[0]["hip_to_BB"], rows[2]["hip_to_BB"]
need = Dmax-Dmin; boom_travel = need - 2*slide + 2*margin
tall_bb_x = hip_x0 - slide + Dmax*math.cos(math.asin((bbh-hip_y)/Dmax))
length = tall_bb_x + crank + 120 + 50 + (WB + 250)   # pedal front + toe + canopy nose + rear wheel radius
with open(out+"/ergo_table.csv","w",newline="") as f:
    w=csv.DictWriter(f, rows[0].keys()); w.writeheader(); w.writerows(rows)
for r in rows: print(r)
print(f"hip->BB range {Dmin}..{Dmax} (span {need}); seat slide covers {2*slide}; boom travel needed {boom_travel:.0f}")
print(f"tall rider, seat fully back: BB x = {tall_bb_x:.0f} from front axle; overall length ~ {length:.0f} (limit {v('layout','max_overall_length')})")
print(f"width: track {v('layout','front_track_cc')} + tyre {v('wheels','tyre_width')} = {v('layout','front_track_cc')+v('wheels','tyre_width')} (limit {v('layout','max_overall_width')})")

# side view of the three riders at their own BB position, hip fixed
fig,ax=plt.subplots(figsize=(15,6.5))
ax.axhline(0,color="k"); 
for cx,od,c in [(0,v("wheels","front_tyre_od"),"gray"),(-WB,v("wheels","rear_tyre_od"),"gray")]:
    ax.add_patch(plt.Circle((cx,od/2),od/2,fill=False,lw=2,color=c))
hx=hip_x0; cols={"small (1550)":"tab:green","mid (1725)":"tab:blue","tall (1900)":"tab:red","USER (1720)":"k"}
for name,(H,D,bb,res) in data.items():
    c=cols[name]; hip=np.array([hx,hip_y]); a,b=seg(H)
    ph,ped,ank,k,d=max(res,key=lambda r:r[4])           # bottom of stroke
    ax.plot(*zip(hip,hip+k,hip+ank),color=c,lw=2.5,label=f"{name}  hip-BB {D:.0f}")
    ax.plot(*zip(hip+ank,hip+ank+0*ank),color=c)
    ax.add_patch(plt.Circle(hip+bb,crank,fill=False,color=c,ls="--"))
    rc=math.radians(rec_set); top=hip+0.50*H*np.array([-math.sin(rc),math.cos(rc)])
    ax.plot(*zip(hip,top),color=c,lw=6,alpha=.5); ax.plot(*top,"o",color=c,ms=14)
    kk=[hip+r[3] for r in res]; ax.plot(*zip(*kk),color=c,lw=.8,alpha=.6)  # knee path
ax.plot([hx-70,hx+300],[400,400],color="brown",lw=4,label="seat pan 400")
ax.axhline(1250,color="m",ls=":",label="spec head clearance 1250")
ax.axhline(bbh,color="orange",ls=":",label="BB height 550")
ax.set_aspect("equal"); ax.set_xlim(-1700,900); ax.set_ylim(-50,1500); ax.grid(alpha=.3)
ax.set_title(f"Stage 1 side view: riders 1550/1725/1900 mm, backrest {rec_set} deg from vertical, seat at nominal position (hip fixed, BB moves on boom). mm, ESTIMATES")
ax.legend(loc="upper right"); fig.savefig(out+"/ergo_side_view.png",dpi=110,bbox_inches="tight")

import json
H,D,bb,res=data["USER (1720)"]; a,b=seg(H)
ph,ped,ank,k,d=max(res,key=lambda r:r[4])
json.dump(dict(H=H,D=D,bb=list(bb),hip_y=hip_y,thigh=a,shank=b,knee=list(k),ankle=list(ank),pedal=list(ped),
  crank=crank,recline=rec_set),open(out+"/user.json","w"),indent=1)
