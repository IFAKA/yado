"""Stage 2: 3D layout (envelopes only, not final parts). X forward, Y left, Z up, origin on ground under front axle line."""
import sys, json, math, yaml
from build123d import *

P = yaml.safe_load(open(sys.argv[1])); out = sys.argv[2]
U = json.load(open(out+"/../stage1/user.json"))
v = lambda s,k: P[s][k]["value"]
WB=v("layout","wheelbase"); T=v("layout","front_track_cc"); tw=v("wheels","tyre_width")
hip=Vector(v("layout","hip_x_from_front_axle"),0,U["hip_y"])
parts={}

def wheel(od, x, y):
    t = Torus((od-tw)/2, tw/2) ; t = Rot(90,0,0)*t
    return Pos(x,y,od/2)*t
parts["wheels_front"] = Compound([wheel(v("wheels","front_tyre_od"),0,s*T/2) for s in (1,-1)])
parts["wheel_rear"]   = wheel(v("wheels","rear_tyre_od"),-WB,0)

# seat: pan + back (envelopes)
rc=math.radians(U["recline"]); seat_w=450
pan = Pos(hip.X+120, 0, v("layout","seat_pan_height")-15)*Box(520, seat_w, 30)
back = Pos(hip.X,0,hip.Z-90)*Rot(0,-U["recline"],0)*Pos(0,0,330)*Box(30, seat_w, 700)
parts["seat"] = Compound([pan, back])

# rider (envelope)
def limb(p, q, d):
    p,q=Vector(*p),Vector(*q); L=(q-p).length; ax=(q-p).normalized()
    c=Cylinder(d/2,L); rot=Rotation(*[0,0,0]); 
    # orient Z axis along ax
    ang=math.degrees(math.acos(max(-1,min(1,ax.Z)))); axis=Vector(0,0,1).cross(ax)
    c = Rot(0,0,0)*c if axis.length<1e-9 else Plane(origin=(0,0,0), z_dir=tuple(ax)).location*c
    return Pos(*((p+q)/2))*Pos(0,0,0)*(Location(Vector(0,0,0))*c) if False else Location((p+q)/2)*(Plane(origin=(0,0,0),z_dir=tuple(ax)).location*Cylinder(d/2,L))
H=U["H"]; legs=[]
knee=hip+Vector(U["knee"][0],0,U["knee"][1]); ank=hip+Vector(U["ankle"][0],0,U["ankle"][1])
for s in (1,-1):
    off=Vector(0,s*110,0)
    legs += [limb(hip+off,knee+off,110), limb(knee+off,ank+off,80),
             limb(ank+off,hip+Vector(U['pedal'][0]+70,off.Y,U['pedal'][1]),70)]
tdir=Vector(-math.sin(rc),0,math.cos(rc)); neck=hip+tdir*0.38*H; head=hip+tdir*0.50*H-tdir*100
nrm=Vector(math.cos(rc),0,math.sin(rc))*110; torso=limb(hip+nrm,neck+nrm,300)
parts["rider_1720_envelope"]=Compound(legs+[torso, Pos(*head)*Sphere(100)])

# drive envelope: BBS01B placeholder (TO CONFIRM) + cranks + boom line
bb=Vector(hip.X+U["bb"][0],0,U["bb"][1]+hip.Z-0)  # bb[1] is dy relative hip
bb=Vector(hip.X+U["bb"][0],0,hip.Z+U["bb"][1])
motor=Pos(*bb)*Rot(90,0,0)*Cylinder(50,150)
ped=hip+Vector(U["pedal"][0],0,U["pedal"][1])
cranks=[limb(bb+Vector(0,s*85,0),(bb+Vector(0,s*85,0)+ (Vector(ped.X-bb.X,0,ped.Z-bb.Z)*(1 if s>0 else -1))),24) for s in (1,-1)]
parts["drive_BBS01B_envelope"]=Compound([motor]+cranks)
print("BB world:",bb)

crank_len=U['crank']
# canopy envelope: side profile (x,z) extruded across width, shoulder width 640+skin
roof=1250+20
nose=bb.X+crank_len+v('canopy','toe_beyond_pedal_axle')+v('canopy','nose_clearance_ahead_of_foot')
prof=[(nose,420),(nose,640),(hip.X+800,roof-170),(hip.X+300,roof),(hip.X-700,roof),(hip.X-760,520)]
w=720
with BuildPart() as cp:
    with BuildSketch(Plane.XZ) as sk: Polygon(*prof, align=None)
    extrude(amount=w/2, both=True)
parts["canopy_envelope"]=cp.part
parts["belly_tub_envelope"]=Pos((nose+hip.X-100)/2,0,360)*Box(nose-(hip.X-100),500,120)
# main frame centre line (placeholder, 30mm tube): rear axle -> front axle region -> BB
line=[(limb((-WB,0,250),(hip.X,0,hip.Z-120),30)),limb((hip.X,0,hip.Z-120),(0,0,330),30),limb((0,0,330),tuple(bb),30)]
parts["frame_centreline_placeholder"]=Compound(line)

for n,p in parts.items(): export_stl(p, f"{out}/{n}.stl")
export_step(Compound(list(parts.values())), out+"/layout.step")
bbx=Compound(list(parts.values())).bounding_box()
print("overall X",bbx.min.X,bbx.max.X,"len",bbx.size.X,"Y width",bbx.size.Y,"height",bbx.size.Z)
