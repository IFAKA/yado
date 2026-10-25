# blender -b -P render_layout.py -- <stl_dir> <out_png_prefix>
import bpy, sys, math, os
from mathutils import Vector
d, pre = sys.argv[sys.argv.index("--")+1:][:2]
bpy.ops.wm.read_factory_settings(use_empty=True)
cols={"wheels_front":(.1,.1,.1),"wheel_rear":(.1,.1,.1),"seat":(.6,.3,.1),"rider_1720_envelope":(.2,.5,.9),
 "drive_BBS01B_envelope":(.9,.5,0),"canopy_envelope":(.7,.9,1),"belly_tub_envelope":(.6,.8,.9),"frame_centreline_placeholder":(.8,0,0)}
for n,c in cols.items():
    bpy.ops.wm.stl_import(filepath=f"{d}/{n}.stl"); o=bpy.context.object
    m=bpy.data.materials.new(n); m.diffuse_color=(*c,0.25 if "envelope" in n and n.startswith(("canopy","belly")) else 1); o.data.materials.append(m)
    o.active_material.diffuse_color=(*c,0.25 if "envelope" in n and n.startswith(("canopy","belly")) else 1)
# ground grid plane
bpy.ops.mesh.primitive_plane_add(size=4000,location=(-600,0,-1)); g=bpy.context.object
gm=bpy.data.materials.new("g"); gm.diffuse_color=(.85,.85,.85,1); g.data.materials.append(gm)
sc=bpy.context.scene; sc.render.engine="BLENDER_WORKBENCH"
sc.display.shading.light="STUDIO"; sc.display.shading.color_type="MATERIAL"; sc.display.shading.show_object_outline=True
sc.render.resolution_x=1600; sc.render.resolution_y=800; sc.render.film_transparent=False
bpy.ops.object.camera_add(); cam=bpy.context.object; sc.camera=cam
cam.data.type="ORTHO"; cam.data.clip_end=100000; cam.data.clip_start=1
sc.world=bpy.data.worlds.new("w"); sc.world.color=(1,1,1)
tgt=Vector((-550,0,650))
views={"side":((-550,-6000,650),2700),"top":((-550,0,6000),2700),"front":((6000,0,650),1500),"iso":((2500,-3500,2200),3000)}
for n,(loc,sc_o) in views.items():
    cam.location=loc; cam.data.ortho_scale=sc_o
    cam.rotation_euler=(tgt-Vector(loc)).to_track_quat('-Z','Y').to_euler()
    if n=="top": sc.render.resolution_y=800
    sc.render.filepath=f"{pre}_{n}.png"; bpy.ops.render.render(write_still=True)
