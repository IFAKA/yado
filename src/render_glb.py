# blender -b -P src/render_glb.py -- model.glb out_prefix [views...]   views: name:azimuth:elevation:scale:cx:cy:cz
import bpy, sys, math, mathutils
argv = sys.argv[sys.argv.index("--") + 1:]
glb, prefix = argv[0], argv[1]; views = argv[2:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=glb)
sc = bpy.context.scene
sc.render.engine = "BLENDER_WORKBENCH"
sh = sc.display.shading; sh.light = "STUDIO"; sh.color_type = "MATERIAL"; sh.show_cavity = False; sh.show_object_outline = True
sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
sc.world = bpy.data.worlds.new("w"); sc.world.color = (1, 1, 1)
sc.render.film_transparent = False
sc.view_settings.view_transform = "Standard"
# glTF import is Y-up: blender converts so that Z up; our X fwd Y left Z up was exported raw and importer rotates; we exported with identity -> handled in export (see build)
cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam")); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = "ORTHO"; cam.data.clip_end = 100000; cam.data.clip_start = 1
for v in views:
    name, az, el, scale, cx, cy, cz = v.split(":")
    az, el = math.radians(float(az)), math.radians(float(el))
    c = mathutils.Vector((float(cx), float(cy), float(cz))) * 0.001
    d = mathutils.Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = c + d * 20
    cam.rotation_euler = (c - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam.data.ortho_scale = float(scale) * 0.001
    sc.render.filepath = f"{prefix}_{name}.png"
    bpy.ops.render.render(write_still=True)
