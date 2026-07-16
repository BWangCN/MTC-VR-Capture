import bpy, sys, os
argv = sys.argv[sys.argv.index("--")+1:]
inp, outp = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=inp)
os.makedirs(os.path.dirname(outp), exist_ok=True)
bpy.ops.export_scene.fbx(filepath=outp, path_mode='COPY', embed_textures=True)
print("CONVERT_OK", outp)
