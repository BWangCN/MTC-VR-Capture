import bpy, sys, os
argv = sys.argv[sys.argv.index("--")+1:]
inp, outp = argv[0], argv[1]
outdir = os.path.dirname(outp)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=inp)
# explicitly write each packed image to a PNG sidecar next to the FBX
for img in bpy.data.images:
    if img.has_data and img.size[0] > 0:
        base = os.path.splitext(bpy.path.basename(img.name))[0]
        path = os.path.join(outdir, base + ".png")
        img.filepath_raw = path
        img.file_format = 'PNG'
        img.save()
        print("SAVED_TEX", path)
bpy.ops.export_scene.fbx(filepath=outp, path_mode='COPY', embed_textures=False)
print("DONE", outp)
