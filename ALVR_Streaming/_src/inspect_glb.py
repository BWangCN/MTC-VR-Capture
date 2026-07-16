import bpy, sys
inp = sys.argv[sys.argv.index("--")+1:][0]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=inp)
print("=== MESHES ===")
for o in bpy.data.objects:
    if o.type == 'MESH':
        m = o.data
        cols = [c.name for c in getattr(m, 'color_attributes', [])]
        print("MESH:", o.name, "| polys:", len(m.polygons), "| UVmaps:", [uv.name for uv in m.uv_layers], "| vertexcolors:", cols)
print("=== MATERIALS ===")
for mat in bpy.data.materials:
    tex = []
    if mat.use_nodes:
        for n in mat.node_tree.nodes:
            if n.type == 'TEX_IMAGE' and n.image:
                tex.append(n.image.name)
    print("MAT:", mat.name, "| textures:", tex)
print("=== IMAGES ===")
for img in bpy.data.images:
    print("IMG:", img.name, "| size:", tuple(img.size), "| has_data:", img.has_data)
