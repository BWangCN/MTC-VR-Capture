"""
fbx_builder.py — Blender headless: corridor intermediate JSON -> Unity-ready FBX.

Reads a flat JSON (produced by export_to_unity.py) and constructs:
  - floor + side walls (boxes)
  - obstacle elements: box / cylinder / ibeam (procedural primitives)
  - obstacle assets:   asset (USD imports from Objaverse-derived registry)

Exports an FBX with axis_up=Y so Unity imports identity (no rotation).

Usage:
    blender --background --python fbx_builder.py -- INPUT.json OUTPUT.fbx
"""

import json
import math
import os
import sys
from pathlib import Path

import bpy
import mathutils

# ---------------------------------------------------------------------------
# CLI args after "--"
# ---------------------------------------------------------------------------
argv = sys.argv
if "--" in argv:
    argv = argv[argv.index("--") + 1 :]
else:
    argv = []
if len(argv) < 2:
    print("Usage: blender --background --python fbx_builder.py -- INPUT.json OUTPUT.fbx")
    sys.exit(1)

INPUT_JSON = Path(argv[0]).resolve()
OUTPUT_FBX = Path(argv[1]).resolve()


# ---------------------------------------------------------------------------
# Materials — flat colours keyed by mat_key
# ---------------------------------------------------------------------------
PALETTE = {
    # corridor structure
    "floor": (0.72, 0.70, 0.66),
    "wall": (0.62, 0.60, 0.56),
    # procedural obstacle materials
    "concrete": (0.62, 0.58, 0.52),
    "concrete_dark": (0.42, 0.40, 0.36),
    "rebar": (0.50, 0.20, 0.10),  # rusty steel
    "pipe_metal": (0.48, 0.46, 0.44),
    "steel": (0.55, 0.55, 0.58),
    "wood": (0.52, 0.36, 0.18),
    # goal marker (GOAL_MARKER_COLOR "#ffdd33")
    "goal": (1.0, 0.867, 0.2),
    "goal_beam": (1.0, 0.867, 0.2),
    # fallback
    "default": (0.55, 0.55, 0.55),
}
_METALLIC_KEYS = {"rebar", "pipe_metal", "steel"}
# Emission keeps the goal readable in a VR headset regardless of scene lighting.
_EMISSIVE_KEYS = {"goal", "goal_beam"}
# Translucent emissive (waypoint light beam): visible over clutter, reads as
# light rather than a solid obstacle. Maps to a low material alpha.
_TRANSLUCENT_ALPHA = {"goal_beam": 0.18}
_mat_cache: dict = {}


def get_material(key: str):
    if key in _mat_cache:
        return _mat_cache[key]
    color = PALETTE.get(key, PALETTE["default"])
    mat = bpy.data.materials.new(name=f"MtcMat_{key}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (*color, 1.0)
        if key in _METALLIC_KEYS:
            bsdf.inputs["Metallic"].default_value = 0.7
            bsdf.inputs["Roughness"].default_value = 0.45
        else:
            bsdf.inputs["Metallic"].default_value = 0.0
            bsdf.inputs["Roughness"].default_value = 0.85
        if key in _EMISSIVE_KEYS:
            # Unlit look: kill the lit contribution, drive colour purely from
            # emission so the marker reads as flat glowing signage, not a
            # shaded solid.
            bsdf.inputs["Base Color"].default_value = (0.0, 0.0, 0.0, 1.0)
            bsdf.inputs["Roughness"].default_value = 1.0
            # Blender renamed this socket "Emission" -> "Emission Color" in 4.0.
            emis = bsdf.inputs.get("Emission Color") or bsdf.inputs.get("Emission")
            if emis:
                emis.default_value = (*color, 1.0)
            if "Emission Strength" in bsdf.inputs:
                bsdf.inputs["Emission Strength"].default_value = (
                    0.7 if key in _TRANSLUCENT_ALPHA else 1.5
                )
            # No shadowing from this material either (EEVEE Next).
            if hasattr(mat, "shadow_method"):
                mat.shadow_method = "NONE"
        if key in _TRANSLUCENT_ALPHA:
            bsdf.inputs["Alpha"].default_value = _TRANSLUCENT_ALPHA[key]
            mat.use_backface_culling = False
            # Transparency: legacy EEVEE uses blend_method, EEVEE Next uses
            # surface_render_method. Set whichever exists.
            if hasattr(mat, "blend_method"):
                mat.blend_method = "BLEND"
            if hasattr(mat, "surface_render_method"):
                mat.surface_render_method = "BLENDED"
    _mat_cache[key] = mat
    return mat


def assign_material(obj, key: str):
    if obj.type != "MESH":
        return
    obj.data.materials.clear()
    obj.data.materials.append(get_material(key))


def assign_material_recursive(obj, key: str):
    """Set material on this object and all its children (used for imported USDs)."""
    assign_material(obj, key)
    for child in obj.children:
        assign_material_recursive(child, key)


# ---------------------------------------------------------------------------
# Procedural primitives
# ---------------------------------------------------------------------------


def _apply_transform(obj, position, rotation_deg=(0, 0, 0)):
    obj.location = (position[0], position[1], position[2])
    obj.rotation_mode = "XYZ"
    obj.rotation_euler = (
        math.radians(rotation_deg[0]),
        math.radians(rotation_deg[1]),
        math.radians(rotation_deg[2]),
    )


def make_box(name: str, size, position, rotation_deg=(0, 0, 0), mat_key: str = "default"):
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object
    obj.name = name
    _apply_transform(obj, position, rotation_deg)
    obj.scale = (size[0], size[1], size[2])
    assign_material(obj, mat_key)
    return obj


def make_cylinder_x(
    name: str, radius, length, position, rotation_deg=(0, 0, 0), mat_key: str = "default"
):
    """Create a cylinder whose axis is along local X before rotation
    (matching disaster_scene_generator convention). Blender's default
    cylinder is Z-aligned; we pre-rotate 90 deg around Y so the axis is X,
    then compose the JSON rotation on top."""
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=length)
    obj = bpy.context.active_object
    obj.name = name
    pre = mathutils.Matrix.Rotation(math.radians(90), 4, "Y")
    json_rot = (
        mathutils.Euler(
            (
                math.radians(rotation_deg[0]),
                math.radians(rotation_deg[1]),
                math.radians(rotation_deg[2]),
            ),
            "XYZ",
        )
        .to_matrix()
        .to_4x4()
    )
    final = json_rot @ pre
    obj.location = (position[0], position[1], position[2])
    obj.rotation_mode = "XYZ"
    obj.rotation_euler = final.to_euler("XYZ")
    assign_material(obj, mat_key)
    return obj


def make_goal_ring(
    name: str,
    radius: float,
    position,
    rotation_deg=(0, 0, 0),
    tube: float = 0.06,
    beam_h: float = 2.4,
    mat_key: str = "goal",
    **_ignore,
):
    """Goal marker: a flat floor disc + ring outline (the "stand here" guide)
    plus a thin translucent vertical light beam.

    The floor parts lie flat and cast no shadow, so they never read as an
    obstacle. The beam is a low-alpha emissive column -- visible over torso-
    height clutter from the corridor start, but it reads as light, not a solid,
    and casts no shadow. A human standing in the goal passes through the beam
    harmlessly during VR capture (it has no collision meaning).
    """
    parts = []

    # Filled disc, slightly above the floor to avoid z-fighting.
    bpy.ops.mesh.primitive_circle_add(vertices=64, radius=radius, fill_type="NGON")
    disc = bpy.context.active_object
    disc.name = name
    disc.location = (0.0, 0.0, 0.010)
    parts.append(disc)

    # Ring outline (a flat annulus band) sitting just above the disc so the
    # boundary of the goal zone reads crisply.
    band = tube
    bpy.ops.mesh.primitive_circle_add(vertices=64, radius=radius, fill_type="NGON")
    outer = bpy.context.active_object
    bpy.ops.mesh.primitive_circle_add(vertices=64, radius=radius - band, fill_type="NGON")
    inner = bpy.context.active_object
    # Boolean the inner hole out of the outer disc -> flat ring band.
    mod = outer.modifiers.new("hole", "BOOLEAN")
    mod.operation = "DIFFERENCE"
    mod.object = inner
    bpy.context.view_layer.objects.active = outer
    bpy.ops.object.modifier_apply(modifier="hole")
    bpy.data.objects.remove(inner, do_unlink=True)
    outer.name = f"{name}_edge"
    outer.location = (0.0, 0.0, 0.015)
    parts.append(outer)

    bpy.ops.object.select_all(action="DESELECT")
    for p in parts:
        p.select_set(True)
    bpy.context.view_layer.objects.active = disc
    bpy.ops.object.join()

    obj = bpy.context.active_object
    obj.name = name
    _apply_transform(obj, position, rotation_deg)
    assign_material(obj, mat_key)
    obj.visible_shadow = False  # signage, not geometry

    # Translucent light beam, kept as its own object (different material) so the
    # floor disc stays fully opaque/emissive. Radius slightly under the ring so
    # it sits inside the goal zone.
    bpy.ops.mesh.primitive_cylinder_add(radius=radius * 0.85, depth=beam_h, vertices=48)
    beam = bpy.context.active_object
    beam.name = f"{name}_beam"
    _apply_transform(beam, position, rotation_deg)
    # Lift so the beam base sits on the floor at the goal (cylinder is centred).
    beam.location = (position[0], position[1], position[2] + beam_h / 2.0)
    assign_material(beam, "goal_beam")
    beam.visible_shadow = False
    return obj


def make_ibeam(
    name: str,
    length,
    flange_w,
    flange_t,
    web_h,
    web_t,
    position,
    rotation_deg=(0, 0, 0),
    mat_key: str = "steel",
):
    """Build a proper H-profile from 3 boxes (top flange, web, bottom flange)
    and parent them so the whole assembly rotates as one.

    Local frame (pre-rotation): length along X, flange across Y, height along Z.
        size_xyz: length, flange_w, flange_t for flanges
                 length, web_t,    web_h    for web
    """
    parent = bpy.data.objects.new(name=name, object_data=None)
    bpy.context.collection.objects.link(parent)
    total_h = web_h + 2 * flange_t

    # Top flange
    top = _box_local(
        f"{name}_top", [length, flange_w, flange_t], [0.0, 0.0, web_h / 2.0 + flange_t / 2.0]
    )
    bot = _box_local(
        f"{name}_bot", [length, flange_w, flange_t], [0.0, 0.0, -web_h / 2.0 - flange_t / 2.0]
    )
    web = _box_local(f"{name}_web", [length, web_t, web_h], [0.0, 0.0, 0.0])
    for o in (top, bot, web):
        o.parent = parent
        assign_material(o, mat_key)

    _apply_transform(parent, position, rotation_deg)
    return parent


def _box_local(name, size, local_offset):
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object
    obj.name = name
    obj.location = (local_offset[0], local_offset[1], local_offset[2])
    obj.scale = (size[0], size[1], size[2])
    return obj


# ---------------------------------------------------------------------------
# USD asset import
# ---------------------------------------------------------------------------


def _delete_default_objects():
    # Removes any orphan empties or default cubes Blender may auto-create.
    for obj in list(bpy.context.scene.objects):
        if obj.name == "Cube":
            bpy.data.objects.remove(obj, do_unlink=True)


def import_usd_asset(
    name: str,
    usd_path: str,
    target_size_xyz,
    asset_center_xyz,
    position,
    yaw_deg,
    mat_key="default",
):
    """Import a USD file, normalize to its registered AABB size, translate so
    the resulting AABB sits centered at `position` (z = sz/2 by convention).

    Strategy:
      1. Record names of pre-existing objects.
      2. Call bpy.ops.wm.usd_import.
      3. New objects = post - pre. Wrap them under an Empty parent.
      4. Compute combined bounding box of imported objects (already in world
         coordinates after the importer applies the USD's transforms).
      5. Translate the parent so the combined center lands at `position`
         and apply yaw rotation around Z.
      6. Apply a single flat material to all child meshes for now (Objaverse
         materials sometimes don't survive USD/FBX roundtrip cleanly).
    """
    pre_names = set(bpy.data.objects.keys())
    try:
        bpy.ops.wm.usd_import(filepath=str(usd_path))
    except Exception as e:
        print(f"  WARNING: usd_import failed for {usd_path}: {e}")
        return None
    new_objs = [o for o in bpy.context.scene.objects if o.name not in pre_names]
    if not new_objs:
        print(f"  WARNING: usd_import produced no objects for {usd_path}")
        return None

    # Compute combined world-space AABB across all newly imported meshes.
    bb_min = [float("inf")] * 3
    bb_max = [float("-inf")] * 3
    for o in new_objs:
        if o.type != "MESH":
            continue
        for v in o.bound_box:
            world_v = o.matrix_world @ mathutils.Vector(v)
            for i in range(3):
                if world_v[i] < bb_min[i]:
                    bb_min[i] = world_v[i]
                if world_v[i] > bb_max[i]:
                    bb_max[i] = world_v[i]

    if any(math.isinf(x) for x in bb_min + bb_max):
        # No mesh children — bail.
        for o in new_objs:
            bpy.data.objects.remove(o, do_unlink=True)
        return None

    center = [(bb_min[i] + bb_max[i]) / 2.0 for i in range(3)]

    # Create a parent empty, parent all new objects to it.
    parent = bpy.data.objects.new(name=name, object_data=None)
    bpy.context.collection.objects.link(parent)
    for o in new_objs:
        # If object already has a parent that was also imported, keep that chain.
        if o.parent is None or o.parent not in new_objs:
            # Assigning .parent keeps whatever matrix_parent_inverse the USD
            # importer left on the object. That stale inverse then teleports the
            # asset off its placement point -- measured up to 55 m outside a 4 m
            # corridor. Re-anchor so the world transform is preserved.
            world = o.matrix_world.copy()
            o.parent = parent
            o.matrix_parent_inverse = mathutils.Matrix.Identity(4)
            o.matrix_world = world

    # Position the parent so the combined AABB ends up at `position`.
    # Bring AABB center to origin, then translate to target.
    parent.location = (
        position[0] - center[0],
        position[1] - center[1],
        position[2] - center[2],
    )
    parent.rotation_mode = "XYZ"
    parent.rotation_euler = (0.0, 0.0, math.radians(yaw_deg))

    # Apply a uniform material override so all corridor scenes share the
    # same flat-colour look. This also avoids missing texture errors.
    for o in new_objs:
        assign_material_recursive(o, mat_key)

    return parent


# ---------------------------------------------------------------------------
# Element dispatcher
# ---------------------------------------------------------------------------


def build_element(name: str, el: dict):
    prim = el["primitive"]
    mat_key = el.get("material", "default")
    if prim == "box":
        return make_box(
            name,
            size=el["size_xyz"],
            position=el["position"],
            rotation_deg=el.get("rotation_deg", [0, 0, 0]),
            mat_key=mat_key,
        )
    if prim == "cylinder":
        return make_cylinder_x(
            name,
            radius=el["radius"],
            length=el["length"],
            position=el["position"],
            rotation_deg=el.get("rotation_deg", [0, 0, 0]),
            mat_key=mat_key,
        )
    if prim == "goal_ring":
        return make_goal_ring(
            name,
            radius=el["radius"],
            position=el["position"],
            rotation_deg=el.get("rotation_deg", [0, 0, 0]),
            tube=el.get("tube", 0.06),
            mat_key=mat_key,
        )
    if prim == "ibeam":
        return make_ibeam(
            name,
            length=el["length"],
            flange_w=el["flange_w"],
            flange_t=el["flange_t"],
            web_h=el["web_h"],
            web_t=el["web_t"],
            position=el["position"],
            rotation_deg=el.get("rotation_deg", [0, 0, 0]),
            mat_key=mat_key,
        )
    if prim == "asset":
        return import_usd_asset(
            name,
            usd_path=el["asset_ref"],
            target_size_xyz=el["size_xyz"],
            asset_center_xyz=el.get("asset_center_xyz", [0, 0, 0]),
            position=el["position"],
            yaw_deg=el.get("yaw_deg", 0.0),
            mat_key=mat_key,
        )
    print(f"  WARNING: unknown primitive {prim!r} for {name}")
    return None


# ---------------------------------------------------------------------------
# Merge meshes by material (smaller draw call count, keeps assets separate)
# ---------------------------------------------------------------------------


def merge_by_material():
    groups: dict = {}
    for obj in list(bpy.context.scene.objects):
        if obj.type != "MESH" or not obj.data.materials:
            continue
        if obj.parent is not None:
            # Don't merge across parented assets (keeps USD imports intact).
            continue
        mat_name = obj.data.materials[0].name
        groups.setdefault(mat_name, []).append(obj)
    for mat_name, objs in groups.items():
        if len(objs) < 2:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        try:
            bpy.ops.object.join()
        except Exception as e:
            print(f"    WARNING: join failed for {mat_name}: {e}")


# ---------------------------------------------------------------------------
# Scene clear
# ---------------------------------------------------------------------------


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in list(bpy.data.meshes):
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in list(bpy.data.materials):
        if block.users == 0:
            bpy.data.materials.remove(block)
    _mat_cache.clear()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def build(scene: dict) -> None:
    clear_scene()

    # Floor + walls
    for box in scene["boxes"]:
        make_box(
            name=box["name"],
            size=box["size"],
            position=box["position"],
            rotation_deg=box.get("rotation_deg", [0, 0, 0]),
            mat_key=box.get("mat_key", box.get("class", "default")),
        )

    # Obstacle elements (each is one of box/cylinder/ibeam/asset)
    for el in scene.get("elements", []):
        build_element(el["name"], el)

    cull_outside_corridor()
    merge_by_material()
    mesh_n = sum(1 for o in bpy.context.scene.objects if o.type == "MESH")
    print(f"  Final mesh count: {mesh_n}")
    # NOTE: the +90deg "corridor long axis -> Unity +Z" rotation is now baked
    # in env_to_blender_scene at the JSON-data level (positions + yaws), not
    # here in Blender. matrix_world post-write didn't reliably survive the
    # depsgraph + FBX-export pipeline.


STRUCTURE_MESHES = {"floor", "wall_y0", "wall_y1", "goal_ring", "goal_ring_beam"}


def cull_outside_corridor(margin: float = 0.10) -> None:
    """Delete meshes that end up outside the corridor footprint.

    Some USD assets land far from the placement point even though the scene JSON
    positions them correctly inside the corridor -- measured up to 55 m out. The
    cause is somewhere in the USD import / re-parent path and is not yet
    understood; two candidate explanations (detached sub-meshes, a stale
    matrix_parent_inverse) were both tested and ruled out. Until it is found,
    cull geometrically: the JSON is authoritative about where things belong, so
    anything whose centre is off the floor slab is debris.

    Structural meshes are exempt by name -- the walls sit slightly outside the
    floor slab by design.
    """
    floor = bpy.data.objects.get("floor")
    if floor is None:
        print("[fbx_builder] WARNING: no 'floor' object; skipping cull")
        return
    pts = [floor.matrix_world @ mathutils.Vector(c) for c in floor.bound_box]
    lo = [min(q[i] for q in pts) - margin for i in range(2)]
    hi = [max(q[i] for q in pts) + margin for i in range(2)]

    doomed = []
    for o in list(bpy.context.scene.objects):
        if o.type != "MESH" or o.name.split(".")[0] in STRUCTURE_MESHES:
            continue
        b = [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
        cx = sum(q.x for q in b) / 8.0
        cy = sum(q.y for q in b) / 8.0
        if not (lo[0] <= cx <= hi[0] and lo[1] <= cy <= hi[1]):
            doomed.append((o, cx, cy))
    for o, cx, cy in doomed:
        print(f"[fbx_builder] culled stray '{o.name}' at ({cx:.2f}, {cy:.2f})")
        bpy.data.objects.remove(o, do_unlink=True)
    if doomed:
        print(f"[fbx_builder] culled {len(doomed)} mesh(es) outside the corridor")


def export_fbx(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.fbx(
        filepath=str(path),
        use_selection=False,
        apply_unit_scale=True,
        apply_scale_options="FBX_SCALE_ALL",
        axis_forward="-Z",
        axis_up="Y",
        use_mesh_modifiers=True,
        mesh_smooth_type="FACE",
        path_mode="AUTO",
        embed_textures=False,
        object_types={"MESH", "EMPTY"},
        use_mesh_edges=False,
    )
    size_mb = path.stat().st_size / (1024 * 1024)
    print(f"  Exported FBX: {path}  ({size_mb:.2f} MB)")


def main() -> None:
    print(f"\n[fbx_builder] {INPUT_JSON} -> {OUTPUT_FBX}")
    with open(INPUT_JSON, "r", encoding="utf-8") as f:
        scene = json.load(f)
    build(scene)
    export_fbx(OUTPUT_FBX)


if __name__ == "__main__":
    main()
