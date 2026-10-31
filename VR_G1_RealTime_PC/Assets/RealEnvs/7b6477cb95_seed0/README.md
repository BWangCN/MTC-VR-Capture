# Obstacle-course scene 7b6477cb95, layout seed 0

Exported 2026-10-30 18:20 by `tools/export_layout_scene.py` (MTC Next, commit d3682f0).

**Test data, research use only.** The room is a ScanNet++ scan (ScanNet++ Terms of Use: research only, not to be redistributed). The objects are Google Scanned Objects (CC BY 4.0) and Amazon Berkeley Objects; see each asset's `object.json` in the source repository for licence and attribution.

## Files
- `scene.glb`: what to show. Node `room` = the scanned room (1,000,000 faces, vertex colours); one node per object (`item_XX_<role>_<class>`, own textures).
- `collision.glb`: physics proxies. `room_collision` (60k faces) + convex hulls per object (`item_XX_hull_i`).
- `markers.glb` (optional overlay): green disc = route start, red disc = route goal (radius 0.3 m), yellow line = the planned route, blue disc = the goal spot for the mug (radius 0.1 m).
- `layout.json`, `task.json`: the full layout and task in the scene frame (Z up).

## Frame and units
- Metres. glTF convention: right-handed, **Y up**, floor at **y = 0**.
- From the scene frame of `layout.json` (ScanNet++ mesh frame, Z up, floor at z = 0.0769 m): `x_gltf = y_scene`, `y_gltf = z_scene - floor_z`, `z_gltf = x_scene`.
- Unity (glTFast or UnityGLTF, which flip x): `x_unity = -y_scene`, `y_unity = z_scene - floor_z`, `z_unity = x_scene` = the MTC Next Unity frame of this scene. Vertex colours need a shader that uses them (glTFast's default does).
- Blender's glTF importer converts Y up to Z up: Blender `(x, y, z) = (y_scene, -x_scene, z_scene - floor_z)`.

## Task
- Instruction: Walk from the start through the gates (walk, squeeze), pick up the mug, put it on the goal spot, then walk to the goal.
- Route start (3.353, 0.000, 3.514), goal (0.553, 0.000, 4.164) (glTF), length 7.20 m.
- Mug goal spot (0.858, 0.473, 0.696) on the bookshelf (glTF).

## Gates (in route order)

| # | Mode | Variant | Centre (glTF) | Gap (m) | Clearance (m) | Bar top (m) |
|---|---|---|---|---|---|---|
| 0 | walk | furniture against the scan | (2.460, 0.000, 1.171) | 0.797 | - | - |
| 1 | squeeze | narrow_gate primitive | (1.029, 0.000, 2.327) | 0.282 | 0.170 | - |

## Objects

| Id | Role | Class | Static | Position (glTF) | Yaw about up (deg, scene) | Size (m) | Asset |
|---|---|---|---|---|---|---|---|
| item_00 | gate | cabinet | yes | (2.435, 0.008, 1.681) | -92.8 | 0.52 x 0.26 x 1.63 | `abo_B07GG2X2MX` |
| item_01 | gate | narrow_gate | yes | (1.029, 0.000, 2.327) | -13.7 | 0.40 x 0.62 x 1.60 | `narrow_gate_gap282_panel_width169` |
| item_02 | target | mug | no | (2.546, 0.714, 2.662) | 109.7 | 0.17 x 0.13 x 0.10 | `gso_Cole_Hardware_Mug_Classic_Blue` |
| item_03 | clutter_floor | shoe | no | (1.214, 0.001, 0.865) | 71.2 | 0.25 x 0.09 x 0.19 | `gso_Womens_Hikerfish_Boot_in_Linen_Leather_Sparkle_Suede_QktIyAkonrU` |
| item_04 | clutter_floor | toy | no | (2.581, 0.000, 2.096) | 53.2 | 0.09 x 0.05 x 0.14 | `gso_Playmates_nickelodeon_teenage_mutant_ninja_turtles_shredder` |
| item_05 | clutter_floor | shoe | no | (1.325, -0.000, 0.546) | 135.7 | 0.11 x 0.30 x 0.10 | `gso_Santa_Cruz_Mens_umxTczr1Ygg` |
| item_06 | clutter_floor | box | no | (1.693, -0.000, 0.374) | 153.1 | 0.07 x 0.13 x 0.02 | `gso_Big_O_Sponges_Assorted_Cellulose_12_pack` |
| item_07 | clutter_surface | box | no | (1.573, 0.750, 3.629) | 147.8 | 0.14 x 0.03 x 0.09 | `gso_Super_Mario_Kart_Super_Nintendo_Entertainment_System` |
| item_08 | clutter_surface | book | no | (0.855, 0.052, 1.003) | -102.3 | 0.14 x 0.05 x 0.21 | `gso_Eat_to_Live_The_Amazing_NutrientRich_Program_for_Fast_and_Sustained_Weight_Loss_Revised_Edition_Book` |
| item_09 | clutter_surface | mug | no | (1.451, 0.750, 3.837) | -107.2 | 0.13 x 0.10 x 0.12 | `gso_Threshold_Porcelain_Coffee_Mug_All_Over_Bead_White` |
| item_10 | clutter_surface | box | no | (1.322, 0.749, 3.677) | 26.0 | 0.09 x 0.05 x 0.12 | `gso_Office_Depot_Dell_Series_1_Remanufactured_Ink_Cartridge_TriColor` |

Static objects (furniture, gate panels) stand on the floor. The others (target mug, clutter) were settled with MuJoCo and are meant to be movable (kinematic until grabbed in MTC Next).
