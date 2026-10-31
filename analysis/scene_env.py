"""The obstacle-course environment (Assets/RealEnvs/<scene>_seed<N>/) in its glTF frame.

glTF frame: right-handed, Y up, metres, floor at y = 0. The Unity world of the course scene is this frame with x
mirrored (Environment root at the origin, scale 1); `mocaprec.unity_to_gltf` converts Unity points into it.
Scene-frame positions of layout.json / task.json (Z up, floor at floor_z_m) convert as
(x, y, z)_gltf = (y_scene, z_scene - floor_z, x_scene)   (README.md of the environment package).
"""
import json
import os
from dataclasses import dataclass, field

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ENV = os.path.join(REPO, "VR_G1_RealTime_PC", "Assets", "RealEnvs", "7b6477cb95_seed0")


@dataclass
class Part:
    name: str
    V: np.ndarray                    # (n, 3) float32, glTF frame
    F: np.ndarray                    # (m, 3) int32
    C: np.ndarray                    # (n, 3) uint8 vertex colours
    kind: str                        # room | object | gate | collision | route


@dataclass
class Environment:
    path: str
    parts: list
    route_gltf: np.ndarray           # (k, 3) planned route polyline
    start_gltf: np.ndarray
    goal_gltf: np.ndarray
    gates: list                      # dicts: mode, centre_gltf, gap_m, ...
    floor_z: float
    meta: dict = field(default_factory=dict)


def scene_to_gltf(p, floor_z):
    p = np.asarray(p, float)
    return np.stack([p[..., 1], p[..., 2] - floor_z, p[..., 0]], -1)


def cluster_decimate(V, F, C, cell):
    """Vertex clustering: merge vertices per cubic cell (mean position and colour), drop collapsed faces."""
    key = np.floor(V / cell).astype(np.int64)
    key -= key.min(0)
    dims = key.max(0) + 1
    lin = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    uniq, inv = np.unique(lin, return_inverse=True)
    n = len(uniq)
    cnt = np.bincount(inv, minlength=n).astype(np.float64)
    Vn = np.stack([np.bincount(inv, V[:, k], n) / cnt for k in range(3)], 1)
    Cn = np.stack([np.bincount(inv, C[:, k].astype(np.float64), n) / cnt for k in range(3)], 1)
    Fn = inv[F]
    keep = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
    Fn = Fn[keep]
    Fs = np.sort(Fn, 1)
    _, first = np.unique(Fs, axis=0, return_index=True)
    Fn = Fn[np.sort(first)]
    used = np.unique(Fn)
    remap = -np.ones(n, np.int64)
    remap[used] = np.arange(len(used))
    return Vn[used].astype(np.float32), remap[Fn].astype(np.int32), np.clip(Cn[used], 0, 255).astype(np.uint8)


def _colours(mesh):
    """Per-vertex RGB (uint8); textures are sampled at the vertex UVs."""
    vis = mesh.visual
    try:
        if vis.kind == "texture":
            vis = vis.to_color()
        c = np.asarray(vis.vertex_colors)[:, :3]
        if len(c) == len(mesh.vertices):
            return c.astype(np.uint8)
    except Exception:
        pass
    return np.full((len(mesh.vertices), 3), 160, np.uint8)


def load_environment(path=DEFAULT_ENV, room_cell=0.02, with_collision=True):
    import trimesh
    with open(os.path.join(path, "layout.json")) as f:
        layout = json.load(f)
    with open(os.path.join(path, "task.json")) as f:
        task = json.load(f)
    floor_z = float(layout["floor_z_m"])
    parts = []
    sc = trimesh.load(os.path.join(path, "scene.glb"), force="scene")
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        m = sc.geometry[g]
        V = trimesh.transform_points(np.asarray(m.vertices, float), T)
        C = _colours(m)
        F = np.asarray(m.faces, np.int64)
        if node == "room":
            V, F, C = cluster_decimate(V, F, C, room_cell)
            kind = "room"
        else:
            kind = "gate" if "_gate_" in node else "object"
        parts.append(Part(node, np.asarray(V, np.float32), np.asarray(F, np.int32), C, kind))
    if with_collision:
        cc = trimesh.load(os.path.join(path, "collision.glb"), force="scene")
        for node in cc.graph.nodes_geometry:
            T, g = cc.graph[node]
            m = cc.geometry[g]
            V = trimesh.transform_points(np.asarray(m.vertices, float), T).astype(np.float32)
            parts.append(Part(node, V, np.asarray(m.faces, np.int32),
                              np.full((len(V), 3), (230, 60, 60), np.uint8), "collision"))
    mk = trimesh.load(os.path.join(path, "markers.glb"), force="scene")
    for node in mk.graph.nodes_geometry:
        T, g = mk.graph[node]
        m = mk.geometry[g]
        V = trimesh.transform_points(np.asarray(m.vertices, float), T).astype(np.float32)
        parts.append(Part(node, V, np.asarray(m.faces, np.int32), _colours(m), "route"))
    route = task["route"]
    gates = []
    for g in layout.get("gates", []):
        gg = dict(g)
        for k in list(gg):
            if k.endswith("_m") and isinstance(gg[k], list) and len(gg[k]) == 3 and k.startswith("center"):
                gg["centre_gltf"] = scene_to_gltf(gg[k], floor_z).tolist()
        gates.append(gg)
    return Environment(path, parts, scene_to_gltf(route["waypoints_m"], floor_z), scene_to_gltf(route["start_m"], floor_z),
                       scene_to_gltf(route["goal_m"], floor_z), gates, floor_z,
                       {"scene_id": layout.get("scene_id"), "seed": layout.get("seed"), "instruction": task.get("instruction"),
                        "goal_spot_gltf": scene_to_gltf(task["goal_spot"]["center_m"] if "center_m" in task.get("goal_spot", {})
                                                        else task["goal_region"]["center_m"], floor_z).tolist()})
