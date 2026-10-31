"""Markers inside obstacles of the VR scene, per take (as Unity showed them: player scale, aligned to the room).

    python analysis/contact_check.py analysis/data/raw/2026-10-30_recordings/recordings [--out analysis/data/out]

Obstacles are the collision proxies Unity used for contact feedback (collision.glb):
- object hulls (item_XX_hull_i, convex): signed distance of a marker centre = max over hull faces of n.p - d
  (exact inside the hull; outside it is a lower bound, which is fine for a contact threshold);
- the room scan (room_collision): unsigned distance to the closest of points sampled every ~8 mm on its surface
  (accurate to about half the spacing); floor triangles (normal up, below 5 cm) are left out, as in Unity. The
  scan's triangle normals are not consistently oriented, so inside/outside cannot be told for the scan.
Contact = a marker centre within 2 cm, Unity's contactRadiusMeters in the course scene.
Markers checked: unlabeled markers kept by the headset box, plus the rigid-body members Motive actually saw
(Pico, and the body markers Motive gave to the stray v4w2 rigid body).
Reported per obstacle: closest approach and when, frames in contact; for hulls also frames with a marker centre
inside / deeper than 2 cm inside.
A marker centre sits about 1 cm off the skin, so a centre inside means the body overlapped the obstacle.
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import scene_env as se  # noqa: E402

DEEP_M = 0.02
CONTACT_M = 0.02                     # Unity's contactRadiusMeters in the course scene


def hull_groups(env_path):
    import trimesh
    cc = trimesh.load(os.path.join(env_path, "collision.glb"), force="scene")
    groups, room = {}, None
    for node in cc.graph.nodes_geometry:
        T, g = cc.graph[node]
        m = cc.geometry[g].copy()
        m.apply_transform(T)
        if node.startswith("room"):
            room = m
            continue
        item = "_".join(node.split("_")[:2])
        n = m.face_normals
        d = np.einsum("ij,ij->i", n, m.vertices[m.faces[:, 0]])
        c = m.vertices.mean(0)
        if not (n @ c - d < 0).all():          # winding check: centroid must be inside
            n, d = -n, -d
        groups.setdefault(item, []).append((n, d, m.bounds))
    return groups, room


def item_names(env_path):
    with open(os.path.join(env_path, "layout.json")) as f:
        lay = json.load(f)
    out = {}
    for it in lay.get("items", []):
        out[it["id"]] = f"{it['id']} {it.get('role', '')} {it.get('class', it.get('category', ''))}".strip()
    return out


def take_points(take, al):
    """(N, 3) glTF marker centres and their frame indices."""
    P, I = [], []
    for i in range(len(take.frame)):
        pts = [take.unl_pos[i][take.unl_kept[i]]]
        for O in take.rb_obs.values():               # observed rigid-body members only
            pts.append(O[i][np.isfinite(O[i]).all(1)])
        p = np.concatenate(pts)
        if len(p):
            P.append(al.to_gltf(i, p))
            I.append(np.full(len(p), i))
    return np.concatenate(P), np.concatenate(I)


def check_take(take, al, groups, room, names):
    P, I = take_points(take, al)
    F = len(take.frame)
    out = {}
    for item, hulls in groups.items():
        sd = np.full(len(P), np.inf)
        for n, d, (lo, hi) in hulls:
            near = np.all((P > lo - 0.05) & (P < hi + 0.05), axis=1)
            if near.any():
                sd[near] = np.minimum(sd[near], (P[near] @ n.T - d).max(1))
        out[names.get(item, item)] = summarize(sd, I, F, take)
    if room is not None:
        tree, S, N = room
        dist, _ = tree.query(P, distance_upper_bound=0.3)
        out["room scan (no floor), unsigned"] = summarize(dist, I, F, take, signed=False)
    return out


def room_samples(room, spacing=0.008, seed=0):
    """KD-tree over points sampled on the room collision mesh (floor removed) with their face normals."""
    import trimesh
    from scipy.spatial import cKDTree
    keep = ~((room.face_normals[:, 1] > 0.9) & (room.triangles_center[:, 1] < 0.05))
    walls = trimesh.Trimesh(room.vertices, room.faces[keep], process=False)
    n = int(walls.area / spacing ** 2)
    S, fi = trimesh.sample.sample_surface(walls, n, seed=seed)
    return cKDTree(S), S, walls.face_normals[fi]


def summarize(sd, I, F, take, signed=True):
    r = {"closest_m": round(float(sd.min()), 4) if np.isfinite(sd).any() else None,
         "frames_in_contact_2cm": int(len(np.unique(I[sd < CONTACT_M])))}
    if np.isfinite(sd).any():
        r["closest_at_s"] = round(float(take.t[I[int(np.argmin(sd))]]), 2)
    if signed:
        deep = sd < -DEEP_M
        r["frames_any_marker_inside"] = int(len(np.unique(I[sd < 0])))
        r["frames_deeper_than_2cm"] = int(len(np.unique(I[deep])))
        if deep.any():
            r["deep_spans_s"] = spans(np.unique(I[deep]), take.t)
    return r


def spans(frames, t, gap=10):
    out, s = [], frames[0]
    for a, b in zip(frames[:-1], frames[1:]):
        if b - a > gap:
            out.append([round(float(t[s]), 2), round(float(t[a]), 2)])
            s = b
    out.append([round(float(t[s]), 2), round(float(t[frames[-1]]), 2)])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--env", default=se.DEFAULT_ENV)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out"))
    a = ap.parse_args(argv)
    groups, room = hull_groups(a.env)
    room = room_samples(room) if room is not None else None
    names = item_names(a.env)
    logs = mr.parse_editor_log(os.path.join(a.root, "Editor.log"))
    res = {}
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        if take.log_name not in logs:
            continue
        al = mr.estimate_alignment(take, logs[take.log_name])
        res[take.name] = check_take(take, al, groups, room, names)
        print(take.name)
        for k, v in res[take.name].items():
            if v["closest_m"] is not None and v["closest_m"] < 0.03:
                print(f"   {k:40s} {json.dumps(v)}")
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "contact_check.json"), "w") as f:
        json.dump(res, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
