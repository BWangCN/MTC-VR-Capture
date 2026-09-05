"""Reference implementation: segment a take on pelvis turning points.

NOT a turnkey tool. It is checked in because the *method* matters and was
arrived at the hard way; the code still carries the paths and the day tag of the
session it was written for, and it imports an FBX reader that is not bundled
here. Adapt it, do not expect to run it as-is.

The method
----------
People walk a corridor back and forth. The pelvis z trace is therefore an
oscillation, and every rising limb that crosses the obstacle is one forward
pass. Turning points make natural clip boundaries because velocity goes to zero
there, so no separate padding rule is needed.

Why not segment on standing intervals: that assumes the person fully stops
between passes. When they do not, several passes merge into one span whose net
displacement cancels, and the span gets discarded as a reposition. On one
session that recovered 27 of 46 real crossings.

Validate the output by counting, independently, how many times the pelvis
crosses the obstacle band in the +z direction. Segment count should equal
crossing count; anything less means the segmenter is dropping passes.

What you must replace
---------------------
* ``ROOTS`` / ``TAG``   - hard-coded to one capture day.
* ``import fbx_reader`` - reads the obstacle band out of the built FBX. Either
  supply your own reader, or derive the band from the env YAML
  (``obstacles[0].position`` plus its geometric params) and mind the
  generator-to-FBX frame change.
* Lateral sign          - ``P[:, :, 0] *= -1``; CSV lateral is negated relative
  to FBX x. See docs/TROUBLESHOOTING.md.
"""

import csv
import glob
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.expanduser("~/mtc_chain_wsl/v2gate/build/p3"))
import fbx_reader

sys.stdout.reconfigure(encoding="utf-8")
ROOTS = ["/mnt/e/MTC_data/capture/09-01-2026", "/mnt/e/MTC_data/capture/09-02-2026"]
TAG = "__take__09-02-2026__"
OUT_FPS = 20.0
PROM = 0.80  # 折返点最小起伏，滤掉走路时的前后微抖
EDGE_TRIM = 0.15  # 段两端各留的余量(s)，避免正好切在折返那一帧上


def load(p):
    lines = [ln for ln in open(p, encoding="utf-8-sig") if ln.strip()]
    rows = list(csv.DictReader([ln for ln in lines if not ln.startswith("#")]))
    P = np.empty((len(rows), 24, 3))
    for j in range(24):
        for a, ax in enumerate("xyz"):
            P[:, j, a] = [float(r[f"p{j}{ax}"]) for r in rows]
    t = np.array([float(r["time"]) for r in rows])
    t -= t[0]
    P[:, :, 0] *= -1  # CSV 横向与 FBX 反号
    return P, t


def turning_points(z, prom):
    """交替的极小/极大序列，起伏小于 prom 的被吸收。"""
    idx = [0]
    mode = None
    for i in range(1, len(z)):
        if mode is None:
            if z[i] - z[idx[-1]] > prom:
                mode = "up"
            elif z[idx[-1]] - z[i] > prom:
                mode = "down"
            elif z[i] < z[idx[-1]] or z[i] > z[idx[-1]]:
                idx[-1] = i if False else idx[-1]
            continue
        if mode == "up":
            if z[i] > z[idx[-1]]:
                pass
            if z[i] >= z[max(idx[-1], 0)] and z[i] >= z[i - 1]:
                pass
        break
    # 简单稳健实现：滑动极值 + 交替筛选
    idx = []
    cur = 0
    direction = 0
    for i in range(1, len(z)):
        if direction >= 0 and z[i] > z[cur]:
            cur = i
            direction = 1
        elif direction <= 0 and z[i] < z[cur]:
            cur = i
            direction = -1
        elif direction > 0 and z[cur] - z[i] > prom:
            idx.append((cur, "max"))
            cur = i
            direction = -1
        elif direction < 0 and z[i] - z[cur] > prom:
            idx.append((cur, "min"))
            cur = i
            direction = 1
    idx.append((cur, "max" if direction > 0 else "min"))
    # 起点从不触发反向判定，第一个上升段会整段丢失——补一个与首个拐点相反的起始拐点
    if idx and idx[0][0] > 0:
        idx.insert(0, (0, "min" if idx[0][1] == "max" else "max"))
    return idx


ENVD = {}
for R in ROOTS:
    if not os.path.isdir(R):
        continue
    for e in sorted(os.listdir(R)):
        d = os.path.join(R, e)
        if os.path.isdir(d) and glob.glob(f"{d}/raw/*{TAG}*.csv"):
            ENVD[e] = d


def geom(env):
    sc = fbx_reader.load_scene(f"{ENVD[env]}/mesh/{env}_simplified.fbx")
    o = {"floor": None, "walls": [], "raw": []}
    for m in sc["meshes"]:
        n = m["model_name"]
        if n in ("goal_ring", "goal_ring_beam") or (n.startswith("wall") and n != "floor"):
            continue
        V = np.asarray(m["verts"], float)
        lo, hi = V.min(0), V.max(0)
        b = [round(float(v), 4) for v in lo] + [round(float(v), 4) for v in hi]
        if n == "floor":
            o["floor"] = b
        elif n.startswith("hwall"):
            o["walls"].append(b)
        else:
            o["raw"].append(b)
    return o


out = {
    "fps": OUT_FPS,
    "envs": {},
    "passes": [],
    "day": "09-02-2026",
    "seg": {"method": "turning_point", "prominence_m": PROM, "edge_trim_s": EDGE_TRIM},
}
stat = {}
for env in sorted(ENVD):
    d = ENVD[env]
    g = geom(env)
    y = yaml.safe_load(open(f"{d}/{env}.yaml", encoding="utf-8"))["obstacles"][0][
        "geometric_params"
    ]
    if "walls" in y:
        wb = [
            [
                round(w["position"][1] - w["size_xyz"][1] / 2 - 1.125, 4),
                round(w["position"][1] + w["size_xyz"][1] / 2 - 1.125, 4),
            ]
            for w in y["walls"]
        ]
        free = [(-1.125, 1.125)]
        for a, b_ in sorted(wb):
            nf = []
            for c0, c1 in free:
                if b_ <= c0 or a >= c1:
                    nf.append((c0, c1))
                    continue
                if c0 < a:
                    nf.append((c0, a))
                if b_ < c1:
                    nf.append((b_, c1))
            free = nf
        g["free"] = [[round(a, 4), round(b_, 4)] for a, b_ in free]
    else:
        g["free"] = []
    out["envs"][env] = g
    fam = "step" if "steprow" in env else "wall"
    parts = g["walls"] if g["walls"] else g["raw"]
    z0 = min(b[2] for b in parts)
    z1 = max(b[5] for b in parts)
    c = 0
    for p in sorted(glob.glob(f"{d}/raw/*{TAG}*.csv")):
        if p.endswith(".collisions.csv"):
            continue
        P, t = load(p)
        if len(t) < 50:
            continue
        z = P[:, 0, 2]
        tps = turning_points(z, PROM)
        for (i0, k0_), (i1, k1_) in zip(tps[:-1], tps[1:], strict=False):
            if k0_ != "min" or k1_ != "max":
                continue
            seg = z[i0 : i1 + 1]
            if not (seg.min() < z0 and seg.max() > z1):
                continue
            a = int(np.searchsorted(t, max(t[i0] - EDGE_TRIM, t[0])))
            b_ = int(np.searchsorted(t, min(t[i1] + EDGE_TRIM, t[-1])))
            R = P[a : b_ + 1]
            if len(R) < 20:
                continue
            c += 1
            rec = dict(
                clip=os.path.basename(p)[:-4],
                env=env,
                fam=fam,
                idx=c - 1,
                sid=f"{env.replace('corridor_atomic_','')}#{c-1:02d}",
                t0=round(float(t[a]), 3),
                t1=round(float(t[b_]), 3),
                f0=int(a),
                f1=int(b_),
                src_fps=round(float(len(t) / max(t[-1], 1e-9)), 2),
                dur=round(float(t[b_] - t[a]), 2),
                dZ=round(float(z[i1] - z[i0]), 3),
                z_span=[round(float(seg.min()), 2), round(float(seg.max()), 2)],
                obs=dict(z0=round(z0, 3), z1=round(z1, 3)),
            )
            if fam == "step":
                pen = over = 0
                worst = 0.0
                peak = -9.0
                for j in (10, 11):
                    F = R[:, j, :]
                    for k in range(len(F)):
                        fx, fy, fz = F[k]
                        for bb in g["raw"]:
                            if bb[0] <= fx <= bb[3] and bb[2] <= fz <= bb[5]:
                                over += 1
                                cc = fy - bb[4]
                                peak = max(peak, cc)
                                if cc < 0:
                                    pen += 1
                                    worst = min(worst, cc)
                                break
                rec.update(
                    over_frames=over,
                    pen_frames=pen,
                    pen_depth=round(float(worst), 3),
                    clear_best=round(float(peak), 3) if peak > -9 else None,
                    foot_peak=round(float(R[:, [10, 11], 1].max()), 3),
                )
            else:
                m = (R[:, 0, 2] >= z0) & (R[:, 0, 2] <= z1)
                x = R[m, 0, 0] if m.any() else R[:, 0, 0]
                ins = sum(1 for v in x if any(aa <= v <= bb for aa, bb in g["free"]))
                rec.update(
                    in_gap_pct=round(100 * ins / max(len(x), 1), 1),
                    lat=[round(float(x.min()), 3), round(float(x.max()), 3)],
                )
            tn = np.arange(t[a], t[b_], 1.0 / OUT_FPS)
            Q = np.empty((len(tn), 24, 3))
            for j in range(24):
                for cc in range(3):
                    Q[:, j, cc] = np.interp(tn, t, P[:, j, cc])
            rec["n"] = len(Q)
            rec["P"] = np.round(Q * 1000).astype(int).reshape(len(Q), -1).tolist()
            out["passes"].append(rec)
    stat[env] = c
json.dump(
    out, open("/mnt/e/MTC_data/capture/_index/segments_09-02-2026.json", "w"), separators=(",", ":")
)
print(f"{'env':<14}{'切出段数':>9}{'真实穿越':>9}")
REAL = {
    "corridor_atomic_steprow_00": 5,
    "corridor_atomic_steprow_01": 5,
    "corridor_atomic_steprow_02": 5,
    "corridor_atomic_wleft_00": 5,
    "corridor_atomic_wleft_01": 5,
    "corridor_atomic_wleft_02": 5,
    "corridor_atomic_wslit_00": 7,
    "corridor_atomic_wslit_01": 7,
    "corridor_atomic_wslit_02": 2,
}
for e, n in stat.items():
    print(f"  {e.replace('corridor_atomic_',''):<12}{n:>9}{REAL.get(e,0):>9}")
print(
    f"\n合计 {sum(stat.values())} 段 / 真实 46 次  -> {os.path.getsize('/mnt/e/MTC_data/capture/_index/segments_09-02-2026.json')/1e6:.2f} MB"
)
