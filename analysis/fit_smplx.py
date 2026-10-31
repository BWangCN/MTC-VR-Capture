"""Fit SMPL-X to a native Motive take whose body markers are unlabeled (no marker names needed).

    PYTHONNOUSERSITE=1 python analysis/fit_smplx.py analysis/data/raw/2026-10-30_recordings/recordings \
        [--takes jumping take_...] [--out analysis/data/out/smplx] [--gender neutral]

Per take (native metric positions; the VR player scale is NOT applied):
1. Body points per frame: unlabeled markers within 0.9 m (horizontal) of the headset, plus the body markers Motive
   gave to the stray rigid body "v4w2" (Take.rb_obs). The 4 Pico headset markers are used as points fixed to the
   head joint.
2. Start pose: in the quietest 0.5 s of the take, SMPL-X (VPoser pose, 16 betas) is fitted to the points with a
   label-free loss (each point ~9 mm off the nearest skin vertex); facing from the headset's forward axis.
3. Anchors: each point of a reference frame is attached to its nearest vertex with a rest-frame offset (as in
   MoSh); the headset markers get joint-local offsets on the head.
4. Tracking: frame by frame, observed points are matched to the predicted anchors (Hungarian assignment, gated;
   staying on the same Motive trajectory is cheaper) and the pose is updated from the previous frame.
5. Batch: all frames together with temporal smoothness; re-match; re-estimate betas and anchor offsets (MoSh
   stage I) on all frames; final fit.
Output per take: <out>/<take>.npz (AMASS-style: poses (F, 165), trans, betas, gender, mocap_framerate, plus
anchors and per-frame residuals) and <out>/<take>.json (report). Frame: native Motive turned to Y up,
p_model = (x, z, -y).
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch
from scipy.optimize import linear_sum_assignment

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import smplx_model as sm  # noqa: E402

FPS = 100
SKIN_OFFSET_M = 0.009                # marker centre to skin (14 mm marker on a ~2 mm suit)
BODY_RADIUS_M = 0.9                  # body points: horizontal distance to the headset
GATE_M = 0.08                        # matching gate
SIGMA_CM = 3.0                       # robust scale of the data term
Z_UP_TO_Y_UP = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float)   # (x, y, z) -> (x, z, -y)


def to_model(P):
    return np.asarray(P, float) @ Z_UP_TO_Y_UP.T


def gmof_cm(r_m):
    r = r_m * 100.0
    return (r ** 2) * (SIGMA_CM ** 2) / (r ** 2 + SIGMA_CM ** 2)


def quat_xyzw_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# ------------------------------------------------------------------------------------------------ data
class TakeData:
    """Per-frame body points (model frame), their Motive ids (-1 = none) and the headset markers."""

    def __init__(self, take):
        self.take = take
        F = len(take.frame)
        head = take.rb[mr.PICO_RB_ID]["pos"]
        others = [i for i in take.rb if i != mr.PICO_RB_ID]
        self.pts, self.ids = [], []
        for i in range(F):
            P, ids = take.unl_pos[i], take.unl_id[i]
            for o in others:
                O = take.rb_obs[o][i]
                ok = np.isfinite(O).all(1)
                P = np.concatenate([P, O[ok]])
                ids = np.concatenate([ids, np.full(ok.sum(), -1, np.int64)])
            h = head[i] if np.isfinite(head[i]).all() else None
            if h is not None:
                d = np.linalg.norm(P[:, :2] - h[:2], axis=1)
                keep = (d < BODY_RADIUS_M) & (P[:, 2] > -0.1) & (P[:, 2] < h[2] + 0.3)
                P, ids = P[keep], ids[keep]
            self.pts.append(to_model(P))
            self.ids.append(ids)
        self.pico = to_model(take.rb_obs[mr.PICO_RB_ID].reshape(-1, 3)).reshape(F, -1, 3)    # (F, 4, 3), NaN = missing
        self.head_pos = to_model(head)
        self.head_fwd = np.full((F, 3), np.nan)
        q = take.rb[mr.PICO_RB_ID]["quat_xyzw"]
        for i in range(F):
            if np.isfinite(q[i]).all():
                self.head_fwd[i] = to_model(quat_xyzw_to_matrix(q[i]) @ np.array([1.0, 0, 0]))   # Motive local +X = front
        self.F = F

    def quiet_window(self, length=50):
        """Start of the length-frame window where the subject stands upright and moves least: headset speed, plus
        a penalty for the headset being below its standing height (a crouch cannot be reached from the standing
        start pose), plus a small one for missing points."""
        h = self.head_pos
        sp = np.r_[0, np.linalg.norm(np.diff(h, axis=0), axis=1)]
        sp = np.nan_to_num(sp, nan=1.0)
        n = np.array([len(p) for p in self.pts], float)
        stand = np.nanpercentile(h[:, 1], 95)
        low = np.nan_to_num(np.clip(stand - h[:, 1] - 0.05, 0, None), nan=1.0)
        box = np.ones(length)
        cost = (np.convolve(sp, box, "valid") + 5.0 * np.convolve(low, box / length, "valid")
                + 0.01 * np.abs(np.convolve(n, box / length, "valid") - np.median(n)))
        return int(np.argmin(cost))


# ------------------------------------------------------------------------------------------------ fitter
class Fitter:
    def __init__(self, data, gender="neutral", device="cuda", log=print):
        self.d = data
        self.dev = device
        self.gender = gender
        self.bm = sm.SMPLX(gender, device)
        self.vp = sm.load_vposer(device)
        self.log = log
        self.betas = torch.zeros(1, sm.N_BETAS, device=device)
        F = data.F
        self.z = torch.zeros(F, 32, device=device)
        self.g6 = torch.zeros(F, 6, device=device)
        self.tr = torch.zeros(F, 3, device=device)
        self.done = np.zeros(F, bool)

    def t(self, a):
        return torch.as_tensor(np.asarray(a), dtype=torch.float32, device=self.dev)

    def yaw_matrix(self, fwd):
        f = np.array([fwd[0], 0, fwd[2]])
        f /= np.linalg.norm(f) + 1e-9
        a = np.arctan2(f[0], f[2])                                        # SMPL-X faces +Z
        c, s = np.cos(a), np.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    # ------------------------------------------------------------------ 2. start pose (label-free)
    def init_window(self, w0, length=50, step=5, iters=500):
        d, bm = self.d, self.bm
        frames = list(range(w0, min(d.F, w0 + length), step))
        K = len(frames)
        fwd = np.nanmedian(d.head_fwd[frames], 0)
        R0 = self.yaw_matrix(fwd)
        vs, _ = bm.shaped(self.betas)
        rest = vs[0].detach().cpu().numpy()
        P = [d.pts[i] for i in frames]
        cen = np.stack([p.mean(0) for p in P])
        low = np.array([p[:, 1].min() for p in P])
        tr0 = cen - rest.mean(0) @ R0.T
        tr0[:, 1] = low - (rest[:, 1].min()) - 0.0
        g6 = self.t(np.repeat(sm.matrix_to_rot6d(torch.tensor(R0[None])).numpy(), K, 0)).requires_grad_(True)
        z = torch.zeros(K, 32, device=self.dev, requires_grad=True)
        tr = self.t(tr0).requires_grad_(True)
        betas = torch.zeros(1, sm.N_BETAS, device=self.dev, requires_grad=True)
        maxn = max(len(p) for p in P)
        obs = torch.zeros(K, maxn, 3, device=self.dev)
        msk = torch.zeros(K, maxn, device=self.dev)
        for k, p in enumerate(P):
            obs[k, :len(p)] = self.t(p)
            msk[k, :len(p)] = 1
        hp = self.t(d.head_pos[frames])
        Rfix = self.t(R0)
        opt = torch.optim.Adam([g6, z, tr, betas], lr=0.03)
        for it in range(iters):
            opt.zero_grad()
            Rg = sm.rot6d_to_matrix(g6)
            v, J = bm.forward_full(betas, Rg, sm.decode_body(self.vp, z), tr)
            dist = torch.cdist(obs, v).min(-1).values                     # (K, maxn) point -> nearest vertex
            data = (gmof_cm(dist - SKIN_OFFSET_M) * msk).sum() / msk.sum()
            head = ((J[:, sm.HEAD] - hp).norm(dim=-1) - 0.15).clamp(min=0) * 100
            yaw = ((Rg[:, :, 2] - Rfix[:, 2][None]) ** 2).sum(-1).mean()  # stay facing the headset direction
            loss = data + 0.5 * (head ** 2).mean() + 1.0 * yaw + 0.01 * (z ** 2).mean() + 0.01 * (betas ** 2).mean()
            loss.backward()
            opt.step()
            if it == iters // 2:
                for pg in opt.param_groups:
                    pg["lr"] *= 0.3
        with torch.no_grad():
            Rg = sm.rot6d_to_matrix(g6)
            v, J = bm.forward_full(betas, Rg, sm.decode_body(self.vp, z), tr)
            dist = torch.cdist(obs, v).min(-1).values
            rms = float(((dist - SKIN_OFFSET_M) ** 2 * msk).sum() / msk.sum()) ** 0.5
        self.betas = betas.detach()
        for k, i in enumerate(frames):
            self.z[i], self.g6[i], self.tr[i] = z[k].detach(), g6[k].detach(), tr[k].detach()
        self.log(f"   start pose: frames {frames[0]}..{frames[-1]}, {maxn} points max, skin-distance RMS {rms * 1000:.1f} mm")
        return frames

    # ------------------------------------------------------------------ 3. anchors
    def make_anchors(self, ref):
        d, bm = self.d, self.bm
        p = d.pts[ref]
        with torch.no_grad():
            Rg = sm.rot6d_to_matrix(self.g6[ref:ref + 1])
            Rb = sm.decode_body(self.vp, self.z[ref:ref + 1])
            v, J = bm.forward_full(self.betas, Rg, Rb, self.tr[ref:ref + 1])
            pt = self.t(p)
            vid = torch.cdist(pt[None], v).min(-1).indices[0]           # nearest vertex per point
            Tv = bm.vertex_transforms(self.betas, Rg, Rb, vid)[0]       # (M, 4, 4)
            vs, _ = bm.shaped(self.betas)
            R = bm.full_rotmats(Rg, Rb)
            pd = bm.posedirs.view(486, bm.n_verts, 3)[:, vid]
            rest = vs[0, vid] + torch.einsum("p,pmc->mc", bm.pose_feature(R)[0], pd)
            local = torch.linalg.solve(Tv[:, :3, :3], (pt - self.tr[ref] - Tv[:, :3, 3])[..., None])[..., 0]
            off = local - rest
            # headset markers: joint-local offsets on the head
            A = bm.rigid_transforms(R, bm.shaped(self.betas)[1])[0, sm.HEAD]
            Jh = J[0, sm.HEAD]
            pico = self.t(np.nan_to_num(d.pico[ref], nan=0.0))
            rig = (A[:3, :3].T @ (pico - Jh).T).T
        self.vids, self.offsets = vid, off
        self.ref, self.ref_ids = ref, d.ids[ref].copy()
        self.pico_off = rig
        self.pico_ok = torch.tensor(np.isfinite(d.pico[ref]).all(1), device=self.dev)
        self.M = len(vid)
        self.log(f"   anchors: {self.M} body markers on {len(torch.unique(vid))} distinct vertices, "
                 f"offsets median {off.norm(dim=-1).median() * 1000:.1f} mm; headset markers {int(self.pico_ok.sum())}")
        return off.norm(dim=-1).cpu().numpy()

    def predict(self, idx, betas=None, offsets=None):
        Rg = sm.rot6d_to_matrix(self.g6[idx])
        Rb = sm.decode_body(self.vp, self.z[idx])
        return self.bm.anchors(self.betas if betas is None else betas, Rg, Rb, self.tr[idx], self.vids,
                               self.offsets if offsets is None else offsets, sm.HEAD, self.pico_off)

    # ------------------------------------------------------------------ 4. tracking
    def match(self, pred, i, prev_ids=None):
        """Observed body points of frame i -> anchors (M,) index into d.pts[i] or -1."""
        p = self.d.pts[i]
        out = -np.ones(self.M, np.int64)
        if not len(p):
            return out
        C = np.linalg.norm(pred[:self.M, None] - p[None], axis=2)
        if prev_ids is not None:
            same = (prev_ids[:, None] >= 0) & (prev_ids[:, None] == self.d.ids[i][None]) & (self.d.ids[i][None] >= 0)
            C = np.where(same, C * 0.25, C)
        C2 = np.where(C < GATE_M, C, 1e3)
        r, c = linear_sum_assignment(C2)
        ok = C2[r, c] < 1e3
        out[r[ok]] = c[ok]
        return out

    def frame_loss(self, pred, obs, msk, pico, pico_m):
        data = (gmof_cm((pred[:, :self.M] - obs).norm(dim=-1)) * msk).sum() / msk.sum().clamp(min=1)
        hp = (gmof_cm((pred[:, self.M:] - pico).norm(dim=-1)) * pico_m).sum() / pico_m.sum().clamp(min=1)
        return data + hp

    def track(self, start, stop, step, iters=15):
        """Sequential pass from frame start (already posed) to stop (exclusive), step +1 or -1."""
        d = self.d
        match_ids = getattr(self, "match_ids", None)
        prev = start
        vel = torch.zeros(3, device=self.dev)
        for i in range(start + step, stop, step):
            with torch.no_grad():
                self.z[i], self.g6[i], self.tr[i] = self.z[prev], self.g6[prev], self.tr[prev] + vel
                pred = self.predict(slice(i, i + 1))[0].cpu().numpy()
            prev_ids = None
            if match_ids is not None:
                pm = self.matches[prev]
                prev_ids = np.where(pm >= 0, np.array([d.ids[prev][k] if k >= 0 else -1 for k in pm]), -1)
            m = self.match(pred, i, prev_ids)
            self.matches[i] = m
            obs = torch.zeros(1, self.M, 3, device=self.dev)
            msk = torch.zeros(1, self.M, device=self.dev)
            ok = m >= 0
            obs[0, ok] = self.t(d.pts[i][m[ok]])
            msk[0, ok] = 1
            pico = self.t(np.nan_to_num(d.pico[i:i + 1], nan=0.0))
            pico_m = self.t(np.isfinite(d.pico[i:i + 1]).all(-1).astype(np.float32)) * self.pico_ok[None]
            z = self.z[i:i + 1].clone().requires_grad_(True)
            g6 = self.g6[i:i + 1].clone().requires_grad_(True)
            tr = self.tr[i:i + 1].clone().requires_grad_(True)
            z0, g0, t0 = z.detach().clone(), g6.detach().clone(), tr.detach().clone()
            opt = torch.optim.Adam([z, g6, tr], lr=0.01)
            for _ in range(iters):
                opt.zero_grad()
                pred = self.bm.anchors(self.betas, sm.rot6d_to_matrix(g6), sm.decode_body(self.vp, z), tr,
                                       self.vids, self.offsets, sm.HEAD, self.pico_off)
                loss = (self.frame_loss(pred, obs, msk, pico, pico_m) + 0.5 * ((z - z0) ** 2).sum()
                        + 10.0 * ((g6 - g0) ** 2).sum() + 0.001 * (z ** 2).sum())
                loss.backward()
                opt.step()
            with torch.no_grad():
                vel = (tr[0] - self.tr[prev]).detach() if abs(i - prev) == 1 else vel
                self.z[i], self.g6[i], self.tr[i] = z[0].detach(), g6[0].detach(), tr[0].detach()
            self.done[i] = True
            prev = i

    # ------------------------------------------------------------------ 5. batch
    def obs_tensors(self):
        F, d = self.d.F, self.d
        obs = torch.zeros(F, self.M, 3, device=self.dev)
        msk = torch.zeros(F, self.M, device=self.dev)
        for i in range(F):
            m = self.matches[i]
            ok = m >= 0
            if ok.any():
                obs[i, ok] = self.t(d.pts[i][m[ok]])
                msk[i, ok] = 1
        pico = self.t(np.nan_to_num(d.pico, nan=0.0))
        pico_m = self.t(np.isfinite(d.pico).all(-1).astype(np.float32)) * self.pico_ok[None]
        return obs, msk, pico, pico_m

    def batch(self, iters=600, fit_shape=False, lr=0.01):
        obs, msk, pico, pico_m = self.obs_tensors()
        z = self.z.clone().requires_grad_(True)
        g6 = self.g6.clone().requires_grad_(True)
        tr = self.tr.clone().requires_grad_(True)
        params = [z, g6, tr]
        betas, offs = self.betas, self.offsets
        if fit_shape:
            betas = self.betas.clone().requires_grad_(True)
            offs = self.offsets.clone().requires_grad_(True)
            off0 = self.offsets.clone()
            params += [betas, offs]
        opt = torch.optim.Adam(params, lr=lr)
        for it in range(iters):
            opt.zero_grad()
            Rg = sm.rot6d_to_matrix(g6)
            pred = self.bm.anchors(betas, Rg, sm.decode_body(self.vp, z), tr, self.vids, offs, sm.HEAD, self.pico_off)
            data = self.frame_loss(pred, obs, msk, pico, pico_m)
            acc_t = ((tr[2:] - 2 * tr[1:-1] + tr[:-2]) * 100).pow(2).sum(-1).mean()
            acc_z = (z[2:] - 2 * z[1:-1] + z[:-2]).pow(2).sum(-1).mean()
            rel = Rg[:-1].transpose(-1, -2) @ Rg[1:]
            acc_r = (rel[1:] - rel[:-1]).pow(2).sum((-1, -2)).mean()
            loss = data + 0.05 * acc_t + 5.0 * acc_z + 50.0 * acc_r + 0.0005 * (z ** 2).sum(-1).mean()
            if fit_shape:
                loss = loss + 0.01 * (betas ** 2).sum() + 1.0 * ((offs - off0) * 100).pow(2).sum(-1).mean()
            loss.backward()
            opt.step()
            if it == int(iters * 0.7):
                for pg in opt.param_groups:
                    pg["lr"] *= 0.3
        self.z, self.g6, self.tr = z.detach(), g6.detach(), tr.detach()
        if fit_shape:
            self.betas, self.offsets = betas.detach(), offs.detach()

    def reanchor(self, min_frames=60, radius=0.15, improve=0.5):
        """Re-pick each anchor's vertex from the marker's own Motive trajectory (or its matches, for points that came
        from v4w2): the vertex in whose skinned frame the marker stays most constant over time. A still reference
        frame cannot tell, e.g., a hand hanging at the hip from a hip marker; motion can."""
        d, bm = self.d, self.bm
        where = {}
        for i in range(d.F):
            for j, mid in enumerate(d.ids[i]):
                if mid >= 0:
                    where.setdefault(int(mid), []).append((i, j))
        changes = []
        with torch.no_grad():
            Rg = sm.rot6d_to_matrix(self.g6)
            Rb = sm.decode_body(self.vp, self.z)
            vs, J = bm.shaped(self.betas)
            vref, _ = bm.forward_full(self.betas, Rg[self.ref:self.ref + 1], Rb[self.ref:self.ref + 1], self.tr[self.ref:self.ref + 1])
            pd_all = bm.posedirs.view(486, bm.n_verts, 3)
            for m in range(self.M):
                mid = int(self.ref_ids[m])
                if mid >= 0 and len(where.get(mid, [])) >= min_frames:
                    fr = np.array([a for a, _ in where[mid]])
                    pts = np.stack([d.pts[a][b] for a, b in where[mid]])
                    src = "trajectory"
                else:
                    fr = np.array([i for i in range(d.F) if self.matches[i][m] >= 0])
                    if len(fr) < min_frames:
                        continue
                    pts = np.stack([d.pts[i][self.matches[i][m]] for i in fr])
                    src = "matches"
                if len(fr) > 600:                                        # enough frames, spread over the take
                    keep = np.linspace(0, len(fr) - 1, 600).astype(int)
                    fr, pts = fr[keep], pts[keep]
                p_ref = vref[0, self.vids[m]]
                cand = torch.nonzero((vref[0] - p_ref).norm(dim=-1) < radius)[:, 0]
                R = bm.full_rotmats(Rg[fr], Rb[fr])
                A = bm.rigid_transforms(R, J.expand(len(fr), -1, -1))
                Tv = torch.einsum("cj,njab->ncab", bm.weights[cand], A)
                rest = vs[0, cand][None] + torch.einsum("np,pcd->ncd", bm.pose_feature(R), pd_all[:, cand])
                q = self.t(pts) - self.tr[fr]
                local = torch.linalg.solve(Tv[..., :3, :3], (q[:, None] - Tv[..., :3, 3])[..., None])[..., 0]
                off = local - rest                                       # (n, C, 3) rest-frame offsets
                var = off.var(0).sum(-1)
                cur = torch.nonzero(cand == self.vids[m])
                vcur = var[cur[0, 0]] if len(cur) else torch.tensor(float("inf"), device=self.dev)
                # only vertices the marker can sit on (mean offset under 3 cm); a far vertex on the same rigid
                # segment can be steadier only because the joint in between is not yet constrained
                var = torch.where(off.mean(0).norm(dim=-1) < 0.03, var, torch.full_like(var, float("inf")))
                b = int(torch.argmin(var))
                if torch.isfinite(var[b]) and var[b] < improve * vcur:
                    changes.append({"anchor": m, "from_vertex": int(self.vids[m]), "to_vertex": int(cand[b]), "source": src,
                                    "frames": int(len(fr)), "spread_mm_before": round(float(vcur.sqrt()) * 1000, 1),
                                    "spread_mm_after": round(float(var[b].sqrt()) * 1000, 1)})
                    self.vids[m] = cand[b]
                    self.offsets[m] = off[:, b].mean(0)
        return changes

    def rematch(self):
        with torch.no_grad():
            pred = torch.cat([self.predict(slice(a, min(a + 512, self.d.F))) for a in range(0, self.d.F, 512)]).cpu().numpy()
        changed = 0
        for i in range(self.d.F):
            m = self.match(pred[i], i)
            changed += int((m != self.matches[i]).sum())
            self.matches[i] = m
        return changed

    def residuals(self):
        obs, msk, pico, pico_m = self.obs_tensors()
        with torch.no_grad():
            pred = torch.cat([self.predict(slice(a, min(a + 512, self.d.F))) for a in range(0, self.d.F, 512)])
            r = (pred[:, :self.M] - obs).norm(dim=-1)
            rp = (pred[:, self.M:] - pico).norm(dim=-1)
        r = torch.where(msk > 0, r, torch.full_like(r, float("nan"))).cpu().numpy()
        rp = torch.where(pico_m > 0, rp, torch.full_like(rp, float("nan"))).cpu().numpy()
        return r, rp

    def unmatched(self):
        n_obs = np.array([len(p) for p in self.d.pts])
        n_m = np.array([(m >= 0).sum() for m in self.matches])
        return n_obs - n_m


def fit_take(take, out_dir, gender="neutral", device="cuda", log=print):
    t0 = time.time()
    d = TakeData(take)
    f = Fitter(d, gender, device, log)
    w0 = d.quiet_window()
    frames = f.init_window(w0)
    ref = frames[len(frames) // 2]
    off_mm = f.make_anchors(ref)
    for i in frames:
        f.done[i] = True
    f.matches = [None] * d.F
    f.matches[ref] = np.arange(f.M)                                     # anchors = points of the ref frame
    f.match_ids = True
    # pose the window frames from ref, then track out of the window in both directions
    f.track(ref, d.F, +1)
    f.track(ref, -1, -1)
    log(f"   tracked {d.F} frames in {time.time() - t0:.0f} s")
    f.batch(iters=600)
    moved = f.reanchor()
    for c in moved:
        log(f"   re-anchored marker {c['anchor']}: vertex {c['from_vertex']} -> {c['to_vertex']} "
            f"(spread {c['spread_mm_before']} -> {c['spread_mm_after']} mm over {c['frames']} frames, {c['source']})")
    ch = f.rematch()
    f.batch(iters=600)
    ch = f.rematch()
    log(f"   batch 2 done; re-match changed {ch} assignments")
    f.batch(iters=400, fit_shape=True)
    ch = f.rematch()
    f.batch(iters=400)
    r, rp = f.residuals()
    un = f.unmatched()
    log(f"   final: re-match changed {ch}; marker residual median {np.nanmedian(r) * 1000:.1f} mm, "
        f"p95 {np.nanpercentile(r, 95) * 1000:.1f} mm; headset {np.nanmedian(rp) * 1000:.1f} mm; "
        f"unmatched points/frame median {np.median(un):.0f}")
    save(f, take, out_dir, r, rp, un, w0, off_mm, time.time() - t0, moved)
    return f


def save(f, take, out_dir, r, rp, un, w0, off_mm, secs, moved=()):
    bm, d = f.bm, f.d
    F = d.F
    with torch.no_grad():
        Rg = sm.rot6d_to_matrix(f.g6)
        Rb = sm.decode_body(f.vp, f.z)
        aa_g = sm.matrix_to_axis_angle(Rg).cpu().numpy()
        aa_b = sm.matrix_to_axis_angle(Rb).reshape(F, -1).cpu().numpy()
        # rest-frame pelvis convention of SMPL-X: AMASS 'trans' is the root translation added to the model
        heights = []
        for a in range(0, F, 256):
            v, _ = bm.forward_full(f.betas, Rg[a:a + 256], Rb[a:a + 256], f.tr[a:a + 256])
            heights.append((v[..., 1].max(-1).values - v[..., 1].min(-1).values).cpu().numpy())
        vs, _ = bm.shaped(f.betas)
        stature = float(vs[0, :, 1].max() - vs[0, :, 1].min())
    poses = np.zeros((F, 165), np.float32)
    poses[:, :3], poses[:, 3:66] = aa_g, aa_b
    os.makedirs(out_dir, exist_ok=True)
    np.savez_compressed(
        os.path.join(out_dir, f"{take.name}.npz"), poses=poses, trans=f.tr.cpu().numpy().astype(np.float32),
        betas=f.betas[0].cpu().numpy().astype(np.float32), num_betas=sm.N_BETAS, gender=f.gender,
        surface_model_type="smplx", mocap_framerate=float(FPS), frame=take.frame,
        anchor_vids=f.vids.cpu().numpy(), anchor_offsets=f.offsets.cpu().numpy(), pico_offsets=f.pico_off.cpu().numpy(),
        residual_m=r.astype(np.float32), pico_residual_m=rp.astype(np.float32),
        matches=np.stack(f.matches).astype(np.int16), unmatched_points=un.astype(np.int16),
        frame_note=f"native Motive turned Y-up: p_model = (x, z, -y); metres; SMPL-X {f.gender}, flat hands")
    seg = vertex_segments(bm, f.vids.cpu().numpy())
    rep = {"take": take.name, "frames": F, "seconds": round(secs, 1), "start_window": [int(w0), int(w0 + 50)],
           "markers_attached": int(f.M), "anchor_body_parts": seg, "re_anchored": list(moved),
           "anchor_offset_mm_at_init": {"median": round(float(np.median(off_mm) * 1000), 1),
                                        "max": round(float(off_mm.max() * 1000), 1)},
           "anchor_offset_mm_final": {"median": round(float(f.offsets.norm(dim=-1).median() * 1000), 1),
                                      "max": round(float(f.offsets.norm(dim=-1).max() * 1000), 1)},
           "stature_m": round(stature, 3), "betas": [round(float(b), 3) for b in f.betas[0]],
           "residual_mm": {"median": round(float(np.nanmedian(r) * 1000), 2), "p95": round(float(np.nanpercentile(r, 95) * 1000), 2),
                           "max": round(float(np.nanmax(r) * 1000), 1)},
           "residual_mm_per_marker": [round(float(x) * 1000, 2) for x in np.nanmedian(r, 0)],
           "headset_residual_mm": {"median": round(float(np.nanmedian(rp) * 1000), 2), "p95": round(float(np.nanpercentile(rp, 95) * 1000), 2)},
           "matched_fraction": round(float(np.isfinite(r).mean()), 4),
           "unmatched_points_per_frame": {"median": float(np.median(un)), "p95": float(np.percentile(un, 95))}}
    with open(os.path.join(out_dir, f"{take.name}.json"), "w") as fh:
        json.dump(rep, fh, indent=1)


def vertex_segments(bm, vids):
    """Counts of anchors by the joint that skins them most."""
    w = bm.weights[torch.as_tensor(vids, device=bm.device)].argmax(-1).cpu().numpy()
    names = sm.JOINT_NAMES + [f"j{k}" for k in range(len(sm.JOINT_NAMES), 55)]
    out = {}
    for j in w:
        n = names[j] if j < 22 else ("left_hand" if 25 <= j < 40 else "right_hand" if j >= 40 else names[j])
        out[n] = out.get(n, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--takes", nargs="*")
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "smplx"))
    ap.add_argument("--gender", default="neutral")
    ap.add_argument("--device", default="cuda")
    a = ap.parse_args(argv)
    torch.manual_seed(0)
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        if a.takes and take.name not in a.takes:
            continue
        print(f"== {take.name}", flush=True)
        fit_take(take, a.out, a.gender, a.device, log=lambda s: print(s, flush=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
