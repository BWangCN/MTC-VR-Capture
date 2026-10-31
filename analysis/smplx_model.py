"""SMPL-X forward pass in PyTorch, from the model .npz, with skinned "anchor" points for markers.

Coordinates: SMPL-X canonical frame, Y up, metres. The model files are licensed (SMPL-X licence) and are read from
outside this repository (default: /data/Humanoid/Marker2SMPLX/body_models, override with MTCVR_BODY_MODELS);
they are never copied here.

Pose parametrisation used by the fitter:
- global rotation: 6D (Zhou et al. 2019) -> rotation matrix;
- body pose (21 joints): VPoser v1 latent (32-d), decoded to rotation matrices;
- jaw, eyes and hands: identity (flat hands), since there are no face or finger markers to drive them.
"""
import os
import sys

import numpy as np
import torch

BODY_MODELS = os.environ.get("MTCVR_BODY_MODELS", "/data/Humanoid/Marker2SMPLX/body_models")
# MANO mean ("relaxed") hand pose, read from an official SMPL-X v1.1 .npz (hands_meanl / hands_meanr); the files in
# BODY_MODELS do not carry it. Used for display only: there are no finger markers, so the fit keeps hand pose 0.
HANDS_MEAN_FILE = os.environ.get("MTCVR_HANDS_MEAN", "/data/Humanoid/UMR/smpl/SMPLX_NEUTRAL.npz")
HAND_JOINTS = (25, 55)                           # SMPL-X joints 25..39 left hand, 40..54 right hand
N_BETAS = 16
N_JOINTS = 55
N_BODY = 21                                      # body_pose joints 1..21
HEAD = 15                                        # SMPL-X joint index of the head
JOINT_NAMES = ["pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee", "spine2", "left_ankle",
               "right_ankle", "spine3", "left_foot", "right_foot", "neck", "left_collar", "right_collar", "head",
               "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist"]


def rot6d_to_matrix(d6):
    a1, a2 = d6[..., :3], d6[..., 3:]
    b1 = torch.nn.functional.normalize(a1, dim=-1)
    b2 = torch.nn.functional.normalize(a2 - (b1 * a2).sum(-1, keepdim=True) * b1, dim=-1)
    return torch.stack([b1, b2, torch.cross(b1, b2, dim=-1)], dim=-2)


def matrix_to_rot6d(R):
    return R[..., :2, :].reshape(*R.shape[:-2], 6)


def axis_angle_to_matrix(aa):
    th = aa.norm(dim=-1, keepdim=True).clamp(min=1e-8)
    k = aa / th
    K = torch.zeros(*aa.shape[:-1], 3, 3, dtype=aa.dtype, device=aa.device)
    K[..., 0, 1], K[..., 0, 2], K[..., 1, 2] = -k[..., 2], k[..., 1], -k[..., 0]
    K[..., 1, 0], K[..., 2, 0], K[..., 2, 1] = k[..., 2], -k[..., 1], k[..., 0]
    th = th[..., None]
    I = torch.eye(3, dtype=aa.dtype, device=aa.device).expand(K.shape)
    return I + torch.sin(th) * K + (1 - torch.cos(th)) * (K @ K)


def matrix_to_axis_angle(R):
    """Robust log map (uses quaternions)."""
    m = R
    tr = m[..., 0, 0] + m[..., 1, 1] + m[..., 2, 2]
    qw = torch.sqrt((1 + tr).clamp(min=1e-10)) / 2
    qx = torch.sqrt((1 + m[..., 0, 0] - m[..., 1, 1] - m[..., 2, 2]).clamp(min=1e-10)) / 2
    qy = torch.sqrt((1 - m[..., 0, 0] + m[..., 1, 1] - m[..., 2, 2]).clamp(min=1e-10)) / 2
    qz = torch.sqrt((1 - m[..., 0, 0] - m[..., 1, 1] + m[..., 2, 2]).clamp(min=1e-10)) / 2
    qx = torch.copysign(qx, m[..., 2, 1] - m[..., 1, 2])
    qy = torch.copysign(qy, m[..., 0, 2] - m[..., 2, 0])
    qz = torch.copysign(qz, m[..., 1, 0] - m[..., 0, 1])
    v = torch.stack([qx, qy, qz], -1)
    s = v.norm(dim=-1, keepdim=True)
    ang = 2 * torch.atan2(s, qw[..., None])
    return v / s.clamp(min=1e-10) * ang


class SMPLX(torch.nn.Module):
    def __init__(self, gender="neutral", device="cuda", dtype=torch.float32, hands="flat"):
        """hands: "flat" (hand pose 0, as fitted) or "relaxed" (MANO mean hand pose, for display)."""
        super().__init__()
        d = np.load(os.path.join(BODY_MODELS, "smplx", f"SMPLX_{gender.upper()}.npz"), allow_pickle=True)
        t = lambda a: torch.tensor(np.asarray(a), dtype=dtype, device=device)  # noqa: E731
        self.v_template = t(d["v_template"])                              # (V, 3)
        self.shapedirs = t(d["shapedirs"][:, :, :N_BETAS])                # (V, 3, B)
        self.posedirs = t(d["posedirs"].reshape(-1, 486).T)               # (486, V*3)
        self.J_regressor = t(d["J_regressor"])                            # (55, V)
        self.weights = t(d["weights"])                                    # (V, 55)
        self.faces = np.asarray(d["f"], np.int64)
        self.parents = torch.tensor(np.asarray(d["kintree_table"])[0].astype(np.int64), device=device)
        self.parents[0] = -1
        self.n_verts = self.v_template.shape[0]
        self.device, self.dtype = device, dtype
        self.hand_R = None
        if hands == "relaxed":
            h = np.load(HANDS_MEAN_FILE, allow_pickle=True)
            aa = np.concatenate([h["hands_meanl"], h["hands_meanr"]]).reshape(30, 3)
            self.hand_R = axis_angle_to_matrix(torch.tensor(aa, dtype=dtype, device=device))

    # -------------------------------------------------------------------------------------- pieces
    def shaped(self, betas):
        """(B, V, 3) shaped template and (B, 55, 3) rest joints."""
        v = self.v_template + torch.einsum("vcb,nb->nvc", self.shapedirs, betas)
        return v, torch.einsum("jv,nvc->njc", self.J_regressor, v)

    def full_rotmats(self, R_glob, R_body):
        """(N, 55, 3, 3) from global (N, 3, 3) and body (N, 21, 3, 3); the rest identity."""
        N = R_glob.shape[0]
        R = torch.eye(3, dtype=self.dtype, device=self.device).expand(N, N_JOINTS, 3, 3).clone()
        R[:, 0] = R_glob
        R[:, 1:1 + N_BODY] = R_body
        if self.hand_R is not None:
            R[:, HAND_JOINTS[0]:HAND_JOINTS[1]] = self.hand_R
        return R

    def rigid_transforms(self, R, J):
        """Global joint transforms A (N, 55, 4, 4) such that x_posed = A[:, j] @ x_rest_homog (rest-pose relative)."""
        N = R.shape[0]
        Jrel = J.clone()
        Jrel[:, 1:] = J[:, 1:] - J[:, self.parents[1:]]
        T = torch.zeros(N, N_JOINTS, 4, 4, dtype=self.dtype, device=self.device)
        T[..., :3, :3] = R
        T[..., :3, 3] = Jrel
        T[..., 3, 3] = 1
        G = [T[:, 0]]
        for j in range(1, N_JOINTS):
            G.append(G[self.parents[j]] @ T[:, j])
        G = torch.stack(G, 1)
        Jh = torch.cat([J, torch.zeros_like(J[..., :1])], -1)[..., None]   # (N, 55, 4, 1)
        t = G[..., :3, 3] - (G @ Jh)[..., :3, 0]                            # make it rest-relative (out of place)
        return torch.cat([torch.cat([G[..., :3, :3], t[..., None]], -1), G[..., 3:, :]], -2)

    def pose_feature(self, R):
        I = torch.eye(3, dtype=self.dtype, device=self.device)
        return (R[:, 1:] - I).reshape(R.shape[0], -1)                      # (N, 486)

    # -------------------------------------------------------------------------------------- outputs
    def anchors(self, betas, R_glob, R_body, transl, vids, offsets, rigid_joint=None, rigid_offsets=None):
        """Posed anchor points (N, M, 3): skinned vertex vids + offset (expressed in the vertex's rest frame).

        betas (1 or N, B); vids (M,) long; offsets (M, 3) rest-frame offsets; optional rigid points
        (K, 3) attached to joint rigid_joint (joint-local coordinates) appended after the M anchors.
        """
        N = R_glob.shape[0]
        vs, J = self.shaped(betas)
        if vs.shape[0] == 1 and N > 1:
            vs, J = vs.expand(N, -1, -1), J.expand(N, -1, -1)
        R = self.full_rotmats(R_glob, R_body)
        A = self.rigid_transforms(R, J)
        pf = self.pose_feature(R)                                           # (N, 486)
        pd = self.posedirs.view(486, self.n_verts, 3)[:, vids]              # (486, M, 3)
        v = vs[:, vids] + torch.einsum("np,pmc->nmc", pf, pd) + offsets[None]
        Tv = torch.einsum("mj,njab->nmab", self.weights[vids], A)           # (N, M, 4, 4)
        out = (Tv[..., :3, :3] @ v[..., None])[..., 0] + Tv[..., :3, 3]
        if rigid_joint is not None and rigid_offsets is not None:
            Ah = A[:, rigid_joint]                                          # rest-relative transform of the joint
            Jr = J[:, rigid_joint]                                          # rest position of the joint
            p = rigid_offsets[None] + Jr[:, None]                           # joint-local -> rest coordinates
            out = torch.cat([out, (Ah[:, None, :3, :3] @ p[..., None])[..., 0] + Ah[:, None, :3, 3]], 1)
        return out + transl[:, None]

    def forward_full(self, betas, R_glob, R_body, transl):
        """Full mesh (N, V, 3) and joints (N, 55, 3)."""
        N = R_glob.shape[0]
        vs, J = self.shaped(betas)
        if vs.shape[0] == 1 and N > 1:
            vs, J = vs.expand(N, -1, -1), J.expand(N, -1, -1)
        R = self.full_rotmats(R_glob, R_body)
        A = self.rigid_transforms(R, J)
        v = vs + (self.pose_feature(R) @ self.posedirs).view(N, self.n_verts, 3)
        Tv = torch.einsum("vj,njab->nvab", self.weights, A)
        verts = (Tv[..., :3, :3] @ v[..., None])[..., 0] + Tv[..., :3, 3] + transl[:, None]
        Jh = torch.cat([J, torch.ones_like(J[..., :1])], -1)[..., None]
        joints = (A @ Jh)[..., :3, 0] + transl[:, None]
        return verts, joints

    def vertex_transforms(self, betas, R_glob, R_body, vids):
        """Blended skinning transforms (N, M, 4, 4) of vertices vids (to map posed offsets back to rest)."""
        N = R_glob.shape[0]
        vs, J = self.shaped(betas)
        if vs.shape[0] == 1 and N > 1:
            J = J.expand(N, -1, -1)
        A = self.rigid_transforms(self.full_rotmats(R_glob, R_body), J)
        return torch.einsum("mj,njab->nmab", self.weights[vids], A)


def load_vposer(device="cuda"):
    """VPoser v1.0 (decoder used as the body-pose parametrisation and prior)."""
    vp_dir = os.path.join(BODY_MODELS, "vposer_v1_0")
    if vp_dir not in sys.path:
        sys.path.insert(0, vp_dir)
    from vposer_smpl import VPoser
    vp = VPoser(num_neurons=512, latentD=32, data_shape=[1, 21, 3], use_cont_repr=True)
    snap = sorted(f for f in os.listdir(os.path.join(vp_dir, "snapshots")) if f.endswith(".pt"))
    sd = torch.load(os.path.join(vp_dir, "snapshots", snap[-1]), map_location="cpu", weights_only=False)
    sd = sd.get("state_dict", sd) if isinstance(sd, dict) else sd
    vp.load_state_dict(sd, strict=False)
    vp.eval().to(device)
    for p in vp.parameters():
        p.requires_grad_(False)
    return vp


def decode_body(vp, z):
    """VPoser latent (N, 32) -> body rotation matrices (N, 21, 3, 3)."""
    return vp.decode(z, output_type="matrot").view(-1, N_BODY, 3, 3)
