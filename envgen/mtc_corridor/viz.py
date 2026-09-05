"""
Visualization: matplotlib top-down + side-view, open3d offscreen 3D.

Everything is headless. The top-down/side-view backends use matplotlib's
"Agg" backend (no GUI). The 3D renderer uses open3d's OffscreenRenderer;
if that fails for any reason (driver, OpenGL context) the function falls
back to matplotlib mplot3d so a PNG is always produced.

Color convention is the single source of truth in constraint_rules.py.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Optional, Sequence

# Force non-interactive backend BEFORE importing pyplot.
import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .constraint_rules import (
    CONSTRAINT_COLORS,
    CORRIDOR_COLOR,
    GOAL_MARKER_COLOR,
    START_MARKER_COLOR,
)
from .corridor import CorridorEnv
from .obstacle import ObstacleInstance

DEFAULT_DPI = 110
DEFAULT_FIG_W = 12
DEFAULT_FIG_H = 8


# ============================================================================
# TOP-DOWN VIEW (x-y plane, looking down -z)
# ============================================================================


def render_top_down(env: CorridorEnv, out_path) -> Path:
    """Top-down schematic: rectangles for obstacles, color-coded by constraint,
    annotated with constraint type abbreviation + d_i.

    Title: env_id | tier | D_env=... | n_obstacles=...
    """
    out_path = Path(out_path)

    fig, ax = plt.subplots(figsize=(DEFAULT_FIG_W, DEFAULT_FIG_H), dpi=DEFAULT_DPI)

    # Corridor outline (walls)
    corridor_rect = Rectangle(
        (0.0, 0.0),
        env.length,
        env.width,
        facecolor="#f5f5f5",
        edgecolor=CORRIDOR_COLOR,
        linewidth=2.5,
        zorder=1,
    )
    ax.add_patch(corridor_rect)

    # Start/goal markers
    ax.scatter(
        env.start_pos[0],
        env.start_pos[1],
        s=260,
        c=START_MARKER_COLOR,
        marker="o",
        edgecolors="black",
        linewidths=1.0,
        zorder=5,
        label="start",
    )
    ax.scatter(
        env.goal_pos[0],
        env.goal_pos[1],
        s=260,
        c=GOAL_MARKER_COLOR,
        marker="s",
        edgecolors="black",
        linewidths=1.0,
        zorder=5,
        label="goal",
    )
    # Arrow from start to goal (visual cue for travel direction)
    ax.annotate(
        "",
        xy=env.goal_pos[:2],
        xytext=env.start_pos[:2],
        arrowprops=dict(arrowstyle="->", color="black", lw=1.0, alpha=0.30),
        zorder=2,
    )

    # Obstacles
    for obs in env.obstacles:
        _draw_obstacle_top_down(ax, obs)

    # Axes / title / legend
    margin = 0.4
    ax.set_xlim(-margin, env.length + margin)
    ax.set_ylim(-margin, env.width + margin)
    ax.set_aspect("equal")
    ax.set_xlabel("x  (corridor travel, metres)")
    ax.set_ylabel("y  (lateral, metres)")
    ax.set_title(
        f"{env.env_id}  |  tier={env.tier}  |  "
        f"D_env={env.env_difficulty:.3f}  |  n_obstacles={len(env.obstacles)}",
        fontsize=12,
    )
    ax.grid(True, alpha=0.25)

    # Constraint legend (every constraint actually represented in the env,
    # including those carried by compound obstacles like block_with_clutter).
    present = sorted({c for o in env.obstacles for c in o.constraints})
    legend_patches = [mpatches.Patch(color=CONSTRAINT_COLORS[c], label=c) for c in present]
    if legend_patches:
        ax.legend(
            handles=legend_patches,
            loc="lower center",
            bbox_to_anchor=(0.5, -0.18),
            ncol=min(len(legend_patches), 5),
            frameon=False,
            fontsize=10,
        )

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _draw_obstacle_top_down(ax, obs: ObstacleInstance) -> None:
    """Draw all sub-AABBs of `obs` projected to x-y; annotate the leftmost
    AABB with the constraint abbreviation + d_i.

    Compound obstacles (block_with_clutter) color sub-AABBs by sub-role:
    the wall block uses the lateral_passage color, clutter items use the
    leg_clearance color, so both constraint kinds are visually identifiable."""
    aabbs = obs.aabbs()
    sub_colors = _sub_aabb_colors(obs, aabbs)

    for (lo, hi), color in zip(aabbs, sub_colors):
        rect = Rectangle(
            (lo[0], lo[1]),
            hi[0] - lo[0],
            hi[1] - lo[1],
            facecolor=color,
            edgecolor="black",
            linewidth=0.7,
            alpha=0.85,
            zorder=3,
        )
        ax.add_patch(rect)

    # Annotation at the top of the obstacle's bounding span.
    lo_all = np.min([lo for lo, _ in aabbs], axis=0)
    hi_all = np.max([hi for _, hi in aabbs], axis=0)
    cx_ann = (lo_all[0] + hi_all[0]) / 2.0
    cy_ann = hi_all[1] + 0.06
    # For compound, abbreviate ALL constraints joined.
    abbr = "+".join(_constraint_abbr(c) for c in obs.constraints) if obs.constraints else "?"
    txt = f"{abbr}\nd={obs.difficulty:.2f}"
    primary_color = CONSTRAINT_COLORS[obs.constraints[0]] if obs.constraints else "#888"
    ax.text(
        cx_ann,
        cy_ann,
        txt,
        ha="center",
        va="bottom",
        fontsize=8,
        zorder=4,
        bbox=dict(
            boxstyle="round,pad=0.18",
            facecolor="white",
            edgecolor=primary_color,
            lw=0.8,
            alpha=0.92,
        ),
    )


def _sub_aabb_colors(obs: ObstacleInstance, aabbs) -> list[str]:
    """Return one color per sub-AABB. v1: compound `block_with_clutter`
    paints the wall block with lateral_passage color and each clutter
    item with leg_clearance color; everything else uses the obstacle's
    primary constraint color."""
    if obs.obstacle_type == "block_with_clutter":
        # By construction, the first AABB is the wall block (see
        # _aabbs_block_with_clutter); the rest are clutter items.
        primary = CONSTRAINT_COLORS["lateral_passage"]
        clutter = CONSTRAINT_COLORS["leg_clearance"]
        return [primary] + [clutter] * (len(aabbs) - 1)
    primary = CONSTRAINT_COLORS[obs.constraints[0]] if obs.constraints else "#888"
    return [primary] * len(aabbs)


def _constraint_abbr(c: str) -> str:
    return {
        "height_constraint": "HEIGHT",
        "full_body_lowering": "FULL-LOW",
        "leg_clearance": "LEG-CLR",
        "lateral_passage": "LAT-PASS",
        "lateral_clearance": "LAT-CLR",
    }.get(c, c.upper())


# ============================================================================
# SIDE VIEW (x-z plane, looking along -y)
# ============================================================================


def render_side_view(env: CorridorEnv, out_path) -> Path:
    """Side-view (x-z): shows vertical structure clearly. Annotates heights.

    Obstacles projected onto the x-z plane (their y-extent collapses to a
    rectangle). This is exactly what the robot 'sees' if standing on the
    corridor centerline."""
    out_path = Path(out_path)

    fig, ax = plt.subplots(figsize=(DEFAULT_FIG_W, DEFAULT_FIG_H * 0.55), dpi=DEFAULT_DPI)

    # Floor + wall outline (we visualize the ceiling cap at wall_height too)
    ax.axhline(0.0, color=CORRIDOR_COLOR, linewidth=2.5, zorder=1)
    ax.axhline(
        env.wall_height, color=CORRIDOR_COLOR, linewidth=1.0, linestyle="--", alpha=0.6, zorder=1
    )

    # G1 reference lines
    ax.axhline(1.32, color="gray", linestyle=":", linewidth=1.0, alpha=0.6)
    ax.text(
        env.length, 1.34, "G1 standing 1.32m", ha="right", va="bottom", fontsize=8, color="gray"
    )
    ax.axhline(0.50, color="gray", linestyle=":", linewidth=1.0, alpha=0.6)
    ax.text(env.length, 0.52, "G1 knee 0.50m", ha="right", va="bottom", fontsize=8, color="gray")

    # Obstacles — centerline cross-section in x-z
    for obs in env.obstacles:
        _draw_obstacle_side_view(ax, obs, env.width)

    # Start/goal markers at floor
    ax.scatter(
        [env.start_pos[0]],
        [0.0],
        s=160,
        c=START_MARKER_COLOR,
        marker="o",
        edgecolors="black",
        zorder=5,
    )
    ax.scatter(
        [env.goal_pos[0]],
        [0.0],
        s=160,
        c=GOAL_MARKER_COLOR,
        marker="s",
        edgecolors="black",
        zorder=5,
    )

    ax.set_xlim(-0.3, env.length + 0.3)
    ax.set_ylim(-0.1, env.wall_height + 0.3)
    ax.set_xlabel("x  (corridor travel, metres)")
    ax.set_ylabel("z  (height, metres)")
    ax.set_title(
        f"{env.env_id}  —  side view  (heights annotated)",
        fontsize=12,
    )
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal")

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _draw_obstacle_side_view(ax, obs: ObstacleInstance, corridor_width: float) -> None:  # noqa: C901
    aabbs = obs.aabbs()
    sub_colors = _sub_aabb_colors(obs, aabbs)
    """Side view = corridor centerline cross-section in x-z. Only draw
    sub-AABBs whose y-extent crosses the centerline (y = corridor_width / 2).

    Effect:
      - beam / ground: span y fully or pass centerline -> drawn full-height
      - dog_hole: only the top lintel intersects the centerline -> opening
        below is visible as empty space
      - pillars: don't cross centerline -> NOT drawn (shown in top-down)
      - lateral_block: drawn faded if not crossing centerline -> indicates
        a lateral obstacle that doesn't block the centerline path
    """
    primary_color = CONSTRAINT_COLORS[obs.constraints[0]] if obs.constraints else "#888"
    centerline_y = corridor_width / 2.0

    has_centerline_part = False
    for (lo, hi), color in zip(aabbs, sub_colors):
        crosses = lo[1] <= centerline_y <= hi[1]
        alpha = 0.85 if crosses else 0.18
        rect = Rectangle(
            (lo[0], lo[2]),
            hi[0] - lo[0],
            hi[2] - lo[2],
            facecolor=color,
            edgecolor="black",
            linewidth=0.7,
            alpha=alpha,
            zorder=3,
        )
        ax.add_patch(rect)
        has_centerline_part = has_centerline_part or crosses
    color = primary_color  # used by annotation arrows below

    # Per-type height annotation
    cp = obs.constraint_params
    cx = float(obs.position[0])
    if "height_constraint" in cp:
        H = cp["height_constraint"]
        ax.annotate(
            f"H={H:.2f}",
            xy=(cx, H),
            xytext=(cx, max(0.05, H - 0.30)),
            ha="center",
            fontsize=7,
            color=color,
            arrowprops=dict(arrowstyle="->", color=color, lw=0.7),
        )
    elif "full_body_lowering" in cp:
        H = cp["full_body_lowering"]
        # Label the OPENING (below the lintel)
        ax.text(
            cx,
            H / 2.0,
            f"opening\n{H:.2f}",
            ha="center",
            va="center",
            fontsize=7,
            color=color,
            bbox=dict(
                boxstyle="round,pad=0.15", facecolor="white", edgecolor=color, lw=0.6, alpha=0.9
            ),
        )
    elif "leg_clearance" in cp:
        H = cp["leg_clearance"]
        ax.text(
            cx,
            H + 0.04,
            f"{H:.2f}",
            ha="center",
            va="bottom",
            fontsize=7,
            color=color,
        )
    elif "lateral_clearance" in cp:
        # Pillars don't cross the centerline; if not visible, drop a marker.
        if not has_centerline_part:
            g = cp["lateral_clearance"]
            ax.text(
                cx,
                0.05,
                f"pillars\ng={g:.2f}",
                ha="center",
                va="bottom",
                fontsize=7,
                color=color,
                alpha=0.7,
            )
    elif "lateral_passage" in cp:
        if not has_centerline_part:
            p = cp["lateral_passage"]
            ax.text(
                cx,
                0.05,
                f"lat blk\np={p:.2f}",
                ha="center",
                va="bottom",
                fontsize=7,
                color=color,
                alpha=0.7,
            )


# ============================================================================
# 3D VIEW
# ============================================================================


def render_3d(env: CorridorEnv, out_path, width: int = 1200, height: int = 800) -> Path:
    """Try pyrender OffscreenRenderer; fall back to matplotlib mplot3d.

    Pyrender uses pyglet's offscreen pbuffer on Windows -- works without a
    display server. The mplot3d fallback has known z-order glitches with
    many overlapping boxes but is always available."""
    out_path = Path(out_path)
    try:
        return _render_3d_pyrender(env, out_path, width, height)
    except Exception as e:
        print(f"[viz] pyrender offscreen failed ({e!r}); falling back to mplot3d.", file=sys.stderr)
        return _render_3d_mplot3d(env, out_path)


def _render_3d_pyrender(env: CorridorEnv, out_path: Path, width: int, height: int) -> Path:
    """Pyrender PBR offscreen render. Floor + 2 walls + obstacles + markers."""
    import pyrender
    import trimesh as tm

    scene = pyrender.Scene(
        bg_color=[1.0, 1.0, 1.0, 1.0],
        ambient_light=[0.45, 0.45, 0.45],
    )

    def _add_box(extents, center, rgba):
        box = tm.creation.box(extents=tuple(extents))
        box.apply_translation(np.asarray(center, dtype=np.float64))
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=list(rgba),
            metallicFactor=0.05,
            roughnessFactor=0.85,
            alphaMode="OPAQUE",
        )
        mesh = pyrender.Mesh.from_trimesh(box, material=material, smooth=False)
        scene.add(mesh)

    # Floor
    _add_box(
        [env.length, env.width, 0.04],
        [env.length / 2.0, env.width / 2.0, -0.02],
        [0.88, 0.88, 0.88, 1.0],
    )

    # Walls intentionally omitted: in a long+narrow corridor the near wall
    # blocks the camera view of all obstacles. The floor's footprint plus
    # the wall-spanning obstacles (beam / dog_hole) make the corridor
    # boundary unambiguous in the render. (Walls ARE drawn in top-down.)

    # Obstacles — per-sub-AABB coloring so compound types are readable.
    for obs in env.obstacles:
        aabbs = obs.aabbs()
        sub_colors = _sub_aabb_colors(obs, aabbs)
        for (lo, hi), hex_color in zip(aabbs, sub_colors):
            rgb = _hex_to_rgb(hex_color)
            rgba = (*rgb, 1.0)
            size = hi - lo
            center = (lo + hi) / 2.0
            _add_box(size.tolist(), center.tolist(), rgba)

    # Start / goal spheres
    def _add_sphere(center, rgba, radius=0.12):
        sph = tm.creation.icosphere(subdivisions=2, radius=radius)
        sph.apply_translation(np.asarray(center, dtype=np.float64))
        material = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=list(rgba),
            metallicFactor=0.2,
            roughnessFactor=0.4,
            alphaMode="OPAQUE",
        )
        mesh = pyrender.Mesh.from_trimesh(sph, material=material, smooth=True)
        scene.add(mesh)

    _add_sphere(env.start_pos.tolist(), (*_hex_to_rgb(START_MARKER_COLOR), 1.0), radius=0.15)
    _add_sphere(env.goal_pos.tolist(), (*_hex_to_rgb(GOAL_MARKER_COLOR), 1.0), radius=0.15)

    # Camera: high-elevation isometric (tilted bird's-eye). Sits well
    # above the scene and a bit behind/laterally, so all obstacles project
    # cleanly onto the floor without one wall occluding the rest.
    cam = pyrender.PerspectiveCamera(yfov=np.deg2rad(45.0), aspectRatio=width / height)
    center = np.array([env.length / 2.0, env.width / 2.0, 0.4])
    eye = np.array(
        [
            env.length / 2.0 - env.length * 0.20,
            -env.width * 0.8 - 0.5,
            max(env.length * 0.95, 5.5),
        ]
    )
    cam_pose = _look_at(eye, center, up=np.array([0.0, 0.0, 1.0]))
    scene.add(cam, pose=cam_pose)

    # Sun-like directional light from above + slight angle.
    light = pyrender.DirectionalLight(color=np.array([1.0, 0.97, 0.92]), intensity=4.5)
    light_pose = _look_at(
        np.array([env.length / 2.0 + 1.0, env.width / 2.0 - 1.5, 4.0]),
        np.array([env.length / 2.0, env.width / 2.0, 0.5]),
        up=np.array([0.0, 1.0, 0.0]),
    )
    scene.add(light, pose=light_pose)
    # Fill light from the opposite side
    fill = pyrender.DirectionalLight(color=np.array([0.9, 0.93, 1.0]), intensity=1.6)
    fill_pose = _look_at(
        np.array([env.length / 2.0 - 1.0, env.width / 2.0 + 1.5, 3.0]),
        np.array([env.length / 2.0, env.width / 2.0, 0.5]),
        up=np.array([0.0, -1.0, 0.0]),
    )
    scene.add(fill, pose=fill_pose)

    renderer = pyrender.OffscreenRenderer(viewport_width=width, viewport_height=height)
    try:
        color, _depth = renderer.render(scene)
    finally:
        renderer.delete()

    import matplotlib.image as mpimg

    mpimg.imsave(out_path, color)
    return out_path


def _look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Build a 4x4 pyrender camera/light pose from eye/target/up.

    Pyrender's camera convention: -Z is forward, +Y is up. We construct
    a column-major basis (right, up, back) and place it at `eye`.
    """
    forward = target - eye
    forward = forward / (np.linalg.norm(forward) + 1e-9)
    right = np.cross(forward, up)
    right = right / (np.linalg.norm(right) + 1e-9)
    real_up = np.cross(right, forward)
    pose = np.eye(4)
    pose[:3, 0] = right
    pose[:3, 1] = real_up
    pose[:3, 2] = -forward  # pyrender forward is -Z
    pose[:3, 3] = eye
    return pose


def _render_3d_mplot3d(env: CorridorEnv, out_path: Path) -> Path:
    """Pure-matplotlib fallback. Boxes drawn as 6 quad faces each."""
    fig = plt.figure(figsize=(DEFAULT_FIG_W, DEFAULT_FIG_H), dpi=DEFAULT_DPI)
    ax = fig.add_subplot(111, projection="3d")

    # Floor
    _draw_box_mpl(
        ax,
        np.array([0, 0, -0.01]),
        np.array([env.length, env.width, 0.0]),
        face="#f0f0f0",
        edge="#bbb",
    )
    # Side walls (only outlines to avoid clutter)
    for y0, y1 in ((0.0, 0.0), (env.width, env.width)):
        _draw_box_mpl(
            ax,
            np.array([0, y0 - 0.025, 0]),
            np.array([env.length, y1 + 0.025, env.wall_height]),
            face="#e5e5e5",
            edge="#aaa",
            alpha=0.25,
        )

    # Obstacles
    for obs in env.obstacles:
        color = CONSTRAINT_COLORS[obs.constraints[0]]
        for lo, hi in obs.aabbs():
            _draw_box_mpl(ax, lo, hi, face=color, edge="black", alpha=0.85)

    # Start/goal
    ax.scatter(*env.start_pos, s=180, c=START_MARKER_COLOR, edgecolors="black", depthshade=True)
    ax.scatter(*env.goal_pos, s=180, c=GOAL_MARKER_COLOR, edgecolors="black", depthshade=True)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_xlim(0, env.length)
    ax.set_ylim(0, env.width)
    ax.set_zlim(0, env.wall_height)
    ax.set_title(f"{env.env_id}  —  3D view (mplot3d fallback)", fontsize=11)
    ax.view_init(elev=28.0, azim=-120.0)
    try:
        ax.set_box_aspect((env.length, env.width, env.wall_height))
    except Exception:
        pass
    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _draw_box_mpl(
    ax, lo: np.ndarray, hi: np.ndarray, face: str, edge: str, alpha: float = 0.9
) -> None:
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    verts = [
        # bottom (z=z0)
        [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
        # top (z=z1)
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
        # y=y0
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        # y=y1
        [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
        # x=x0
        [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
        # x=x1
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
    ]
    poly = Poly3DCollection(verts, alpha=alpha, facecolor=face, edgecolor=edge, linewidths=0.5)
    ax.add_collection3d(poly)


def _hex_to_rgb(hexstr: str) -> tuple[float, float, float]:
    s = hexstr.lstrip("#")
    return (int(s[0:2], 16) / 255.0, int(s[2:4], 16) / 255.0, int(s[4:6], 16) / 255.0)


# ============================================================================
# Summary figure across multiple environments
# ============================================================================


def render_dataset_summary(
    envs: Sequence[CorridorEnv],
    out_path,
) -> Path:
    """One figure with three panels:
    (a) D_env histogram, colored by tier
    (b) per-constraint count bar chart, faceted by tier
    (c) obstacle-count distribution, faceted by tier
    """
    out_path = Path(out_path)

    tiers = ["easy", "medium", "hard"]
    tier_colors = {"easy": "#7fbf7b", "medium": "#fdbf6f", "hard": "#cc4c4c"}
    from .constraint_rules import CONSTRAINT_TYPES

    by_tier: dict[str, list[CorridorEnv]] = {t: [] for t in tiers}
    for e in envs:
        if e.tier in by_tier:
            by_tier[e.tier].append(e)

    fig = plt.figure(figsize=(15, 10), dpi=DEFAULT_DPI)
    gs = fig.add_gridspec(2, 2)

    # (a) D_env histogram
    ax_a = fig.add_subplot(gs[0, 0])
    for t in tiers:
        d_vals = [e.env_difficulty for e in by_tier[t]]
        if d_vals:
            ax_a.hist(
                d_vals,
                bins=10,
                range=(0, 1),
                alpha=0.7,
                label=t,
                color=tier_colors[t],
                edgecolor="black",
                linewidth=0.5,
            )
    ax_a.set_xlabel("D_env (max d_i)")
    ax_a.set_ylabel("count")
    ax_a.set_title("(a) D_env distribution by tier")
    ax_a.legend()
    ax_a.grid(True, alpha=0.25)

    # (b) Constraint counts faceted by tier
    ax_b = fig.add_subplot(gs[0, 1])
    n_tiers = len(tiers)
    width = 0.8 / n_tiers
    x_positions = np.arange(len(CONSTRAINT_TYPES))
    for i, t in enumerate(tiers):
        counts = [sum(e.constraint_counts.get(c, 0) for e in by_tier[t]) for c in CONSTRAINT_TYPES]
        ax_b.bar(
            x_positions + i * width - 0.4 + width / 2,
            counts,
            width,
            label=t,
            color=tier_colors[t],
            edgecolor="black",
            linewidth=0.5,
        )
    ax_b.set_xticks(x_positions)
    ax_b.set_xticklabels([_constraint_abbr(c) for c in CONSTRAINT_TYPES], rotation=30, ha="right")
    ax_b.set_ylabel("total count across envs")
    ax_b.set_title("(b) Constraint type counts by tier")
    ax_b.legend()
    ax_b.grid(True, alpha=0.25, axis="y")

    # (c) Obstacles-per-env distribution
    ax_c = fig.add_subplot(gs[1, :])
    for t in tiers:
        n_vals = [len(e.obstacles) for e in by_tier[t]]
        if n_vals:
            ax_c.hist(
                n_vals,
                bins=np.arange(0, 14) - 0.5,
                alpha=0.7,
                label=t,
                color=tier_colors[t],
                edgecolor="black",
                linewidth=0.5,
            )
    ax_c.set_xlabel("# obstacles per env")
    ax_c.set_ylabel("count")
    ax_c.set_title("(c) Obstacle count distribution by tier")
    ax_c.legend()
    ax_c.grid(True, alpha=0.25)

    fig.suptitle(f"MTC Challenge v1 — dataset summary  ({len(envs)} envs)", fontsize=14)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    return out_path
