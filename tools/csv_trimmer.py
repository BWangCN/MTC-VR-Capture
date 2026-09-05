#!/usr/bin/env python3
"""
Interactive CSV trajectory trimmer.

Visualises 24-joint body-tracking CSV files in 3D and lets the user
mark start / end points to crop valid trajectory segments.

Usage:
    python csv_trimmer.py  path/to/recording.csv
    python csv_trimmer.py  path/to/recording.csv  -o trimmed/

Controls (keyboard):
    Space       Play / Pause
    Left/Right  Step -1 / +1 frame
    [  /  ]     Step -10 / +10 frames
    A           Set trim START at current frame
    D           Set trim END   at current frame
    S           Save current [start, end] segment to disk
    R           Reset start / end markers
    Q / Esc     Quit

The slider at the bottom can also be dragged to scrub.
Green  vertical line = trim start
Red    vertical line = trim end
Yellow vertical lines = auto-detected trajectory boundaries
"""

import argparse
import csv
import os
import sys
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# ── Joint topology (same as csv24_to_robot.py) ─────────────────────────
JOINT_NAMES = [
    "pelvis", "left_hip", "right_hip", "spine1",
    "left_knee", "right_knee", "spine2",
    "left_ankle", "right_ankle", "spine3",
    "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head",
    "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow",
    "left_wrist", "right_wrist",
    "left_hand", "right_hand",
]

# Bone connections for drawing the skeleton
BONES = [
    # spine
    ("pelvis", "spine1"), ("spine1", "spine2"), ("spine2", "spine3"),
    ("spine3", "neck"), ("neck", "head"),
    # left leg
    ("pelvis", "left_hip"), ("left_hip", "left_knee"),
    ("left_knee", "left_ankle"), ("left_ankle", "left_foot"),
    # right leg
    ("pelvis", "right_hip"), ("right_hip", "right_knee"),
    ("right_knee", "right_ankle"), ("right_ankle", "right_foot"),
    # left arm
    ("spine3", "left_collar"), ("left_collar", "left_shoulder"),
    ("left_shoulder", "left_elbow"), ("left_elbow", "left_wrist"),
    ("left_wrist", "left_hand"),
    # right arm
    ("spine3", "right_collar"), ("right_collar", "right_shoulder"),
    ("right_shoulder", "right_elbow"), ("right_elbow", "right_wrist"),
    ("right_wrist", "right_hand"),
]

BONE_INDICES = [(JOINT_NAMES.index(a), JOINT_NAMES.index(b)) for a, b in BONES]

# ── CSV loading ────────────────────────────────────────────────────────
def load_csv(csv_path: str):
    """Return (header_row, rows_as_dicts, positions_array[N,24,3], times[N])."""
    rows = []
    with open(csv_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        for row in reader:
            rows.append(row)

    n = len(rows)
    positions = np.zeros((n, 24, 3), dtype=np.float64)
    times = np.zeros(n, dtype=np.float64)
    frames = np.zeros(n, dtype=np.int64)

    for i, row in enumerate(rows):
        times[i] = float(row.get("time", i))
        frames[i] = int(row.get("frame", i))
        for j in range(24):
            positions[i, j, 0] = float(row[f"p{j}x"])
            positions[i, j, 1] = float(row[f"p{j}y"])
            positions[i, j, 2] = float(row[f"p{j}z"])

    return fieldnames, rows, positions, times, frames


# ── Auto boundary detection ───────────────────────────────────────────
def detect_boundaries(positions, times, pos_thresh=0.5, time_thresh=1.0):
    """
    Detect likely trajectory boundaries by looking for:
      - Large pelvis position jumps (> pos_thresh metres)
      - Large time gaps (> time_thresh seconds)
    Returns list of frame indices where a new trajectory begins.
    """
    boundaries = []
    pelvis = positions[:, 0, :]  # (N, 3)
    for i in range(1, len(pelvis)):
        dist = np.linalg.norm(pelvis[i] - pelvis[i - 1])
        dt = abs(times[i] - times[i - 1])
        if dist > pos_thresh or dt > time_thresh:
            boundaries.append(i)
    return boundaries


# ── Main interactive viewer ───────────────────────────────────────────
class CsvTrimmer:
    def __init__(self, csv_path, output_dir):
        self.csv_path = csv_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        print(f"Loading {csv_path} ...")
        self.fieldnames, self.rows, self.positions, self.times, self.frame_ids = load_csv(csv_path)
        self.n_frames = len(self.rows)
        print(f"  {self.n_frames} frames, time {self.times[0]:.3f}–{self.times[-1]:.3f}s")

        self.boundaries = detect_boundaries(self.positions, self.times)
        if self.boundaries:
            print(f"  Auto-detected {len(self.boundaries)} trajectory boundary(ies) at rows: {self.boundaries}")
        else:
            print("  No trajectory boundaries detected.")

        # trim markers
        self.trim_start = None  # row index
        self.trim_end = None
        self.current = 0
        self.playing = False
        self.save_counter = 0

        # ── figure setup ──
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title("CSV Trajectory Trimmer")
        self.ax3d = self.fig.add_axes([0.05, 0.25, 0.55, 0.70], projection="3d")
        self.ax_traj = self.fig.add_axes([0.05, 0.08, 0.75, 0.12])
        self.ax_slider = self.fig.add_axes([0.05, 0.02, 0.75, 0.03])

        # info text
        self.ax_info = self.fig.add_axes([0.65, 0.25, 0.33, 0.70])
        self.ax_info.axis("off")

        # buttons
        self.ax_btn_start = self.fig.add_axes([0.83, 0.17, 0.07, 0.04])
        self.ax_btn_end = self.fig.add_axes([0.91, 0.17, 0.07, 0.04])
        self.ax_btn_save = self.fig.add_axes([0.83, 0.12, 0.07, 0.04])
        self.ax_btn_reset = self.fig.add_axes([0.91, 0.12, 0.07, 0.04])
        self.ax_btn_play = self.fig.add_axes([0.83, 0.07, 0.15, 0.04])

        self.btn_start = Button(self.ax_btn_start, "Set Start")
        self.btn_end = Button(self.ax_btn_end, "Set End")
        self.btn_save = Button(self.ax_btn_save, "Save")
        self.btn_reset = Button(self.ax_btn_reset, "Reset")
        self.btn_play = Button(self.ax_btn_play, "Play / Pause")

        self.btn_start.on_clicked(lambda _: self.set_start())
        self.btn_end.on_clicked(lambda _: self.set_end())
        self.btn_save.on_clicked(lambda _: self.save_segment())
        self.btn_reset.on_clicked(lambda _: self.reset_markers())
        self.btn_play.on_clicked(lambda _: self.toggle_play())

        # slider
        self.slider = Slider(self.ax_slider, "Frame", 0, self.n_frames - 1,
                             valinit=0, valstep=1, valfmt="%d")
        self.slider.on_changed(self.on_slider)

        # trajectory overview (pelvis Y vs X as top-down path)
        pelvis_xy = self.positions[:, 0, :]  # (N, 3)
        self.ax_traj.plot(range(self.n_frames), pelvis_xy[:, 0], "b-", lw=0.5, label="pelvis_x")
        self.ax_traj.plot(range(self.n_frames), pelvis_xy[:, 2], "r-", lw=0.5, label="pelvis_z")
        self.ax_traj.set_xlim(0, self.n_frames - 1)
        self.ax_traj.set_ylabel("pos")
        self.ax_traj.legend(loc="upper right", fontsize=7)
        self.ax_traj.set_title("Pelvis X (blue) / Z (red) over frames", fontsize=8)

        # draw boundary lines
        for b in self.boundaries:
            self.ax_traj.axvline(b, color="gold", lw=1, ls="--", alpha=0.7)

        # markers on trajectory axis
        self.traj_cursor = self.ax_traj.axvline(0, color="black", lw=1)
        self.traj_start_line = None
        self.traj_end_line = None

        # keyboard
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

        # initial draw
        self._draw_skeleton()
        self._update_info()

        # timer for playback
        self.timer = self.fig.canvas.new_timer(interval=33)  # ~30 fps
        self.timer.add_callback(self._tick)
        self.timer.start()

    # ── drawing helpers ──
    def _draw_skeleton(self):
        self.ax3d.cla()
        pos = self.positions[self.current]  # (24, 3)

        # joints
        self.ax3d.scatter(pos[:, 0], pos[:, 2], pos[:, 1],
                          c="dodgerblue", s=30, depthshade=True)

        # bones
        for a, b in BONE_INDICES:
            xs = [pos[a, 0], pos[b, 0]]
            ys = [pos[a, 2], pos[b, 2]]
            zs = [pos[a, 1], pos[b, 1]]
            self.ax3d.plot(xs, ys, zs, "k-", lw=1.5)

        # pelvis marker
        self.ax3d.scatter([pos[0, 0]], [pos[0, 2]], [pos[0, 1]],
                          c="red", s=60, marker="^")

        # axes labels
        self.ax3d.set_xlabel("X")
        self.ax3d.set_ylabel("Z")
        self.ax3d.set_zlabel("Y (up)")

        # set limits centred on pelvis
        cx, cy, cz = pos[0, 0], pos[0, 2], pos[0, 1]
        r = 1.2
        self.ax3d.set_xlim(cx - r, cx + r)
        self.ax3d.set_ylim(cy - r, cy + r)
        self.ax3d.set_zlim(cz - 0.3, cz + 2 * r)

        row_id = self.frame_ids[self.current]
        t = self.times[self.current]
        self.ax3d.set_title(f"Row {self.current}/{self.n_frames-1}  |  frame={row_id}  t={t:.3f}s",
                            fontsize=10)

    def _update_info(self):
        self.ax_info.cla()
        self.ax_info.axis("off")

        lines = [
            f"File: {Path(self.csv_path).name}",
            f"Total rows: {self.n_frames}",
            f"Time range: {self.times[0]:.3f} – {self.times[-1]:.3f}s",
            f"Boundaries: {len(self.boundaries)}",
            "",
            f"Current row: {self.current}",
            f"  frame = {self.frame_ids[self.current]}",
            f"  time  = {self.times[self.current]:.4f}s",
            "",
            "── Trim ──",
            f"Start: {self.trim_start if self.trim_start is not None else '(not set)'}",
            f"End:   {self.trim_end if self.trim_end is not None else '(not set)'}",
        ]

        if self.trim_start is not None and self.trim_end is not None:
            seg_len = self.trim_end - self.trim_start + 1
            seg_dur = self.times[self.trim_end] - self.times[self.trim_start]
            lines.append(f"Length: {seg_len} frames ({seg_dur:.3f}s)")

        lines += [
            "",
            "── Controls ──",
            "Space    Play/Pause",
            "←/→      ±1 frame",
            "[/]      ±10 frames",
            "A        Set start",
            "D        Set end",
            "S        Save segment",
            "R        Reset markers",
            "Q/Esc    Quit",
        ]

        text = "\n".join(lines)
        self.ax_info.text(0.0, 1.0, text, transform=self.ax_info.transAxes,
                          fontsize=9, verticalalignment="top", fontfamily="monospace")

    def _update_traj_markers(self):
        # cursor
        self.traj_cursor.set_xdata([self.current, self.current])

        # start line
        if self.traj_start_line:
            self.traj_start_line.remove()
            self.traj_start_line = None
        if self.trim_start is not None:
            self.traj_start_line = self.ax_traj.axvline(
                self.trim_start, color="green", lw=2, ls="-")

        # end line
        if self.traj_end_line:
            self.traj_end_line.remove()
            self.traj_end_line = None
        if self.trim_end is not None:
            self.traj_end_line = self.ax_traj.axvline(
                self.trim_end, color="red", lw=2, ls="-")

    def _refresh(self):
        self._draw_skeleton()
        self._update_info()
        self._update_traj_markers()
        self.slider.set_val(self.current)
        self.fig.canvas.draw_idle()

    # ── callbacks ──
    def on_slider(self, val):
        idx = int(val)
        if idx != self.current:
            self.current = idx
            self._draw_skeleton()
            self._update_info()
            self._update_traj_markers()
            self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == " ":
            self.toggle_play()
        elif event.key == "right":
            self.step(1)
        elif event.key == "left":
            self.step(-1)
        elif event.key == "]":
            self.step(10)
        elif event.key == "[":
            self.step(-10)
        elif event.key == "a":
            self.set_start()
        elif event.key == "d":
            self.set_end()
        elif event.key == "s":
            self.save_segment()
        elif event.key == "r":
            self.reset_markers()
        elif event.key in ("q", "escape"):
            plt.close(self.fig)

    def step(self, delta):
        self.current = max(0, min(self.n_frames - 1, self.current + delta))
        self._refresh()

    def toggle_play(self):
        self.playing = not self.playing

    def _tick(self):
        if self.playing:
            if self.current < self.n_frames - 1:
                self.current += 1
                self._refresh()
            else:
                self.playing = False

    # ── trim actions ──
    def set_start(self):
        self.trim_start = self.current
        print(f"  Trim START set to row {self.current} (frame={self.frame_ids[self.current]}, t={self.times[self.current]:.3f}s)")
        self._refresh()

    def set_end(self):
        self.trim_end = self.current
        print(f"  Trim END set to row {self.current} (frame={self.frame_ids[self.current]}, t={self.times[self.current]:.3f}s)")
        self._refresh()

    def reset_markers(self):
        self.trim_start = None
        self.trim_end = None
        print("  Markers reset.")
        self._refresh()

    def save_segment(self):
        if self.trim_start is None or self.trim_end is None:
            print("  ERROR: Set both start and end before saving!")
            return
        if self.trim_start > self.trim_end:
            self.trim_start, self.trim_end = self.trim_end, self.trim_start

        seg_rows = self.rows[self.trim_start: self.trim_end + 1]
        seg_len = len(seg_rows)
        seg_dur = self.times[self.trim_end] - self.times[self.trim_start]

        stem = Path(self.csv_path).stem
        self.save_counter += 1
        out_name = f"{stem}_trim{self.save_counter:02d}.csv"
        out_path = self.output_dir / out_name

        with open(out_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames)
            writer.writeheader()
            writer.writerows(seg_rows)

        print(f"  SAVED: {out_path}")
        print(f"         rows {self.trim_start}–{self.trim_end}  ({seg_len} frames, {seg_dur:.3f}s)")

    # ── run ──
    def run(self):
        plt.show()


# ── CLI ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Interactive CSV trajectory trimmer with 3D skeleton preview.")
    parser.add_argument("csv_file", help="Path to the 24-joint body-tracking CSV.")
    parser.add_argument("-o", "--output", default=None,
                        help="Output directory for trimmed CSVs (default: same dir as input).")
    args = parser.parse_args()

    csv_path = args.csv_file
    if not os.path.isfile(csv_path):
        print(f"ERROR: file not found: {csv_path}")
        sys.exit(1)

    output_dir = args.output if args.output else str(Path(csv_path).parent)

    trimmer = CsvTrimmer(csv_path, output_dir)
    trimmer.run()


if __name__ == "__main__":
    main()
