"""Still images from the viewer page: top view per take, and close-ups where each take passes each gate.

    python analysis/render_stills.py analysis/data/raw/2026-10-30_recordings/recordings \
        [--viewer analysis/data/out/viewer/mocap_scene_viewer.html] [--out analysis/data/out/stills]

Writes top_<k>.png, gate<g>_<k>_{side,above}.png and two sheets: sheet_top_views.png (all takes) and
sheet_gates.png (both gates, every course take, from above). Gate frames = the frame where the headset comes
closest to the gate centre. Markers are drawn enlarged (x2.5).
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
from chrome_cdp import Chrome  # noqa: E402


def gate_frames(root, env_path):
    with open(os.path.join(env_path, "layout.json")) as f:
        lay = json.load(f)
    gates = []
    for g in lay["gates"]:
        c = se.scene_to_gltf(g["center_m"], lay["floor_z_m"])
        entry = se.scene_to_gltf(list(g["entry_xy"]) + [lay["floor_z_m"]], lay["floor_z_m"])
        exit_ = se.scene_to_gltf(list(g["exit_xy"]) + [lay["floor_z_m"]], lay["floor_z_m"])
        gates.append({"mode": g["mode"], "c": c, "dir": (exit_ - entry) / np.linalg.norm(exit_ - entry)})
    logs = mr.parse_editor_log(os.path.join(root, "Editor.log"))
    rows = []
    for p in mr.find_takes(root):
        take = mr.load_take(p)
        if take.log_name not in logs:
            continue
        al = mr.estimate_alignment(take, logs[take.log_name])
        head = np.stack([al.to_gltf(i, take.rb[mr.PICO_RB_ID]["pos"][i]) for i in range(len(take.frame))])
        r = []
        for g in gates:
            d = np.linalg.norm(head[:, [0, 2]] - g["c"][[0, 2]], axis=1)
            r.append((int(np.argmin(d)), float(d.min())))
        rows.append((take.name, r))
    return gates, rows


def set_cam(c, pos, tgt):
    c.eval(f"viewer.setView('free'); viewer.camera.up.set(0,1,0); viewer.camera.position.set({pos[0]},{pos[1]},{pos[2]}); "
           f"viewer.controls.target.set({tgt[0]},{tgt[1]},{tgt[2]}); viewer.controls.update(); viewer.render(); true")


def _font(size):
    from PIL import ImageFont
    try:
        import matplotlib
        return ImageFont.truetype(os.path.join(matplotlib.get_data_path(), "fonts", "ttf", "DejaVuSans.ttf"), size)
    except Exception:
        return ImageFont.load_default()


def sheet(paths, cols, out, crop=None, scale=0.5, labels=None, font_px=30):
    from PIL import Image, ImageDraw
    ims = [Image.open(p).convert("RGB") for p in paths]
    font = _font(font_px)
    if crop:
        ims = [im.crop(crop) for im in ims]
    w, h = ims[0].size
    rows = (len(ims) + cols - 1) // cols
    S = Image.new("RGB", (w * cols, h * rows), (21, 23, 26))
    for k, im in enumerate(ims):
        S.paste(im, ((k % cols) * w, (k // cols) * h))
        if labels:
            x, y = (k % cols) * w + 14, (k // cols) * h + 12
            dr = ImageDraw.Draw(S)
            box = dr.textbbox((x, y), labels[k], font=font)
            dr.rectangle((box[0] - 6, box[1] - 4, box[2] + 6, box[3] + 4), fill=(21, 23, 26))
            dr.text((x, y), labels[k], fill=(255, 255, 255), font=font)
    S = S.resize((int(S.width * scale), int(S.height * scale)), Image.LANCZOS)
    S.save(out)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--env", default=se.DEFAULT_ENV)
    ap.add_argument("--viewer", default=os.path.join(HERE, "data", "out", "viewer", "mocap_scene_viewer.html"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "stills"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    gates, rows = gate_frames(a.root, a.env)
    url = "file://" + os.path.abspath(a.viewer) + "?capture=1&show=big"
    tops, gate_shots, gate_labels = [], [], []
    with Chrome(1500, 1100) as c:
        c.open(url)
        c.wait_for("document.title === 'ready'", timeout=300)
        for k, (name, _) in enumerate(rows):
            c.eval(f"viewer.setTake({k}); viewer.setView('top'); viewer.setFrame(0); true")
            p = os.path.join(a.out, f"top_{k}.png")
            c.screenshot(p)
            tops.append(p)
        c.send("Emulation.setDeviceMetricsOverride", {"width": 1100, "height": 1100, "deviceScaleFactor": 1, "mobile": False})
        for g, G in enumerate(gates):
            for k, (name, r) in enumerate(rows):
                f, dist = r[g]
                if dist > 0.25:                     # this take does not pass the gate
                    continue
                c.eval(f"viewer.setTake({k}); viewer.setFrame({f}); true")
                set_cam(c, G["c"] + np.array([0.001, 2.25, 0]), G["c"])
                p = os.path.join(a.out, f"gate{g}_{k}_above.png")
                c.screenshot(p)
                gate_shots.append(p)
                gate_labels.append(f"{G['mode']} gate · {name[:13] + '…' + name[-4:] if len(name) > 20 else name} · t {f / 100:.2f} s")
                side = np.cross(G["dir"], [0, 1, 0])
                set_cam(c, G["c"] + 1.5 * side + np.array([0, 1.7, 0]), G["c"] + np.array([0, 0.6, 0]))
                c.screenshot(os.path.join(a.out, f"gate{g}_{k}_side.png"))
    sheet(tops, 3, os.path.join(a.out, "sheet_top_views.png"), crop=(160, 60, 1340, 1040), scale=0.55,
          labels=[n if len(n) <= 20 else n[:13] + "…" + n[-4:] for n, _ in rows], font_px=34)
    sheet(gate_shots, 5, os.path.join(a.out, "sheet_gates.png"), crop=(150, 150, 950, 950), scale=0.55, labels=gate_labels)
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
