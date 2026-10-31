"""Picture-in-picture: the SMPL-X main video (third person, full environment) with the first-person 3DGS video
as an inset in the top-left corner of the follow view.

    python analysis/compose_pip.py [--main analysis/data/out/fit_videos] [--fpv analysis/data/out/fpv] \
        [--out analysis/data/out/pip] [--inset-width 640]

Pairs <main>/NN_<take>_smplx_main.mp4 with <fpv>/<take>_fpv3dgs.mp4. Both are rendered at the same frame rate
from frame 0 of the take, so frame k of one is frame k of the other.
"""
import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def compose(main, fpv, out, inset_w=640, margin=24, label="First person  |  3DGS"):
    inset_h = inset_w * 9 // 16
    f = (f"[1:v]scale={inset_w}:{inset_h}:flags=lanczos,"
         f"drawbox=x=0:y=0:w=iw:h=ih:color=white@0.85:t=3,"
         f"drawtext=fontfile={FONT}:text='{label}':x=12:y=10:fontsize=20:fontcolor=white:"
         f"box=1:boxcolor=black@0.55:boxborderw=6[pip];"
         f"[0:v][pip]overlay={margin}:{margin}:shortest=1[v]")
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", main, "-i", fpv, "-filter_complex", f, "-map", "[v]",
           "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out]
    subprocess.run(cmd, check=True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--main", default=os.path.join(HERE, "data", "out", "fit_videos"))
    ap.add_argument("--fpv", default=os.path.join(HERE, "data", "out", "fpv"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "pip"))
    ap.add_argument("--inset-width", type=int, default=640)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    for m in sorted(glob.glob(os.path.join(a.main, "*_smplx_main.mp4"))):
        name = re.sub(r"^\d+_", "", os.path.basename(m)).replace("_smplx_main.mp4", "")
        fp = os.path.join(a.fpv, f"{name}_fpv3dgs.mp4")
        if not os.path.exists(fp):
            print(f"{name}: no first-person video, skipped")
            continue
        out = os.path.join(a.out, os.path.basename(m).replace("_smplx_main.mp4", "_smplx_fpv3dgs.mp4"))
        compose(m, fp, out, a.inset_width)
        print("wrote", out, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
