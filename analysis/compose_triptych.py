"""Three equal panels side by side: third person (SMPL-X, follow view) | top view | first person (3DGS).

    python analysis/compose_triptych.py [--follow-top analysis/data/out/follow_top] [--fpv analysis/data/out/fpv] \
        [--out analysis/data/out/deliver]

Inputs: <follow_top>/NN_<take>_follow_top.mp4 (2560x720: follow view left half, top view right half; render_videos.py
--size 2560x720 --query "splitw=0.5 ...") and <fpv>/<take>_fpv3dgs.mp4 (1280x720), same frame rate, both from frame 0.
Writes, per take, into <out>/:
- triptych/NN_<take>_triptych.mp4      3840x720, a label on each panel;
- third_person/NN_<take>_third_person.mp4, top_view/NN_<take>_top_view.mp4, first_person/NN_<take>_first_person_3dgs.mp4
  (1280x720 each, no labels).
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
ENC = ["-c:v", "libx264", "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart"]


def label(text):
    return (f"drawtext=fontfile={FONT}:text='{text}':x=14:y=12:fontsize=24:fontcolor=white:"
            f"box=1:boxcolor=black@0.55:boxborderw=7")


def run(cmd):
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error"] + cmd, check=True)


def one(job, out):
    idx_name, ft, fp = job
    tri = os.path.join(out, "triptych", f"{idx_name}_triptych.mp4")
    f = (f"[0:v]split=2[x][y];[x]crop=1280:720:0:0,{label('Third person  |  SMPL-X fit')}[a];"
         f"[y]crop=1280:720:1280:0,{label('Top view')}[b];"
         f"[1:v]scale=1280:720,{label('First person  |  3DGS')}[c];[a][b][c]hstack=inputs=3[v]")
    run(["-i", ft, "-i", fp, "-filter_complex", f, "-map", "[v]", "-shortest"] + ENC + [tri])
    run(["-i", ft, "-vf", "crop=1280:720:0:0"] + ENC + [os.path.join(out, "third_person", f"{idx_name}_third_person.mp4")])
    run(["-i", ft, "-vf", "crop=1280:720:1280:0"] + ENC + [os.path.join(out, "top_view", f"{idx_name}_top_view.mp4")])
    shutil.copyfile(fp, os.path.join(out, "first_person", f"{idx_name}_first_person_3dgs.mp4"))
    return tri


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--follow-top", default=os.path.join(HERE, "data", "out", "follow_top"))
    ap.add_argument("--fpv", default=os.path.join(HERE, "data", "out", "fpv"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "deliver"))
    a = ap.parse_args(argv)
    for sub in ("triptych", "third_person", "top_view", "first_person"):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    jobs = []
    for ft in sorted(glob.glob(os.path.join(a.follow_top, "*_follow_top.mp4"))):
        idx_name = os.path.basename(ft).replace("_follow_top.mp4", "")
        name = re.sub(r"^\d+_", "", idx_name)
        fp = os.path.join(a.fpv, f"{name}_fpv3dgs.mp4")
        if os.path.exists(fp):
            jobs.append((idx_name, ft, fp))
        else:
            print(f"{name}: no first-person video, skipped")
    with ThreadPoolExecutor(max_workers=max(1, len(jobs))) as ex:
        for tri in ex.map(lambda j: one(j, a.out), jobs):
            print("wrote", tri, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
