"""MP4 per take from the viewer page: elevated follow view (left) and top view of the whole room (right).

    python analysis/render_videos.py [--viewer analysis/data/out/viewer/mocap_scene_viewer.html] \
        [--out analysis/data/out/videos] [--fps 25] [--takes 0 1 ...]

Drives headless Chrome (chrome_cdp.py) frame by frame and pipes JPEG screenshots into ffmpeg (H.264, yuv420p).
Real time: the 100 Hz takes are sampled every 100/fps frames. Markers are drawn enlarged (x2.5) so they are
visible at this resolution.
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from chrome_cdp import Chrome  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--viewer", default=os.path.join(HERE, "data", "out", "viewer", "mocap_scene_viewer.html"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "videos"))
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--size", default="1600x800")
    ap.add_argument("--takes", type=int, nargs="*")
    a = ap.parse_args(argv)
    w, h = (int(x) for x in a.size.split("x"))
    os.makedirs(a.out, exist_ok=True)
    url = "file://" + os.path.abspath(a.viewer) + "?capture=1&split=1&show=big&view=follow"
    with Chrome(w, h) as c:
        c.open(url)
        c.wait_for("document.title === 'ready'", timeout=300)
        n_takes = c.eval("document.getElementById('take').options.length")
        labels = c.eval("Array.from(document.getElementById('take').options).map(o => o.textContent)")
        for k in (a.takes if a.takes is not None else range(n_takes)):
            c.eval(f"viewer.setTake({k}); viewer.setView('follow'); true")
            F = c.eval("viewer.frames()")
            name = labels[k].split(" ")[0]
            path = os.path.join(a.out, f"{k:02d}_{name}.mp4")
            ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(a.fps),
                                   "-c:v", "mjpeg", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22",
                                   "-preset", "medium", "-movflags", "+faststart", path], stdin=subprocess.PIPE)
            step = max(1, round(100 / a.fps))
            t0 = time.time()
            for i in range(0, F, step):
                c.eval(f"viewer.setFrame({i}); true")
                ff.stdin.write(c.screenshot(fmt="jpeg", quality=90))
            ff.stdin.close()
            ff.wait()
            print(f"{path}: {len(range(0, F, step))} frames in {time.time() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
