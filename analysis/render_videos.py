"""MP4 per take from the viewer page: elevated follow view (left) and top view of the whole room (right).

    python analysis/render_videos.py [--viewer analysis/data/out/viewer/mocap_scene_viewer.html] \
        [--out analysis/data/out/videos] [--fps 25] [--takes 0 1 ...]

Drives headless Chrome (chrome_cdp.py) frame by frame and pipes JPEG screenshots into ffmpeg (H.264, yuv420p).
Real time: the 100 Hz takes are sampled every 100/fps frames. By default markers are drawn enlarged (x2.5,
--query show=big) and WebGL runs on SwiftShader; --angle vulkan uses the GPU (about 15x faster at 1080p).
SMPL-X videos: viewer built with build_viewer.py --fits, then e.g.
    --viewer .../viewer_body/mocap_scene_viewer.html --size 1920x1080 --fps 50 --query hide=cam --angle vulkan
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
    ap.add_argument("--query", default="show=big", help="extra viewer URL parameters, e.g. 'hide=cam'")
    ap.add_argument("--angle", default="swiftshader", help="Chrome WebGL backend: swiftshader (CPU), vulkan or gl-egl (GPU)")
    ap.add_argument("--suffix", default="", help="added to each video file name")
    ap.add_argument("--quality", type=int, default=90, help="JPEG quality of the captured frames")
    ap.add_argument("--crf", type=int, default=22, help="x264 quality (lower = better)")
    a = ap.parse_args(argv)
    w, h = (int(x) for x in a.size.split("x"))
    os.makedirs(a.out, exist_ok=True)
    url = "file://" + os.path.abspath(a.viewer) + "?capture=1&split=1&view=follow" + (f"&{a.query}" if a.query else "")
    with Chrome(w, h, angle=a.angle, timeout=120) as c:
        c.open(url)
        c.wait_for("document.title === 'ready'", timeout=300)
        n_takes = c.eval("document.getElementById('take').options.length")
        labels = c.eval("Array.from(document.getElementById('take').options).map(o => o.textContent)")
        for k in (a.takes if a.takes is not None else range(n_takes)):
            c.eval(f"viewer.setTake({k}); viewer.setView('follow'); true")
            F = c.eval("viewer.frames()")
            name = labels[k].split(" ")[0]
            path = os.path.join(a.out, f"{k:02d}_{name}{a.suffix}.mp4")
            ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "image2pipe", "-framerate", str(a.fps),
                                   "-c:v", "mjpeg", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(a.crf),
                                   "-preset", "medium", "-movflags", "+faststart", path], stdin=subprocess.PIPE)
            step = max(1, round(100 / a.fps))
            t0 = time.time()
            for i in range(0, F, step):
                c.eval(f"viewer.setFrame({i}); true")
                ff.stdin.write(c.screenshot(fmt="jpeg", quality=a.quality))
            ff.stdin.close()
            ff.wait()
            print(f"{path}: {len(range(0, F, step))} frames in {time.time() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
