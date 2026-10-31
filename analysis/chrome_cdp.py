"""Minimal Chrome DevTools Protocol client over --remote-debugging-pipe (standard library only).

Used to screenshot the viewer page and to capture video frames from it. Chrome reads commands on fd 3 and writes
replies and events on fd 4, each message a JSON object terminated by a NUL byte. WebGL runs on SwiftShader. Each
Chrome gets its own throwaway profile folder (analysis/data/.chrome-profiles/, git-ignored), never the user's.
"""
import base64
import json
import os
import select
import shlex
import shutil
import subprocess
import time


class CDPError(RuntimeError):
    pass


class Chrome:
    def __init__(self, width=1600, height=1000, chrome="google-chrome", timeout=60.0, profile=None, angle="swiftshader"):
        self._own_profile = profile is None
        self.profile = profile or os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", ".chrome-profiles",
                                               f"p{os.getpid()}-{id(self):x}")
        os.makedirs(self.profile, exist_ok=True)
        to_r, to_w = os.pipe()          # we write commands, Chrome reads them on fd 3
        from_r, from_w = os.pipe()      # Chrome writes on fd 4, we read
        gl = (["--enable-unsafe-swiftshader", "--use-angle=swiftshader"] if angle == "swiftshader" else
              [f"--use-angle={angle}", "--ignore-gpu-blocklist", "--enable-gpu"] + (["--enable-features=Vulkan"] if angle == "vulkan" else []))
        args = [chrome, "--headless=new", "--no-sandbox", "--remote-debugging-pipe", *gl,
                "--hide-scrollbars", "--mute-audio", "--no-first-run",
                "--disable-background-timer-throttling", "--disable-renderer-backgrounding",
                f"--window-size={width},{height}",
                f"--user-data-dir={self.profile}", "about:blank"]
        cmd = "exec " + " ".join(shlex.quote(a) for a in args) + f" 3<&{to_r} 4>&{from_w}"
        self.proc = subprocess.Popen(["bash", "-c", cmd], pass_fds=(to_r, from_w),
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        os.close(to_r)
        os.close(from_w)
        self.w, self.r = to_w, from_r
        self.buf = b""
        self.next_id = 0
        self.timeout = timeout
        self.width, self.height = width, height
        self.session = None

    # ------------------------------------------------------------------ transport
    def _read_msg(self, deadline):
        while b"\0" not in self.buf:
            left = deadline - time.time()
            if left <= 0:
                raise CDPError("timeout waiting for Chrome")
            ready, _, _ = select.select([self.r], [], [], left)
            if ready:
                chunk = os.read(self.r, 1 << 20)
                if not chunk:
                    raise CDPError("Chrome closed the pipe")
                self.buf += chunk
        msg, self.buf = self.buf.split(b"\0", 1)
        return json.loads(msg)

    def send(self, method, params=None, session=True, timeout=None):
        self.next_id += 1
        msg = {"id": self.next_id, "method": method, "params": params or {}}
        if session and self.session:
            msg["sessionId"] = self.session
        os.write(self.w, json.dumps(msg).encode() + b"\0")
        deadline = time.time() + (timeout or self.timeout)
        while True:
            m = self._read_msg(deadline)
            if m.get("id") == self.next_id:
                if "error" in m:
                    raise CDPError(f"{method}: {m['error']}")
                return m.get("result", {})

    # ------------------------------------------------------------------ page helpers
    def open(self, url):
        tid = self.send("Target.createTarget", {"url": "about:blank"}, session=False)["targetId"]
        self.session = self.send("Target.attachToTarget", {"targetId": tid, "flatten": True}, session=False)["sessionId"]
        self.send("Page.enable")
        self.send("Runtime.enable")
        self.send("Emulation.setDeviceMetricsOverride",
                  {"width": self.width, "height": self.height, "deviceScaleFactor": 1, "mobile": False})
        self.send("Page.navigate", {"url": url})

    def eval(self, expr, timeout=None):
        r = self.send("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True},
                      timeout=timeout)
        if "exceptionDetails" in r:
            raise CDPError(f"JS error: {r['exceptionDetails']}")
        return r.get("result", {}).get("value")

    def wait_for(self, expr, timeout=120.0, every=0.25):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                if self.eval(expr):
                    return True
            except CDPError:
                pass
            time.sleep(every)
        raise CDPError(f"timeout waiting for {expr!r}")

    def screenshot(self, path=None, fmt="png", quality=None):
        p = {"format": fmt, "captureBeyondViewport": False}
        if quality is not None:
            p["quality"] = quality
        data = base64.b64decode(self.send("Page.captureScreenshot", p)["data"])
        if path:
            with open(path, "wb") as f:
                f.write(data)
        return data

    def close(self):
        try:
            self.send("Browser.close", session=False, timeout=5)
        except Exception:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        for fd in (self.w, self.r):
            try:
                os.close(fd)
            except OSError:
                pass
        if self._own_profile:
            shutil.rmtree(self.profile, ignore_errors=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
