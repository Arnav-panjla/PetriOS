"""Raspberry Pi CSI camera (OV5647 etc.) through rpicam-vid's MJPEG output.

No Python camera libraries needed: rpicam-vid writes JPEG frames to a pipe.
Same reading interface as SerialCamera (latest / wait_frame / fps / connected).
Only one program can use the CSI camera at a time.
"""

from __future__ import annotations

import logging
import shutil
import subprocess
import threading
import time

logger = logging.getLogger(__name__)

SOI, EOI = b"\xff\xd8", b"\xff\xd9"


class PiCamera:
    def __init__(self, width: int = 1280, height: int = 720, fps: int = 15) -> None:
        self.width, self.height, self.framerate = width, height, fps
        # Capture settings the GUI can change; apply() restarts rpicam-vid.
        self.sharpness = 1.0  # rpicam: 1.0 = normal
        self.contrast = 1.0
        self.denoise = "auto"  # auto, off, cdn_off, cdn_fast, cdn_hq
        self.zoom = 1.0  # digital zoom on the centre (rpicam --roi)
        self._restarting = False
        self.port = "CSI (rpicam-vid)"
        self.connected = False
        self.last_error: str | None = None
        self.bytes_rx = 0
        self.bad_frames = 0
        self.tail = b""
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._times: list[float] = []
        self._stop = False
        self._proc: subprocess.Popen | None = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True
        if self._proc is not None:
            self._proc.terminate()

    SIZES = {"640x480": (640, 480), "1280x720": (1280, 720), "1296x972": (1296, 972)}
    DENOISE = ("auto", "off", "cdn_off", "cdn_fast", "cdn_hq")

    def settings(self) -> dict:
        return {
            "size": f"{self.width}x{self.height}",
            "sizes": list(self.SIZES),
            "sharpness": self.sharpness,
            "contrast": self.contrast,
            "denoise": self.denoise,
            "denoise_modes": list(self.DENOISE),
            "zoom": self.zoom,
        }

    def apply(self, **kw) -> None:
        """Change capture settings and restart rpicam-vid with them."""
        if "size" in kw:
            if kw["size"] not in self.SIZES:
                raise ValueError(f"size must be one of {list(self.SIZES)}")
            self.width, self.height = self.SIZES[kw["size"]]
        if "sharpness" in kw:
            self.sharpness = min(16.0, max(0.0, float(kw["sharpness"])))
        if "contrast" in kw:
            self.contrast = min(16.0, max(0.0, float(kw["contrast"])))
        if "denoise" in kw:
            if kw["denoise"] not in self.DENOISE:
                raise ValueError(f"denoise must be one of {list(self.DENOISE)}")
            self.denoise = kw["denoise"]
        if "zoom" in kw:
            self.zoom = min(4.0, max(1.0, float(kw["zoom"])))
        self._restarting = True
        if self._proc is not None:
            self._proc.terminate()

    @property
    def fps(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._times if now - t < 3.0]
        return len(recent) / 3.0 if recent else 0.0

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def wait_frame(self, last_seq: int, timeout: float = 2.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq or self._stop, timeout)
            return self._jpeg, self._seq

    def _publish(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()
        now = time.monotonic()
        self._times = [t for t in self._times if now - t < 3.0] + [now]

    def _run(self) -> None:
        exe = shutil.which("rpicam-vid")
        while not self._stop:
            if exe is None:
                self.connected, self.last_error = False, "rpicam-vid not found"
                time.sleep(2.0)
                continue
            cmd = [
                exe, "-t", "0", "--nopreview", "--codec", "mjpeg", "--quality", "70",
                "--width", str(self.width), "--height", str(self.height),
                "--framerate", str(self.framerate), "--inline",
                "--sharpness", str(self.sharpness), "--contrast", str(self.contrast),
                "--denoise", self.denoise,
            ]
            if self.zoom > 1.0:
                inset = (1.0 - 1.0 / self.zoom) / 2.0
                cmd += ["--roi", f"{inset:.4f},{inset:.4f},{1.0 / self.zoom:.4f},{1.0 / self.zoom:.4f}"]
            cmd += ["-o", "-"]
            try:
                self._proc = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
                )
                self.connected, self.last_error = True, None
                self._read(self._proc)
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
            self.connected = False
            if self._restarting:
                self._restarting = False
                time.sleep(0.5)  # let the camera be released
                continue
            if not self._stop:
                self.last_error = self.last_error or "rpicam-vid exited (camera busy or not connected?)"
                time.sleep(2.0)

    def _read(self, proc: subprocess.Popen) -> None:
        buf = bytearray()
        while not self._stop:
            chunk = proc.stdout.read(65536)
            if not chunk:
                return
            self.bytes_rx += len(chunk)
            buf += chunk
            while True:
                start = buf.find(SOI)
                if start < 0:
                    buf.clear()
                    break
                end = buf.find(EOI, start + 2)
                if end < 0:
                    del buf[:start]
                    break
                self._publish(bytes(buf[start : end + 2]))
                del buf[: end + 2]
