"""USB webcam (V4L2, manual focus) with a focus meter and petri dish detection.

Same reading interface as the other cameras (latest / wait_frame / fps / connected).
Detection reuses PetriOS/vision/petri_detect.py (analyze) and only runs on capture().
"""

from __future__ import annotations

import functools
import importlib.util
import logging
import threading
import time
from pathlib import Path

logger = logging.getLogger(__name__)

DETECT_PATH = Path(__file__).resolve().parents[4] / "vision" / "petri_detect.py"


def find_device() -> str | None:
    """First V4L2 capture node that is not a Pi ISP/codec node."""
    root = Path("/sys/class/video4linux")
    if not root.is_dir():
        return None
    for node in sorted(root.glob("video*"), key=lambda p: int(p.name[5:])):
        try:
            name = (node / "name").read_text().strip().lower()
            index = (node / "index").read_text().strip()
        except OSError:
            continue
        if index == "0" and not any(s in name for s in ("pispbe", "rpi", "hevc", "codec")):
            return f"/dev/{node.name}"
    return None


@functools.lru_cache(maxsize=1)
def load_detector():
    spec = importlib.util.spec_from_file_location("petri_detect", DETECT_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {DETECT_PATH}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class WebCam:
    def __init__(self, device: str | None = None, rotate: int = 0, width: int = 1280, height: int = 720) -> None:
        self.device = device  # None = auto
        self.port = device or "auto"
        self.width, self.height = width, height
        self.rotate = rotate
        self.live_dish = False
        self.connected = False
        self.last_error: str | None = None
        self.focus = 0.0
        self.focus_peak = 0.0
        self.size = (0, 0)
        self.result: dict | None = None
        self._result_jpeg: bytes | None = None
        self._raw = None
        self._dish = None
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._times: list[float] = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    @property
    def fps(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._times if now - t < 3.0]
        return len(recent) / 3.0 if recent else 0.0

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def wait_frame(self, last_seq: int, timeout: float = 2.0) -> tuple[bytes | None, int]:
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq or self._stop, timeout)
            return self._jpeg, self._seq

    def result_jpeg(self) -> bytes | None:
        return self._result_jpeg

    def set_rotate(self, deg: int) -> None:
        if deg not in (0, 90, 180, 270):
            raise ValueError("rotate must be 0, 90, 180 or 270")
        self.rotate = deg

    def reset_peak(self) -> None:
        self.focus_peak = 0.0

    def capture(self) -> dict:
        """Run petri detection on the latest frame. Raises ValueError/RuntimeError."""
        import cv2

        frame = self._raw
        if frame is None:
            raise ValueError("webcam has no frame yet (not connected?)")
        det = load_detector()
        res = det.analyze(frame.copy())  # ValueError when no dish
        mm = res["mm_per_px"]
        zone_at = {(x, y): (r, clipped) for x, y, r, clipped in res["zones"]}
        disks = []
        for x, y, r in res["disks"]:
            z = zone_at.get((x, y))
            disks.append(
                {
                    "x": float(x), "y": float(y), "r_mm": float(r * mm),
                    "zone_mm": None if z is None else float(z[0] * mm),
                    "clipped": None if z is None else bool(z[1]),
                }
            )
        ok, buf = cv2.imencode(".jpg", res["annotated"], [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError("could not encode result")
        self._result_jpeg = buf.tobytes()
        self.result = {
            "time": time.strftime("%H:%M:%S"),
            "dish_r_mm": float(res["dish"][2] * mm),
            "mm_per_px": float(mm),
            "disks": disks,
            "focus": self.focus,
        }
        return self.result

    def _open(self):
        import cv2

        dev = self.device or find_device()
        if dev is None:
            self.last_error = "no USB webcam found (plug it in)"
            return None
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not cap.isOpened():
            self.last_error = f"cannot open {dev}"
            cap.release()
            return None
        self.port = dev
        return cap

    def _run(self) -> None:
        try:
            import cv2
        except ImportError:
            self.last_error = "opencv missing"
            return
        rot = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        n = 0
        while not self._stop:
            cap = self._open()
            if cap is None:
                self.connected = False
                time.sleep(2.0)
                continue
            self.connected, self.last_error = True, None
            while not self._stop:
                ok, frame = cap.read()
                if not ok:
                    self.last_error = "read failed (unplugged?)"
                    break
                if self.rotate:
                    frame = cv2.rotate(frame, rot[self.rotate])
                self._raw = frame
                self.size = (frame.shape[1], frame.shape[0])
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                h, w = gray.shape
                crop = gray[h // 4 : 3 * h // 4, w // 4 : 3 * w // 4]  # centre, where the dish is
                self.focus = float(cv2.Laplacian(crop, cv2.CV_64F).var())
                self.focus_peak = max(self.focus_peak, self.focus)
                view = frame
                if self.live_dish:
                    n += 1
                    if n % 3 == 1:
                        try:
                            self._dish = load_detector().detect_dish(gray, w)
                        except Exception:
                            self._dish = None
                    if self._dish is not None:
                        view = frame.copy()
                        d = self._dish
                        cv2.circle(view, (int(d[0]), int(d[1])), int(d[2]), (255, 0, 0), 3)
                ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with self._cond:
                        self._jpeg = buf.tobytes()
                        self._seq += 1
                        self._cond.notify_all()
                    now = time.monotonic()
                    self._times = [t for t in self._times if now - t < 3.0] + [now]
            cap.release()
            self.connected = False
            time.sleep(1.0)
