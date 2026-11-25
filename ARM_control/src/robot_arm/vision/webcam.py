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
        self.params: dict = {}  # petri_detect SPEC overrides set from the GUI
        self.manual_dish: list | None = None  # [x, y, r] px, replaces the auto dish
        self._cap = None  # frame the last result came from
        self._n = 0
        self._result_jpeg: bytes | None = None
        self._annotated = None
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

    def spec(self) -> list[dict]:
        try:
            det = load_detector()
        except Exception:  # opencv/numpy missing
            return []
        return [
            {"key": k, "label": v[4], "default": v[0], "min": v[1], "max": v[2], "step": v[3]}
            for k, v in det.SPEC.items()
        ]

    def capture(self) -> dict:
        """Grab the latest frame and run petri detection on it. Raises ValueError."""
        if self._raw is None:
            raise ValueError("webcam has no frame yet (not connected?)")
        self._cap = self._raw.copy()
        res = self.redetect()
        from robot_arm.vision import vlm

        if vlm.enabled() and res["disks"]:
            res["vlm"] = "reading"
            threading.Thread(target=self._read_labels, args=(self._cap, res), daemon=True).start()
        return res

    def _read_labels(self, frame, res: dict) -> None:
        """Ask the GPU VLM what each disk says; print to the terminal, show on the GUI image."""
        from robot_arm.vision import vlm

        mm = res["mm_per_px"]
        boxes = [(d["x"], d["y"], d["r_mm"] / mm) for d in res["disks"]]
        print(f"[vlm] capture #{res['n']}: reading {len(boxes)} disk(s)...", flush=True)
        try:
            labels = vlm.read_disks(frame, boxes)
        except Exception as exc:  # keep the GUI alive whatever happens
            print(f"[vlm] failed: {exc}", flush=True)
            res["vlm"] = None
            return
        for d, label in zip(res["disks"], labels):
            d["label"] = label
        print("[vlm] done: " + ", ".join(labels), flush=True)
        if self.result is not res:  # a newer capture replaced this one
            return
        self._draw_labels(res)
        res["vlm"] = "done"
        res["rev"] += 1

    def _draw_labels(self, res: dict) -> None:
        import cv2

        img = self._annotated.copy()
        for d in res["disks"]:
            label = d.get("label")
            if not label or label in ("?", "error"):
                label = label or ""
                if not label:
                    continue
            org = (max(0, int(d["x"]) - 40), max(14, int(d["y"] - d["r_mm"] / res["mm_per_px"]) - 22))
            cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            self._result_jpeg = buf.tobytes()

    def redetect(self) -> dict:
        """Run detection again on the captured frame with the current params."""
        import cv2

        if self._cap is None:
            return self.capture()
        det = load_detector()
        res = det.analyze(self._cap.copy(), self.params, self.manual_dish)  # ValueError: no dish
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
        self._annotated = res["annotated"]
        self._n += 1
        d = res["dish"]
        self.result = {
            "n": self._n,
            "time": time.strftime("%H:%M:%S"),
            "dish_r_mm": float(d[2] * mm),
            "dish": [float(d[0]), float(d[1]), float(d[2])],
            "frame": [int(self._cap.shape[1]), int(self._cap.shape[0])],
            "mm_per_px": float(mm),
            "disks": disks,
            "focus": self.focus,
            "vlm": None,  # None = off, "reading", "done"
            "rev": 0,  # bumps when the image/labels change, so the GUI reloads
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
                            self._dish = self.manual_dish or load_detector().find_dish(frame, self.params)
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
