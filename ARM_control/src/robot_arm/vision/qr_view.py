"""Rotate the camera image and mark QR codes: green box = decoded, red = seen but unreadable.

Runs in its own thread on frames from SerialCamera, so slow detection never
stalls the serial reader. Needs numpy and opencv (opencv-python-headless).
"""

from __future__ import annotations

import logging
import threading
import time

logger = logging.getLogger(__name__)

GREEN = (0, 200, 0)
RED = (0, 0, 230)
KEEP_S = 8.0  # how long a seen code stays in the results list


class QRView:
    """Same reading interface as SerialCamera (latest / wait_frame), annotated."""

    def __init__(self, camera, rotate: int = 90, detect: bool = True) -> None:
        import cv2
        import numpy as np

        self._cv2, self._np = cv2, np
        self.camera = camera
        self.rotate = rotate % 360
        self.detect = detect
        self._detector = cv2.QRCodeDetector()
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._seen: dict[str, dict] = {}
        # comparison stats
        self.frames = 0
        self.hits = 0  # frames with at least one decoded code
        self.seen_frames = 0  # frames with any code found, decoded or not
        self.proc_ms = 0.0
        self.size = (0, 0)
        self.jpeg_kb = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def wait_frame(self, last_seq: int, timeout: float = 2.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq or self._stop, timeout)
            return self._jpeg, self._seq

    def results(self) -> list[dict]:
        """Codes seen in the last few seconds: text, whether it decoded, age in s."""
        now = time.monotonic()
        with self._cond:
            self._seen = {k: v for k, v in self._seen.items() if now - v["t"] < KEEP_S}
            return [
                {"text": k, "ok": v["ok"], "age": round(now - v["t"], 1)}
                for k, v in sorted(self._seen.items(), key=lambda kv: -kv[1]["t"])
            ]

    def _run(self) -> None:
        seq = -1
        while not self._stop:
            jpeg, seq = self.camera.wait_frame(seq, timeout=2.0)
            if jpeg is None:
                continue
            t0 = time.monotonic()
            try:
                out = self._process(jpeg)
            except Exception as exc:  # never let one bad frame kill the thread
                logger.warning("frame processing failed: %s", exc)
                continue
            ms = (time.monotonic() - t0) * 1000.0
            self.proc_ms = ms if self.frames <= 1 else 0.9 * self.proc_ms + 0.1 * ms
            self.jpeg_kb = len(jpeg) / 1024.0 if self.frames <= 1 else 0.9 * self.jpeg_kb + 0.1 * len(jpeg) / 1024.0
            with self._cond:
                self._jpeg = out
                self._seq += 1
                self._cond.notify_all()

    def _find(self, img):
        """Return [(corner points Nx2 float, text)]."""
        cv2, np = self._cv2, self._np
        for scale in (1.0, 2.0):
            view = img if scale == 1.0 else cv2.resize(
                img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC
            )
            ok, texts, points, _ = self._detector.detectAndDecodeMulti(view)
            if points is None or len(points) == 0:
                continue
            texts = list(texts) if texts is not None else []
            found = []
            for i, quad in enumerate(points):
                text = texts[i] if i < len(texts) else ""
                found.append((np.asarray(quad, dtype=float).reshape(-1, 2) / scale, text))
            return found
        return []

    def _process(self, jpeg: bytes) -> bytes:
        cv2, np = self._cv2, self._np
        img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return jpeg
        turns = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
        if self.rotate in turns:
            img = cv2.rotate(img, turns[self.rotate])
        self.size = (img.shape[1], img.shape[0])
        self.frames += 1
        if self.detect:
            now = time.monotonic()
            found = self._find(img)
            self.seen_frames += bool(found)
            self.hits += any(t for _, t in found)
            thick = max(6, img.shape[1] // 45)
            for quad, text in found:
                good = bool(text)
                colour = GREEN if good else RED
                poly = quad.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(img, [poly], True, colour, thick)
                x, y = int(quad[:, 0].min()), int(quad[:, 1].min())
                label = text if good else "QR (unreadable)"
                cv2.putText(img, label[:40], (max(x, 2), max(y - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2, cv2.LINE_AA)
                with self._cond:
                    key = text if good else "(unreadable QR)"
                    self._seen[key] = {"ok": good, "t": now}
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else jpeg

    def stats(self) -> dict:
        return {
            "frames": self.frames,
            "decoded_pct": round(100.0 * self.hits / self.frames, 1) if self.frames else 0.0,
            "seen_pct": round(100.0 * self.seen_frames / self.frames, 1) if self.frames else 0.0,
            "proc_ms": round(self.proc_ms, 1),
            "width": self.size[0],
            "height": self.size[1],
            "jpeg_kb": round(self.jpeg_kb, 1),
        }
