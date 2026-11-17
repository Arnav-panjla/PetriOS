"""Rotate the camera image, clean it up, and mark QR codes.

Green box = decoded, red box = a QR code was seen but could not be read.

Every feature is a switch in `options`, changed at runtime from the GUI:

    detect          look for QR codes at all
    focus           live sharpness meter (Laplacian variance) for manual lens focusing
    denoise         denoise the image used for detection
    sharpen         unsharp mask on the image used for detection
    clahe           local contrast boost on the image used for detection
    show_filtered   show the filtered grey image instead of the original colour image
    wechat          use OpenCV's WeChat QR detector (needs opencv-contrib)
    robust          if nothing decodes, retry on sharpened / CLAHE / threshold / upscaled copies

Runs in its own thread on frames from the camera, so slow processing never
stalls the serial or CSI reader. Needs numpy and opencv.
"""

from __future__ import annotations

import collections
import logging
import threading
import time

logger = logging.getLogger(__name__)

GREEN = (0, 200, 0)
RED = (0, 0, 230)
KEEP_S = 8.0  # how long a seen code stays in the results list
PEAK_WINDOW_S = 30.0  # focus peak is the best score in this window

DEFAULT_OPTIONS = {
    "detect": True,
    "focus": False,
    "denoise": False,
    "sharpen": False,
    "clahe": False,
    "show_filtered": False,
    "wechat": False,
    "robust": False,
}


# (key, label, group, default, min, max, step) for every tunable filter value.
PARAM_SPEC = [
    ("denoise_h", "Denoise strength", "denoise", 7, 1, 30, 1),
    ("denoise_search", "Denoise search window", "denoise", 15, 5, 41, 2),
    ("sharpen_sigma", "Sharpen radius (sigma)", "sharpen", 2.0, 0.5, 6.0, 0.1),
    ("sharpen_amount", "Sharpen amount", "sharpen", 0.8, 0.1, 4.0, 0.1),
    ("clahe_clip", "CLAHE clip limit", "clahe", 2.5, 1.0, 10.0, 0.5),
    ("clahe_tile", "CLAHE tile grid", "clahe", 8, 2, 16, 1),
    ("focus_roi", "Focus area (% of frame)", "focus", 50, 20, 100, 5),
    ("robust_upscale", "Robust: max upscale", "robust", 3, 1, 4, 1),
    ("thr_block", "Robust: threshold block", "robust", 31, 11, 71, 2),
    ("thr_c", "Robust: threshold offset", "robust", 10, 0, 30, 1),
]
DEFAULT_PARAMS = {k: d for k, _l, _g, d, _lo, _hi, _st in PARAM_SPEC}


class QRView:
    """Same reading interface as SerialCamera (latest / wait_frame), annotated."""

    def __init__(self, camera, rotate: int = 90, detect: bool = True) -> None:
        import cv2
        import numpy as np

        self._cv2, self._np = cv2, np
        self.camera = camera
        self.rotate = rotate % 360
        self._options = dict(DEFAULT_OPTIONS)
        self._options["detect"] = detect
        self._params = dict(DEFAULT_PARAMS)
        self._detector = cv2.QRCodeDetector()
        self._clahe_cache: dict = {}
        self._wechat = None
        if hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
            try:
                self._wechat = cv2.wechat_qrcode_WeChatQRCode()
            except Exception as exc:  # models missing etc.
                logger.warning("WeChat QR detector unavailable: %s", exc)
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._seen: dict[str, dict] = {}
        self._focus_hist: collections.deque = collections.deque()
        # comparison stats
        self.frames = 0
        self.hits = 0  # frames with at least one decoded code
        self.seen_frames = 0  # frames with any code found, decoded or not
        self.proc_ms = 0.0
        self.detect_ms = 0.0
        self.focus = 0.0
        self.focus_peak = 0.0
        self.size = (0, 0)
        self.jpeg_kb = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    # ---- options -------------------------------------------------------
    @property
    def wechat_available(self) -> bool:
        return self._wechat is not None

    def options(self) -> dict:
        with self._cond:
            return dict(
                self._options,
                wechat_available=self.wechat_available,
                rotate=self.rotate,
                params=dict(self._params),
                param_spec=[
                    {"key": k, "label": l, "group": g, "default": d, "min": lo, "max": hi, "step": st}
                    for k, l, g, d, lo, hi, st in PARAM_SPEC
                ],
            )

    def set_rotate(self, degrees: int) -> None:
        """Clockwise rotation of the image, in steps of 90."""
        degrees = int(degrees)
        if degrees % 90 != 0:
            raise ValueError("rotation must be a multiple of 90")
        self.rotate = degrees % 360

    def set_param(self, key: str, value: float) -> None:
        """Tune one filter value. key '_reset' restores every default."""
        if key == "_reset":
            with self._cond:
                self._params = dict(DEFAULT_PARAMS)
            return
        spec = {k: (lo, hi, st) for k, _l, _g, _d, lo, hi, st in PARAM_SPEC}
        if key not in spec:
            raise ValueError(f"unknown filter value: {key}")
        lo, hi, _step = spec[key]
        value = min(hi, max(lo, float(value)))
        if key in ("denoise_search", "thr_block"):  # OpenCV needs odd window sizes
            value = int(value) | 1
        if isinstance(DEFAULT_PARAMS[key], int):
            value = int(round(value))
        with self._cond:
            self._params[key] = value

    def set_option(self, key: str, value: bool) -> None:
        if key not in DEFAULT_OPTIONS:
            raise ValueError(f"unknown image option: {key}")
        if key == "wechat" and value and not self.wechat_available:
            raise ValueError("WeChat detector needs opencv-contrib-python-headless")
        with self._cond:
            self._options[key] = bool(value)

    # ---- frame access --------------------------------------------------
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

    def stats(self) -> dict:
        return {
            "frames": self.frames,
            "decoded_pct": round(100.0 * self.hits / self.frames, 1) if self.frames else 0.0,
            "seen_pct": round(100.0 * self.seen_frames / self.frames, 1) if self.frames else 0.0,
            "proc_ms": round(self.proc_ms, 1),
            "detect_ms": round(self.detect_ms, 1),
            "focus": round(self.focus, 1),
            "focus_peak": round(self.focus_peak, 1),
            "width": self.size[0],
            "height": self.size[1],
            "jpeg_kb": round(self.jpeg_kb, 1),
        }

    # ---- worker --------------------------------------------------------
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
            first = self.frames <= 1
            self.proc_ms = ms if first else 0.9 * self.proc_ms + 0.1 * ms
            kb = len(jpeg) / 1024.0
            self.jpeg_kb = kb if first else 0.9 * self.jpeg_kb + 0.1 * kb
            with self._cond:
                self._jpeg = out
                self._seq += 1
                self._cond.notify_all()

    # ---- image helpers -------------------------------------------------
    @staticmethod
    def _focus_box(shape, pct: float):
        h, w = shape[:2]
        fh, fw = int(h * pct / 100.0), int(w * pct / 100.0)
        y0, x0 = (h - fh) // 2, (w - fw) // 2
        return x0, y0, x0 + fw, y0 + fh

    def _focus_score(self, gray, pct: float) -> float:
        """Variance of the Laplacian of the central area: higher means sharper."""
        x0, y0, x1, y1 = self._focus_box(gray.shape, pct)
        return float(self._cv2.Laplacian(gray[y0:y1, x0:x1], self._cv2.CV_64F).var())

    def _clahe_for(self, clip: float, tile: int):
        key = (clip, tile)
        if key not in self._clahe_cache:
            self._clahe_cache = {key: self._cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))}
        return self._clahe_cache[key]

    def _unsharp(self, gray, sigma: float, amount: float):
        blur = self._cv2.GaussianBlur(gray, (0, 0), sigma)
        return self._cv2.addWeighted(gray, 1.0 + amount, blur, -amount, 0)

    def _filtered(self, gray, opts, p):
        cv2 = self._cv2
        out = gray
        if opts["denoise"]:
            out = cv2.fastNlMeansDenoising(out, None, p["denoise_h"], 5, p["denoise_search"])
        if opts["sharpen"]:
            out = self._unsharp(out, p["sharpen_sigma"], p["sharpen_amount"])
        if opts["clahe"]:
            out = self._clahe_for(p["clahe_clip"], p["clahe_tile"]).apply(out)
        return out

    def _decode(self, gray, use_wechat: bool):
        """Return [(corner points Nx2 float, text)]. text is '' when not decodable."""
        np = self._np
        if use_wechat and self._wechat is not None:
            texts, points = self._wechat.detectAndDecode(gray)
            return [
                (np.asarray(quad, dtype=float).reshape(-1, 2), t)
                for t, quad in zip(texts, points)
            ]
        ok, texts, points, _ = self._detector.detectAndDecodeMulti(gray)
        if points is None or len(points) == 0:
            return []
        texts = list(texts) if texts is not None else []
        return [
            (np.asarray(quad, dtype=float).reshape(-1, 2), texts[i] if i < len(texts) else "")
            for i, quad in enumerate(points)
        ]

    def _outline_only(self, gray):
        """Find QR corners without decoding, for red boxes."""
        np = self._np
        ok, points = self._detector.detectMulti(gray)
        if not ok or points is None:
            return []
        return [(np.asarray(q, dtype=float).reshape(-1, 2), "") for q in points]

    def _robust_variants(self, gray, p):
        """Extra copies of the image to try when the plain pass finds nothing."""
        cv2 = self._cv2
        yield self._unsharp(gray, p["sharpen_sigma"], p["sharpen_amount"]), 1.0
        yield self._clahe_for(p["clahe_clip"], p["clahe_tile"]).apply(gray), 1.0
        yield cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, p["thr_block"], p["thr_c"]
        ), 1.0
        for scale in range(2, int(p["robust_upscale"]) + 1):
            big = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            yield self._unsharp(big, p["sharpen_sigma"] * scale, p["sharpen_amount"] * 0.6), float(scale)

    def _find(self, gray, opts, p):
        found = self._decode(gray, opts["wechat"])
        if any(t for _, t in found) or not opts["robust"]:
            return found
        for variant, scale in self._robust_variants(gray, p):
            tries = self._decode(variant, opts["wechat"])
            if any(t for _, t in tries):
                return [(q / scale, t) for q, t in tries]
        return found

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
        with self._cond:
            opts = dict(self._options)
            p = dict(self._params)
        needs_gray = opts["detect"] or opts["focus"]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if needs_gray else None

        if opts["focus"]:
            now = time.monotonic()
            self.focus = self._focus_score(gray, p["focus_roi"])
            self._focus_hist.append((now, self.focus))
            while self._focus_hist and now - self._focus_hist[0][0] > PEAK_WINDOW_S:
                self._focus_hist.popleft()
            self.focus_peak = max(v for _, v in self._focus_hist)

        shown = img
        if opts["detect"]:
            work = self._filtered(gray, opts, p)
            if opts["show_filtered"]:
                shown = cv2.cvtColor(work, cv2.COLOR_GRAY2BGR)
            t0 = time.monotonic()
            found = self._find(work, opts, p)
            if not found:
                found = self._outline_only(work)
            ms = (time.monotonic() - t0) * 1000.0
            self.detect_ms = ms if self.frames <= 1 else 0.9 * self.detect_ms + 0.1 * ms
            now = time.monotonic()
            self.seen_frames += bool(found)
            self.hits += any(t for _, t in found)
            thick = max(6, shown.shape[1] // 45)
            for quad, text in found:
                good = bool(text)
                colour = GREEN if good else RED
                poly = quad.astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(shown, [poly], True, colour, thick)
                x, y = int(quad[:, 0].min()), int(quad[:, 1].min())
                label = text if good else "QR (unreadable)"
                cv2.putText(shown, label[:40], (max(x, 2), max(y - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 2, cv2.LINE_AA)
                with self._cond:
                    key = text if good else "(unreadable QR)"
                    self._seen[key] = {"ok": good, "t": now}
        if opts["focus"]:
            self._draw_focus(shown, p["focus_roi"])
        ok, buf = cv2.imencode(".jpg", shown, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return buf.tobytes() if ok else jpeg

    def _draw_focus(self, img, pct: float) -> None:
        """Score text, a bar against the recent peak, and the area that is measured."""
        cv2 = self._cv2
        h, w = img.shape[:2]
        x0, y0, x1, y1 = self._focus_box(img.shape, pct)
        cv2.rectangle(img, (x0, y0), (x1, y1), (255, 200, 0), 1)
        frac = min(1.0, self.focus / self.focus_peak) if self.focus_peak > 0 else 0.0
        bar_w = int(w * 0.5)
        cv2.rectangle(img, (8, 8), (8 + bar_w, 24), (60, 60, 60), -1)
        colour = (0, 200, 0) if frac > 0.9 else (0, 200, 255) if frac > 0.6 else (0, 0, 230)
        cv2.rectangle(img, (8, 8), (8 + int(bar_w * frac), 24), colour, -1)
        cv2.putText(img, f"focus {self.focus:.0f} / peak {self.focus_peak:.0f}", (14, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
