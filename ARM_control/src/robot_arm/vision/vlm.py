"""Read antibiotic disk labels with a vision LLM served by Ollama on the GPU machine.

Opt-in: set OLLAMA_URL (e.g. http://127.0.0.1:11434 through an SSH tunnel).
Model defaults to qwen3.5:9b, override with VLM_MODEL.
"""

from __future__ import annotations

import base64
import json
import logging
import os
import re
import urllib.request

logger = logging.getLogger(__name__)

PROMPT = (
    "Read the text printed on the disc in the image.\n\n"
    "The marking is either:\n"
    "1. One line containing uppercase letters only.\n"
    "2. Two lines, with uppercase letters on the upper line and a number on the lower line.\n\n"
    "If there are two lines, combine them as:\n"
    "LETTERS NUMBER\n\n"
    "Return only the text printed on the disc.\n"
    "Do not identify the antibiotic.\n"
    "Do not explain your answer.\n"
    "Do not provide alternatives."
)

# Ignore http_proxy etc: the tunnel is on localhost and the lab proxy would break it.
_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def enabled() -> bool:
    return bool(os.environ.get("OLLAMA_URL"))


def ask(jpeg: bytes) -> str:
    url = os.environ["OLLAMA_URL"].rstrip("/") + "/api/chat"
    body = json.dumps({
        "model": os.environ.get("VLM_MODEL", "qwen3.5:9b"),
        "stream": False,
        "think": False,  # skip reasoning tokens, we want the label only
        "options": {"temperature": 0.0, "num_predict": 32},
        "messages": [{"role": "user", "content": PROMPT, "images": [base64.b64encode(jpeg).decode()]}],
    }).encode()
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
    with _opener.open(req, timeout=120) as resp:
        data = json.loads(resp.read().decode())
    return (data.get("message") or {}).get("content", "").strip()


def clean(raw: str) -> str:
    if len(raw.strip()) > 14:  # a sentence, not a label ("I cannot read ...")
        return "?"
    text = re.sub(r"[^A-Z0-9 ]", "", raw.upper()).strip()
    m = re.search(r"\b([A-Z]{2,4})\s?(\d{1,3})\b", text)
    if m:
        return f"{m.group(1)} {m.group(2)}"
    m = re.search(r"\b[A-Z]{2,4}\b", text)
    return m.group(0) if m and m.group(0) != "UNREADABLE" else "?"


def crop_disk(frame, x: float, y: float, r: float):
    """Square, upscaled crop of one disk (BGR array)."""
    import cv2

    half = int(r * 1.2)
    h, w = frame.shape[:2]
    x0, y0 = max(0, int(x) - half), max(0, int(y) - half)
    crop = frame[y0:min(h, int(y) + half), x0:min(w, int(x) + half)]
    scale = max(1, 320 // max(crop.shape[:2]))
    return cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)


def _jpeg(img) -> bytes:
    import cv2

    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise ValueError("could not encode disk crop")
    return buf.tobytes()


def _rotate(img, deg: float):
    import cv2

    if deg == 0:
        return img
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), deg, 1.0)
    return cv2.warpAffine(img, m, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)


def read_one(crop, tag: str = "") -> tuple[str, list[str]]:
    """The stamp can be at any angle, so ask at several rotations and vote.

    Returns (label, per-angle labels). Valid labels with a dose beat letters-only
    ones; ties go to the most frequent answer.
    """
    import cv2  # noqa: F401  (fail early if opencv is missing)

    n = int(os.environ.get("VLM_ANGLES", "8"))
    votes: dict[str, int] = {}
    per_angle = []
    for k in range(n):
        deg = 360.0 * k / n
        img = _rotate(crop, deg)
        if os.environ.get("VLM_SAVE_CROPS"):
            os.makedirs("/tmp/vlm_crops", exist_ok=True)
            cv2.imwrite(f"/tmp/vlm_crops/{tag}_rot{int(deg)}.jpg", img)
        label = clean(ask(_jpeg(img)))
        per_angle.append(label)
        if label != "?":
            votes[label] = votes.get(label, 0) + 1
    if not votes:
        return "?", per_angle
    best = max(votes, key=lambda lb: (votes[lb] + (1 if " " in lb else 0), votes[lb], len(lb)))
    return best, per_angle


def read_disks(frame, disks: list[tuple[float, float, float]]) -> list[str]:
    """One label per (x, y, r) disk in pixels. Prints to the terminal as it goes."""
    labels = []
    for i, (x, y, r) in enumerate(disks, 1):
        try:
            label, per_angle = read_one(crop_disk(frame, x, y, r), f"disk{i}")
            print(f"[vlm] disk {i} at ({x:.0f},{y:.0f}): {label}   (angles: {per_angle})", flush=True)
        except Exception as exc:
            label = "error"
            print(f"[vlm] disk {i} failed: {exc}", flush=True)
        labels.append(label)
    return labels
