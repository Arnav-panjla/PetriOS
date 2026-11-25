"""Read antibiotic disk labels with a vision LLM served by Ollama on the GPU machine.

Opt-in: set OLLAMA_URL (e.g. http://127.0.0.1:11434 through an SSH tunnel).
Model defaults to qwen3.5:9b, override with VLM_MODEL.
"""

from __future__ import annotations

import base64
import functools
import json
import logging
import os
import re
import sys
import urllib.request
from pathlib import Path

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


_PIPELINE_DIR = Path(__file__).resolve().parents[4] / "vision" / "antibiotic_vision"


@functools.lru_cache(maxsize=1)
def _pipeline():
    """Load extract_binary_discs and align_binary_discs from vision/antibiotic_vision."""
    if str(_PIPELINE_DIR) not in sys.path:
        sys.path.insert(0, str(_PIPELINE_DIR))
    import align_binary_discs
    import extract_binary_discs

    return extract_binary_discs, align_binary_discs


def prepare_disk(frame, x: float, y: float, r: float):
    """Binarise (Otsu) and align (method C) one disk. Returns an upright black-on-white image.

    Uses the disk position found by the GUI detector. Morphology stays off
    (it destroys letters on small discs, see README). Returns None on failure.
    """
    ext, aln = _pipeline()
    r = max(8, int(round(r)))
    _, _, binary = ext.make_binary_disc(frame, int(round(x)), int(round(y)), r)
    binary = aln.strip_disc_rim(binary)
    ang180 = aln.method_two_line_centroids(aln.ink_mask(binary))
    aligned, _ = aln.apply_and_pick_flip(binary, ang180)
    return aligned


def aligned_angles() -> list[float]:
    """Angles for an aligned crop: a full sweep every VLM_ALIGNED_STEP degrees (default 45).

    Set VLM_ALIGNED_STEP=30 for 12 angles. VLM_ALIGNED_ANGLES (comma list, e.g. "0,180")
    overrides the sweep.
    """
    raw = os.environ.get("VLM_ALIGNED_ANGLES")
    if raw:
        try:
            angles = [float(a) for a in raw.split(",") if a.strip()]
        except ValueError:
            angles = []
        if angles:
            return angles
    try:
        step = float(os.environ.get("VLM_ALIGNED_STEP", "45"))
    except ValueError:
        step = 45.0
    step = min(max(step, 5.0), 180.0)
    return [k * step for k in range(int(360 // step))]


def read_one(crop, tag: str = "", angles: list[float] | None = None) -> tuple[str, list[str]]:
    """Ask the VLM at each angle and vote.

    Without `angles` the stamp is assumed to be at any angle (VLM_ANGLES rotations).
    With an aligned crop, pass aligned_angles(): the flip choice in the aligner is a heuristic.
    Returns (label, per-angle labels). Valid labels with a dose beat letters-only
    ones; ties go to the most frequent answer.
    """
    import cv2  # noqa: F401  (fail early if opencv is missing)

    if angles is None:
        n = int(os.environ.get("VLM_ANGLES", "8"))
        angles = [360.0 * k / n for k in range(n)]
    votes: dict[str, int] = {}
    per_angle = []
    for deg in angles:
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
            aligned = None
            if os.environ.get("VLM_ALIGN", "1") != "0":
                try:
                    aligned = prepare_disk(frame, x, y, r)
                except Exception as exc:  # fall back to the raw colour crop
                    print(f"[vlm] disk {i} binarise/align failed ({exc}), using raw crop", flush=True)
            if aligned is not None:
                label, per_angle = read_one(aligned, f"disk{i}", angles=aligned_angles())
            else:
                label, per_angle = read_one(crop_disk(frame, x, y, r), f"disk{i}")
            print(f"[vlm] disk {i} at ({x:.0f},{y:.0f}): {label}   (angles: {per_angle})", flush=True)
        except Exception as exc:
            label = "error"
            print(f"[vlm] disk {i} failed: {exc}", flush=True)
        labels.append(label)
    return labels
