"""
ocr_hausdorff_glyphs.py
=======================
Read morph-then-upright discs as binary stamps.

Not EasyOCR/Tesseract. Each connected letter is matched to A–Z / 0–9
templates with modified Hausdorff distance.

Run:  python ocr_hausdorff_glyphs.py
"""
import csv
import os
import re
import string

import cv2
import numpy as np
from scipy.spatial.distance import cdist

from align_binary_discs import ink_mask
from align_letters_above_digits import _row_blobs, split_two_rows
from ocr_letters_above import list_upright
from pick_final import canonical

SRC_DIR = "test1_letters_above"
TPL_SIZE = 48
LETTERS = string.ascii_uppercase
DIGITS = "0123456789"

GROUND_TRUTH = {
    "disc_01": "IPM 10",
    "disc_02": "TZP 110",
    "disc_03": "GM 10",
    "disc_04": "MEM 10",
    "disc_05": "AN 30",
    "disc_06": "CAZ 30",
    "disc_07": "SXT",
    "disc_08": "NN 10",
    "disc_09": "AM 10",
    "disc_10": "CRO 30",
    "disc_11": "CZ 30",
    "disc_12": "CIP 5",
}


def render_glyph(ch, font, scale, thick):
    canvas = np.full((80, 80), 255, np.uint8)
    (tw, th), _ = cv2.getTextSize(ch, font, scale, thick)
    x = (80 - tw) // 2
    y = (80 + th) // 2
    cv2.putText(canvas, ch, (x, y), font, scale, 0, thick, cv2.LINE_AA)
    ink = (canvas < 128).astype(np.uint8) * 255
    ink = cv2.dilate(ink, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    return pack_ink(ink)


def pack_ink(ink_u8):
    """Crop ink, pad, resize to TPL_SIZE. Return 0/1 array (1=ink)."""
    ys, xs = np.where(ink_u8 > 0)
    if len(xs) == 0:
        return np.zeros((TPL_SIZE, TPL_SIZE), np.uint8)
    y1, y2 = ys.min(), ys.max() + 1
    x1, x2 = xs.min(), xs.max() + 1
    crop = ink_u8[y1:y2, x1:x2]
    h, w = crop.shape
    side = max(h, w) + 4
    square = np.zeros((side, side), np.uint8)
    oy, ox = (side - h) // 2, (side - w) // 2
    square[oy:oy + h, ox:ox + w] = (crop > 0).astype(np.uint8)
    return cv2.resize(square, (TPL_SIZE, TPL_SIZE), interpolation=cv2.INTER_NEAREST)


def build_templates():
    fonts = [
        (cv2.FONT_HERSHEY_SIMPLEX, 1.8, 4),
        (cv2.FONT_HERSHEY_DUPLEX, 1.7, 4),
        (cv2.FONT_HERSHEY_COMPLEX, 1.6, 3),
    ]
    tpls = {ch: [] for ch in LETTERS + DIGITS}
    for ch in LETTERS + DIGITS:
        for font, scale, thick in fonts:
            tpls[ch].append(render_glyph(ch, font, scale, thick))
    return tpls


def ink_points(mask01):
    ys, xs = np.where(mask01 > 0)
    if len(xs) == 0:
        return np.zeros((1, 2), np.float64)
    pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
    if len(pts) > 400:
        idx = np.linspace(0, len(pts) - 1, 400).astype(int)
        pts = pts[idx]
    return pts


def modified_hausdorff(a, b):
    da = cdist(a, b).min(axis=1).mean()
    db = cdist(b, a).min(axis=1).mean()
    return 0.5 * (da + db)


def match_glyph(crop_bw, alphabet, templates):
    ink = (crop_bw < 128).astype(np.uint8) * 255
    q = pack_ink(ink)
    qp = ink_points(q)
    best_ch, best_d = "?", 1e9
    for ch in alphabet:
        for tpl in templates[ch]:
            d = modified_hausdorff(qp, ink_points(tpl))
            if d < best_d:
                best_d, best_ch = d, ch
    return best_ch, best_d


def split_wide_blob(gray, blob, median_w):
    """Cut a merged word blob into letter-width slices via vertical projection."""
    x, y, w, h = blob["x"], blob["y"], blob["w"], blob["h"]
    roi = gray[y:y + h, x:x + w]
    ink = (roi < 128).astype(np.uint8)
    n_est = max(2, int(round(w / max(median_w, 1))))
    if n_est < 2 or w < median_w * 1.55:
        return [blob]
    col = ink.sum(axis=0).astype(np.float64)
    # smooth
    k = max(3, w // 20)
    col = np.convolve(col, np.ones(k) / k, mode="same")
    # valleys between peaks — search interior
    cuts = []
    target = [int(w * (i + 1) / n_est) for i in range(n_est - 1)]
    for t in target:
        lo, hi = max(4, t - w // 8), min(w - 4, t + w // 8)
        if lo >= hi:
            continue
        cuts.append(int(lo + np.argmin(col[lo:hi])))
    cuts = sorted(set(cuts))
    if not cuts:
        return [blob]
    xs = [0] + cuts + [w]
    out = []
    for a, b in zip(xs, xs[1:]):
        if b - a < 6:
            continue
        sub = roi[:, a:b]
        ys, xs_ = np.where(sub < 128)
        if len(xs_) < 10:
            continue
        out.append({
            "x": x + a, "y": y + int(ys.min()),
            "w": b - a, "h": int(ys.max() - ys.min() + 1),
            "cx": x + a + (b - a) / 2.0,
            "cy": y + float(ys.mean()),
        })
    return out or [blob]


def glyphs_in_row(gray, row):
    ws = [b["w"] for b in row]
    median_w = float(np.median(ws)) if ws else 40.0
    parts = []
    for b in row:
        parts.extend(split_wide_blob(gray, b, median_w))
    return sorted(parts, key=lambda p: p["cx"])


def crop_blob(gray, b, pad=2):
    h, w = gray.shape
    x1 = max(b["x"] - pad, 0)
    y1 = max(b["y"] - pad, 0)
    x2 = min(b["x"] + b["w"] + pad, w)
    y2 = min(b["y"] + b["h"] + pad, h)
    return gray[y1:y2, x1:x2]


def looks_like_label(text):
    flat = re.sub(r"[^A-Z0-9]", "", (text or "").upper())
    return bool(re.fullmatch(r"[A-Z]{2,4}(?:\d{1,3})?", flat))


def read_disc(gray, templates):
    rows = split_two_rows(ink_mask(gray), min_area=40)
    if not rows:
        rows = [_row_blobs(ink_mask(gray), min_area=40)]
        if not rows[0]:
            return "", 1e9, 0
    texts = []
    dists = []
    for i, row in enumerate(rows):
        alphabet = DIGITS if (len(rows) >= 2 and i == len(rows) - 1) else LETTERS
        if len(rows) == 1:
            alphabet = LETTERS
        chars = []
        for b in glyphs_in_row(gray, row):
            ch, d = match_glyph(crop_blob(gray, b), alphabet, templates)
            chars.append(ch)
            dists.append(d)
        if chars:
            texts.append("".join(chars))
    if len(texts) >= 2:
        text = f"{texts[0]} {texts[-1]}"
    else:
        text = texts[0] if texts else ""
    mean_d = float(np.mean(dists)) if dists else 1e9
    return text, mean_d, len(dists)


def main():
    files = list_upright()
    if not files:
        raise SystemExit(f"No disc_XX_out_*.png in {SRC_DIR}/")
    print("Building glyph templates...")
    templates = build_templates()
    n_ok = 0
    rows = []
    print(f"Hausdorff OCR on {len(files)} morph+upright discs\n")
    for f in files:
        stem = re.match(r"(disc_\d+)_out_", f).group(1)
        path = os.path.join(SRC_DIR, f)
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        read, mean_d, n_g = read_disc(gray, templates)
        truth = GROUND_TRUTH[stem]
        ok = canonical(read) == canonical(truth)
        n_ok += int(ok)
        unsure = (not looks_like_label(read)) or mean_d > 6.5 or n_g < 2
        print(
            f"  {'OK  ' if ok else 'MISS'} {stem:8s}  "
            f"read '{read}'  gt '{truth}'  d={mean_d:.2f}  "
            f"{'-> gemma' if unsure else ''}"
        )
        rows.append({
            "stem": stem,
            "file": f,
            "path": path,
            "hausdorff": read,
            "mean_d": mean_d,
            "n_glyphs": n_g,
            "unsure": unsure,
            "ok": ok,
            "truth": truth,
        })
    print(f"\nAccuracy: {n_ok}/{len(files)}")
    os.makedirs("test1_ocr_hausdorff", exist_ok=True)
    out_csv = os.path.join("test1_ocr_hausdorff", "results.csv")
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"Wrote {out_csv}")
    return rows


if __name__ == "__main__":
    main()
