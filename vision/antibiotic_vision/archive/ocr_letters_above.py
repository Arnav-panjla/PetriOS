"""
ocr_letters_above.py
====================
OCR the upright discs in test1_letters_above/ (after C + letters-above-digits).

Minimal prep only (Gemini suggestion):
  1) tight bounding crop around ink blobs
  2) 3x upscale
  3) EasyOCR
  4) regex / canonical parse for CODE + dose

Run:  python ocr_letters_above.py
Out:  test1_ocr_final/results.csv
"""
import csv
import os
import re

import cv2
import easyocr
import numpy as np

from align_binary_discs import sheet
from pick_final import canonical

SRC_DIR = "test1_letters_above"
OUT_DIR = "test1_ocr_final"
ALLOWLIST = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"

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


def clean_crop_for_ocr(gray):
    h, w = gray.shape
    ink = (gray < 128).astype(np.uint8) * 255
    contours, _ = cv2.findContours(ink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_area = h * w
    cx0, cy0 = w / 2.0, h / 2.0
    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (img_area * 0.001 < area < img_area * 0.20):
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        # skip long thin rim arcs near the edge
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
        dist = ((cx - cx0) ** 2 + (cy - cy0) ** 2) ** 0.5
        if aspect > 4.0 and dist > min(h, w) * 0.28:
            continue
        boxes.append((x, y, bw, bh))

    if not boxes:
        crop = gray
    else:
        padding = 6
        x1 = max(min(b[0] for b in boxes) - padding, 0)
        y1 = max(min(b[1] for b in boxes) - padding, 0)
        x2 = min(max(b[0] + b[2] for b in boxes) + padding, w)
        y2 = min(max(b[1] + b[3] for b in boxes) + padding, h)
        crop = gray[y1:y2, x1:x2]
        if crop.size == 0:
            crop = gray

    return cv2.resize(crop, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)


def parse_disc_text(raw_text):
    """
    Prefer CODE then optional dose. Use pick_final.canonical for O/0, I/1, etc.
    inside the dose part only (does not smash CRO → CR0).
    """
    flat = "".join(raw_text.split()).upper()
    if not flat:
        return ""

    # Prefer longest letters-prefix + digits that canonical accepts
    can = canonical(flat)
    if can:
        # pretty-print: insert space before dose digits
        m = re.fullmatch(r"([A-Z]+)(\d*)", can)
        if m:
            name, val = m.group(1), m.group(2)
            return f"{name} {val}".strip() if val else name

    m = re.search(r"([A-Z]{2,4})(\d{0,3})", flat)
    if m:
        name, val = m.group(1), m.group(2)
        return f"{name} {val}".strip() if val else name
    return flat


def list_upright():
    """One file per disc: newest disc_XX_out_0|180.png."""
    best = {}
    for f in os.listdir(SRC_DIR):
        m = re.fullmatch(r"(disc_\d+)_out_(0|180)\.png", f)
        if not m:
            continue
        stem = m.group(1)
        p = os.path.join(SRC_DIR, f)
        mt = os.path.getmtime(p)
        if stem not in best or mt > best[stem][0]:
            best[stem] = (mt, f)
    return [best[k][1] for k in sorted(best)]


def ocr_best(crop, reader):
    """Try normal + inverted; keep the reading that parses more like a label."""
    candidates = []
    for img in (crop, 255 - crop):
        hits = reader.readtext(img, detail=1, allowlist=ALLOWLIST)
        if not hits:
            continue
        raw = " ".join(h[1] for h in hits).upper()
        conf = float(sum(h[2] for h in hits))
        parsed = parse_disc_text(raw)
        flat = "".join(parsed.split())
        # prefer CODE+digits or letters-only code
        if re.fullmatch(r"[A-Z]{2,4}\d{1,3}", flat):
            prior = 3.0
        elif re.fullmatch(r"[A-Z]{2,4}", flat):
            prior = 2.0
        elif re.search(r"[A-Z]{2,}", flat) and re.search(r"\d", flat):
            prior = 1.5
        else:
            prior = 0.3
        candidates.append((prior * conf, raw, parsed, conf))
    if not candidates:
        return "", "", 0.0
    candidates.sort(reverse=True)
    _, raw, parsed, conf = candidates[0]
    return raw, parsed, conf


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    files = list_upright()
    if not files:
        raise SystemExit(
            f"No disc_XX_out_0/180.png in {SRC_DIR}/ — run align_letters_above_digits.py first"
        )

    print("Loading EasyOCR...")
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)

    rows = []
    crop_sheet = []
    n_ok = 0

    print(f"OCR {len(files)} upright discs from {SRC_DIR}/\n")
    for f in files:
        stem = re.match(r"(disc_\d+)_out_", f).group(1)
        path = os.path.join(SRC_DIR, f)
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        crop = clean_crop_for_ocr(gray)
        raw, parsed, conf = ocr_best(crop, reader)

        truth = GROUND_TRUTH.get(stem, "")
        ok = canonical(parsed) == canonical(truth) if truth else False
        n_ok += int(ok)
        flag = "OK" if ok else "MISS"
        print(f"[{stem}] source={f}  raw='{raw}'  parsed='{parsed}'  gt='{truth}'  [{flag}]")

        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_crop.png"), crop)
        crop_sheet.append((f"{stem} {parsed}", crop))
        rows.append({
            "image": stem,
            "source": f,
            "raw": raw,
            "parsed": parsed,
            "confidence": f"{conf:.4f}",
            "ground_truth": truth,
            "correct": int(ok),
        })

    sheet(crop_sheet, os.path.join(OUT_DIR, "_crops.png"), cell=200)
    csv_path = os.path.join(OUT_DIR, "results.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"\nAccuracy: {n_ok}/{len(files)}")
    print(f"  {OUT_DIR}/_crops.png")
    print(f"  {csv_path}")


if __name__ == "__main__":
    main()
