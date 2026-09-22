"""
run_stock_plates.py
===================
Winning path on selected unique stock plates (no exact duplicates):

  plate → Hough+Otsu → C → open 5x5 x2 → letters-above → rim strip → Gemma → parse_label

Does not touch test1_* folders.

Run:  python run_stock_plates.py
Out:  selected_plates/  (source copies + per-plate work + results.csv)
"""
import csv
import os
import shutil
import urllib.error

import cv2
import easyocr
import numpy as np

from align_binary_discs import (
    apply_and_pick_flip,
    ink_mask,
    method_two_line_centroids,
    sheet,
    strip_disc_rim,
)
from align_letters_above_digits import flip_to_letters_above
from extract_binary_discs import detect_discs, make_binary_disc
from morph_stroke_separate import stroke_separate
from ocr_gemma3 import MODEL, ask_gemma, models, ollama_ok, parse_label

ROOT = "selected_plates"

# Unique scenes only. Skip hash-dups (14=13, 16=4) and low-res copies of test4–9.
PLATES = [
    ("p01_five_discs", os.path.join("images", "test4.png")),
    ("p02_ctx_caz_cla", os.path.join("images", "test5.png")),
    ("p03_caz_ctx_cla_pair", os.path.join("images", "test7.png")),
    ("p04_cfp_met_ipm_van", os.path.join("images", "test8.png")),
    ("p05_gen_rif_cot", os.path.join("images", "test9.png")),
    ("p06_seven_discs", os.path.join("images", "test2.png")),
    ("p07_mem10_x4", os.path.join("images", "image (13).png")),
]


def crop_footer(img):
    """Drop the solid Shutterstock ID bar if it is a bright strip."""
    h, w = img.shape[:2]
    band = 28
    if h <= band + 40:
        return img
    strip = img[-band:]
    if float(np.mean(strip)) > 220:
        return img[:-band]
    return img


def keep_paper_discs(discs, gray):
    """Drop colony / rim false circles: similar radius, bright interior."""
    scored = []
    h, w = gray.shape
    for x, y, r in discs:
        inner = max(3, int(r * 0.5))
        roi = gray[max(0, y - inner): y + inner, max(0, x - inner): x + inner]
        if roi.size == 0:
            continue
        scored.append((x, y, r, float(np.mean(roi))))
    if not scored:
        return []
    scored.sort(key=lambda t: -t[3])
    bright = [t for t in scored if t[3] >= 145]
    use = bright or scored[:8]
    rs = np.array([t[2] for t in use], dtype=np.float64)
    med = float(np.median(rs))
    out = []
    for x, y, r, mu in use:
        if abs(r - med) > 0.35 * max(med, 1.0):
            continue
        out.append((x, y, r))
    out.sort(key=lambda c: (c[1], c[0]))
    return out[:16]


def main():
    if not ollama_ok():
        raise SystemExit("Ollama is not running.")
    names = models()
    if not any(MODEL in n or n.startswith(MODEL.split(":")[0]) for n in names):
        raise SystemExit(f"Need {MODEL}")

    os.makedirs(ROOT, exist_ok=True)
    print("Loading EasyOCR (0/180 only)...")
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)

    rows = []
    for stem, src in PLATES:
        if not os.path.isfile(src):
            print(f"missing {src}")
            continue
        dst = os.path.join(ROOT, f"{stem}.png")
        shutil.copy2(src, dst)
        img = cv2.imread(dst)
        img = crop_footer(img)
        work = os.path.join(ROOT, stem)
        os.makedirs(os.path.join(work, "crops"), exist_ok=True)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        discs = keep_paper_discs(detect_discs(img), gray)
        print(f"\n=== {stem}  {src}  discs={len(discs)} ===")
        annotated = img.copy()
        thumbs = []
        for i, (x, y, r) in enumerate(discs, 1):
            cv2.circle(annotated, (x, y), r, (0, 255, 0), 2)
            cv2.putText(
                annotated, str(i), (x - 8, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA,
            )
            _, _, binary = make_binary_disc(img, x, y, r)
            mask = ink_mask(binary)
            ang180 = method_two_line_centroids(mask)
            aligned, _ = apply_and_pick_flip(binary, ang180)
            opened = stroke_separate(aligned, "open", 5, 2)
            upright, ang, *_ = flip_to_letters_above(opened, reader)
            crop_path = os.path.join(work, "crops", f"disc_{i:02d}.png")
            stripped = strip_disc_rim(upright)
            cv2.imwrite(crop_path, stripped)
            thumbs.append((f"{i:02d}", stripped))
            print(f"  disc_{i:02d}  C+open+flip {ang}  Gemma...", flush=True)
            try:
                raw = ask_gemma(crop_path)
            except urllib.error.URLError as e:
                raw = f"[error] {e}"
            parsed = parse_label(raw, stripped)
            print(f"    raw={raw!r}  parsed='{parsed}'")
            rows.append({
                "plate": stem,
                "source": src,
                "disc": f"disc_{i:02d}",
                "flip": str(ang),
                "raw": raw,
                "parsed": parsed,
                "crop": crop_path,
            })
        cv2.imwrite(os.path.join(work, "_annotated.png"), annotated)
        if thumbs:
            sheet(thumbs, os.path.join(work, "_crops.png"), cols=4, cell=140)

    csv_path = os.path.join(ROOT, "results.csv")
    if rows:
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
    print(f"\nWrote {csv_path}  ({len(rows)} discs)")


if __name__ == "__main__":
    main()
