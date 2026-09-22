"""
morph_stroke_separate.py
========================
After method-C alignment: light morphological erosion / opening to thin
blobby strokes and separate touching characters.

No OCR — visual compare only.

Variants (black ink on white; morphology on ink=255 mask):
  01_erode_5x5_x2
  02_erode_7x7_x2
  03_erode_9x9_x3
  04_open_5x5_x2
  05_open_7x7_x2
  06_open_9x9_x2

Run:  python morph_stroke_separate.py
Out:  test1_morph/
"""
import os
import re

import cv2
import numpy as np

from align_binary_discs import (
    SRC_DIR,
    apply_and_pick_flip,
    ink_mask,
    list_discs,
    method_two_line_centroids,
    sheet,
)

OUT_DIR = "test1_morph"

# (name, op, kernel_size, iterations)
# On ~400px discs with thick strokes, 2x2/3x3 x1 is nearly invisible.
VARIANTS = [
    ("01_erode_5x5_x2", "erode", 5, 2),
    ("02_erode_7x7_x2", "erode", 7, 2),
    ("03_erode_9x9_x3", "erode", 9, 3),
    ("04_open_5x5_x2", "open", 5, 2),
    ("05_open_7x7_x2", "open", 7, 2),
    ("06_open_9x9_x2", "open", 9, 2),
]


def kernel(size):
    # rectangular kernel as Gemini suggested (2x2 / 3x3)
    return cv2.getStructuringElement(cv2.MORPH_RECT, (size, size))


def stroke_separate(gray_bw, op, ksize, iterations=1):
    """
    gray_bw: black letters on white.
    Work on ink mask (255=letter), then map back to black-on-white.
    """
    ink = ink_mask(gray_bw)
    ker = kernel(ksize)
    if op == "erode":
        out_ink = cv2.erode(ink, ker, iterations=iterations)
    elif op == "open":
        out_ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, ker, iterations=iterations)
    else:
        raise ValueError(op)
    out = np.full_like(gray_bw, 255)
    out[out_ink > 0] = 0
    return out


def load_or_make_c_aligned(path, stem):
    """Prefer newest C image in test1_aligned/; else compute C from binary."""
    aligned_dir = "test1_aligned"
    best = None
    best_mtime = -1.0
    if os.path.isdir(aligned_dir):
        for f in os.listdir(aligned_dir):
            if f.startswith(f"{stem}_C_two_line_") and f.endswith(".png"):
                p = os.path.join(aligned_dir, f)
                m = os.path.getmtime(p)
                if m > best_mtime:
                    best_mtime, best = m, p
    if best is not None:
        img = cv2.imread(best, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            return img, os.path.basename(best)

    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    mask = ink_mask(img)
    ang180 = method_two_line_centroids(mask)
    aligned, ang = apply_and_pick_flip(img, ang180)
    return aligned, f"{stem}_C_{ang:.1f} (computed)"


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    files = list_discs()
    if not files:
        raise SystemExit(f"No disc_XX.png in {SRC_DIR}")

    sheets = {"00_C_aligned": []}
    for vname, _, _, _ in VARIANTS:
        sheets[vname] = []

    print(f"Morph stroke-separate on {len(files)} C-aligned discs\n")
    for f in files:
        stem = os.path.splitext(f)[0]
        path = os.path.join(SRC_DIR, f)
        aligned, src_name = load_or_make_c_aligned(path, stem)
        print(f"[{stem}] source={src_name}")

        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_00_C.png"), aligned)
        sheets["00_C_aligned"].append((stem, aligned))

        for vname, op, ksize, iters in VARIANTS:
            out = stroke_separate(aligned, op, ksize, iters)
            out_path = os.path.join(OUT_DIR, f"{stem}_{vname}.png")
            cv2.imwrite(out_path, out)
            sheets[vname].append((stem, out))
            print(f"  -> {vname}")

    sheet(sheets["00_C_aligned"], os.path.join(OUT_DIR, "_00_C_aligned.png"))
    for vname, _, _, _ in VARIANTS:
        sheet(sheets[vname], os.path.join(OUT_DIR, f"_{vname}.png"))

    print(f"\nDone. Compare contact sheets in {OUT_DIR}/")
    print("  _00_C_aligned.png")
    for vname, _, _, _ in VARIANTS:
        print(f"  _{vname}.png")


if __name__ == "__main__":
    main()
