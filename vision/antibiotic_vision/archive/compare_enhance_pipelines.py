"""
compare_enhance_pipelines.py
============================
Pure image processing. No OCR, no Gemma. Does not recompute method C.

P0 = shared prefix only (already done before this script):
  test1.png → Hough crop → Otsu → saved C rotate → letters-above → rim strip

P1–P4 = short tails after rim strip.
P5–P12 = 5 or 6 extra steps after rim strip.

Run:  python compare_enhance_pipelines.py
Out:  test1_pipeline_bakeoff/
"""
import os

import cv2

from align_binary_discs import list_discs, sheet, strip_disc_rim
from pipeline_c_letters_enhance import (
    box_mean,
    clahe,
    contrast_stretch,
    gaussian_smooth,
    highboost,
    laplacian_sharpen,
    median_smooth,
    morph_close,
    morph_erode,
    morph_open,
    otsu,
    unsharp_mask,
)

SRC_DIR = "test1_c_letters_above"
OUT_DIR = "test1_pipeline_bakeoff"


def chain(*fns):
    def run(gray):
        out = gray
        for fn in fns:
            out = fn(out)
        return out
    return run


PIPELINES = [
    ("P0_rim", chain()),
    ("P1_open", chain(morph_open)),
    ("P2_med_open", chain(median_smooth, morph_open)),
    ("P3_med_gauss_otsu", chain(median_smooth, gaussian_smooth, otsu)),
    ("P4_med_gauss_otsu_open", chain(median_smooth, gaussian_smooth, otsu, morph_open)),
    (
        "P5_med_gauss_unsharp_otsu_open",
        chain(median_smooth, gaussian_smooth, unsharp_mask, otsu, morph_open),
    ),
    (
        "P6_med_gauss_highboost_otsu_open",
        chain(median_smooth, gaussian_smooth, highboost, otsu, morph_open),
    ),
    (
        "P7_med_gauss_otsu_close_open_erode",
        chain(
            median_smooth,
            gaussian_smooth,
            otsu,
            morph_close,
            morph_open,
            morph_erode,
        ),
    ),
    (
        "P8_med_gauss_otsu_open_close",
        chain(median_smooth, gaussian_smooth, otsu, morph_open, morph_close),
    ),
    (
        "P9_med_gauss_lap_stretch_otsu_open",
        chain(
            median_smooth,
            gaussian_smooth,
            laplacian_sharpen,
            contrast_stretch,
            otsu,
            morph_open,
        ),
    ),
    (
        "P10_med_clahe_gauss_otsu_open",
        chain(median_smooth, clahe, gaussian_smooth, otsu, morph_open),
    ),
    (
        "P11_med_box_gauss_otsu_open_erode",
        chain(
            median_smooth,
            box_mean,
            gaussian_smooth,
            otsu,
            morph_open,
            morph_erode,
        ),
    ),
    (
        "P12_gauss_med_gauss_otsu_open_erode",
        chain(
            gaussian_smooth,
            median_smooth,
            gaussian_smooth,
            otsu,
            morph_open,
            morph_erode,
        ),
    ),
]


def main():
    files = list_discs()
    if not files:
        raise SystemExit("No discs — run extract_binary_discs.py")

    os.makedirs(OUT_DIR, exist_ok=True)
    sheets = {name: [] for name, _ in PIPELINES}

    for f in files:
        stem = os.path.splitext(f)[0]
        src = os.path.join(SRC_DIR, f)
        img = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"skip missing {src}")
            continue
        base = strip_disc_rim(img)
        for name, fn in PIPELINES:
            out = fn(base)
            cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_{name}.png"), out)
            sheets[name].append((stem, out))
        print(stem)

    for name, _ in PIPELINES:
        sheet(sheets[name], os.path.join(OUT_DIR, f"_{name}.png"))
        print(f"  {name}")

    old_csv = os.path.join(OUT_DIR, "results.csv")
    if os.path.isfile(old_csv):
        os.remove(old_csv)


if __name__ == "__main__":
    main()
