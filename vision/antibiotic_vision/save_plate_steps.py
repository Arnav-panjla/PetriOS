"""
Save every stage of the test1 chain for each photo in new_images_for_testing.

Circles come from the full-plate finder. Stages match the test1 folders:
binary, method C, open 5x5 x2, letters-above.
"""
import glob
import os

import cv2
import easyocr

from align_binary_discs import (
    apply_and_pick_flip,
    ink_mask,
    method_two_line_centroids,
)
from align_letters_above_digits import flip_to_letters_above
from detect_discs_v2 import annotate, crop_footer, detect_discs_v2
from extract_binary_discs import make_binary_disc
from morph_stroke_separate import stroke_separate

SRC = "new_images_for_testing"
ROOT = "new_images_steps"
STAGES = ("01_binary", "02_aligned", "03_morph", "04_letters_above")


def main():
    os.makedirs(ROOT, exist_ok=True)
    print("Loading EasyOCR...", flush=True)
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)
    for src in sorted(glob.glob(os.path.join(SRC, "*.png"))):
        stem = os.path.splitext(os.path.basename(src))[0]
        full = cv2.imread(src)
        img = crop_footer(full)
        discs = detect_discs_v2(full)
        work = os.path.join(ROOT, stem)
        for stage in STAGES:
            os.makedirs(os.path.join(work, stage), exist_ok=True)
        cv2.imwrite(os.path.join(work, "_annotated.png"), annotate(img, discs))
        print(f"\n=== {stem[:8]}  discs={len(discs)} ===", flush=True)
        for i, (x, y, r) in enumerate(discs, 1):
            name = f"disc_{i:02d}.png"
            _, _, binary = make_binary_disc(img, x, y, r)
            aligned, _ = apply_and_pick_flip(
                binary, method_two_line_centroids(ink_mask(binary))
            )
            opened = stroke_separate(aligned, "open", 5, 2)
            upright, ang, *_ = flip_to_letters_above(opened, reader)
            cv2.imwrite(os.path.join(work, "01_binary", name), binary)
            cv2.imwrite(os.path.join(work, "02_aligned", name), aligned)
            cv2.imwrite(os.path.join(work, "03_morph", name), opened)
            cv2.imwrite(os.path.join(work, "04_letters_above", name), upright)
            print(f"  {name}  flip {ang}", flush=True)
    print(f"\nWrote {ROOT}/", flush=True)


if __name__ == "__main__":
    main()
