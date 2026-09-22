"""
ocr_pipeline_bakeoff.py
=======================
EasyOCR (not Gemma) on every image in test1_pipeline_bakeoff/.

Run:  python ocr_pipeline_bakeoff.py
"""
import csv
import os

import cv2
import easyocr

from compare_enhance_pipelines import OUT_DIR, PIPELINES
from ocr_letters_above import GROUND_TRUTH, clean_crop_for_ocr, ocr_best
from pick_final import canonical

CSV_PATH = os.path.join(OUT_DIR, "easyocr_results.csv")


def main():
    names = [n for n, _ in PIPELINES]
    print("Loading EasyOCR (not Gemma)...")
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)

    rows = []
    scores = {n: 0 for n in names}
    misses = {n: [] for n in names}

    for i in range(1, 13):
        stem = f"disc_{i:02d}"
        truth = GROUND_TRUTH[stem]
        for name in names:
            path = os.path.join(OUT_DIR, f"{stem}_{name}.png")
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print(f"missing {path}")
                continue
            crop = clean_crop_for_ocr(gray)
            raw, parsed, conf = ocr_best(crop, reader)
            ok = canonical(parsed) == canonical(truth)
            scores[name] += int(ok)
            if not ok:
                misses[name].append(f"{stem} got '{parsed}' gt '{truth}' raw='{raw}'")
            mark = "OK" if ok else "MISS"
            print(f"  {mark} {stem} {name:40s} '{parsed}'  gt '{truth}'", flush=True)
            rows.append({
                "stem": stem,
                "pipeline": name,
                "raw": raw,
                "parsed": parsed,
                "gt": truth,
                "ok": int(ok),
                "conf": f"{conf:.4f}",
            })

    print("\nEasyOCR accuracy")
    for name in names:
        print(f"  {name:40s}  {scores[name]}/12")
        for m in misses[name]:
            print(f"      {m}")

    with open(CSV_PATH, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nWrote {CSV_PATH}")


if __name__ == "__main__":
    main()
