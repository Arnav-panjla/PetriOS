"""
Their plate pipeline, morphology off and aligner off.

  green-circle discs
  -> binary (crop, white outside, Otsu)
  -> rim strip
  -> Gemma 3 4B, their prompt and parse_label

No rotation. No morphological open. No color-photo reader.
"""
import csv
import glob
import os

import cv2

from align_binary_discs import strip_disc_rim
from detect_discs_v2 import annotate, crop_footer, detect_discs_v2
from extract_binary_discs import make_binary_disc
from ocr_gemma3 import MODEL, ask_gemma, models, ollama_ok, parse_label

SRC = "new_images_for_testing"
ROOT = "new_images_pipeline_no_align"
STAGES = ("01_binary", "02_rim_strip")


def main():
    if not ollama_ok():
        raise SystemExit("Ollama is not running.")
    names = models()
    if not any(MODEL in n or n.startswith(MODEL.split(":")[0]) for n in names):
        raise SystemExit(f"Need {MODEL}")

    os.makedirs(ROOT, exist_ok=True)
    rows = []
    csv_path = os.path.join(ROOT, "results.csv")
    fields = ["plate", "disc", "flip", "raw", "parsed", "crop"]

    def save():
        with open(csv_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)

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
            stripped = strip_disc_rim(binary)
            cv2.imwrite(os.path.join(work, "01_binary", name), binary)
            crop_path = os.path.join(work, "02_rim_strip", name)
            cv2.imwrite(crop_path, stripped)
            print(f"  {name}  Gemma...", flush=True)
            try:
                raw = ask_gemma(crop_path)
            except Exception as e:
                raw = f"[error] {type(e).__name__}: {e}"
            parsed = parse_label(raw, stripped)
            print(f"    raw={raw!r}  parsed={parsed!r}", flush=True)
            rows.append({
                "plate": stem,
                "disc": f"disc_{i:02d}",
                "flip": "off",
                "raw": raw,
                "parsed": parsed,
                "crop": crop_path,
            })
            save()
    print(f"\nWrote {csv_path}  ({len(rows)} discs)", flush=True)


if __name__ == "__main__":
    main()
