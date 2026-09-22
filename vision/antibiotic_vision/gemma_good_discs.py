"""
Gemma only on crops that still look like stamps after CV.
"""
import csv
import os
import shutil
import urllib.error

import cv2

from ocr_gemma3 import MODEL, ask_gemma, models, ollama_ok, parse_label
from pick_final import canonical

OUT = os.path.join("selected_plates", "good_only")

# Eye-picked: letters still readable as a stamp. No blanks, no fused blobs.
GOOD = [
    ("p01_five_discs", "disc_03", "E 15"),
    ("p01_five_discs", "disc_04", "VA 30"),
    ("p02_ctx_caz_cla", "disc_02", "CTX 30"),
    ("p02_ctx_caz_cla", "disc_03", "CAZ CLA"),
    ("p03_caz_ctx_cla_pair", "disc_04", "CTX 30"),
    ("p04_cfp_met_ipm_van", "disc_02", "MET 5"),
    ("p05_gen_rif_cot", "disc_04", "E 15"),
    ("p07_mem10_x4", "disc_02", "MEM 10"),
    ("p07_mem10_x4", "disc_03", "MEM 10"),
]


def main():
    if not ollama_ok():
        raise SystemExit("Ollama is not running.")
    names = models()
    if not any(MODEL in n or n.startswith(MODEL.split(":")[0]) for n in names):
        raise SystemExit(f"Need {MODEL}")

    os.makedirs(OUT, exist_ok=True)
    rows = []
    n_ok = 0
    print(f"Gemma on {len(GOOD)} good crops only (skipped {39 - len(GOOD)} junk)\n")
    for plate, disc, truth in GOOD:
        src = os.path.join("selected_plates", plate, "crops", f"{disc}.png")
        dst = os.path.join(OUT, f"{plate}_{disc}.png")
        shutil.copy2(src, dst)
        gray = cv2.imread(src, cv2.IMREAD_GRAYSCALE)
        print(f"  {plate} {disc}  gt '{truth}' ...", flush=True)
        try:
            raw = ask_gemma(src)
        except urllib.error.URLError as e:
            raw = f"[error] {e}"
        parsed = parse_label(raw, gray)
        ok = canonical(parsed) == canonical(truth)
        n_ok += int(ok)
        print(f"    {'OK' if ok else 'MISS'}  parsed='{parsed}'  raw={raw!r}")
        rows.append({
            "plate": plate,
            "disc": disc,
            "gt": truth,
            "raw": raw,
            "parsed": parsed,
            "ok": int(ok),
        })

    print(f"\nGemma on good discs: {n_ok}/{len(GOOD)}")
    path = os.path.join(OUT, "results.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
