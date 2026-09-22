"""
ocr_pipeline_gemma.py
=====================
Gemma 3 on every image in test1_pipeline_bakeoff/. Same parse/score as EasyOCR.

Run:  python ocr_pipeline_gemma.py
Out:  test1_pipeline_bakeoff/gemma_results.csv
"""
import csv
import os
import urllib.error

import cv2

from compare_enhance_pipelines import OUT_DIR, PIPELINES
from ocr_gemma3 import MODEL, ask_gemma, models, ollama_ok
from ocr_letters_above import GROUND_TRUTH, parse_disc_text
from pick_final import canonical

CSV_PATH = os.path.join(OUT_DIR, "gemma_results.csv")


def load_done():
    done = {}
    if not os.path.isfile(CSV_PATH):
        return done
    with open(CSV_PATH, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            done[(r["stem"], r["pipeline"])] = r
    return done


def write_all(rows):
    with open(CSV_PATH, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(
            fh,
            fieldnames=["stem", "pipeline", "raw", "parsed", "gt", "ok"],
        )
        w.writeheader()
        w.writerows(rows)


def main():
    if not ollama_ok():
        raise SystemExit("Ollama is not running.")
    names_avail = models()
    if not any(MODEL in n or n.startswith(MODEL.split(":")[0]) for n in names_avail):
        raise SystemExit(f"Pull {MODEL} first. Have: {names_avail}")

    pipes = [n for n, _ in PIPELINES]
    done = load_done()
    rows = list(done.values())
    print(f"Gemma {MODEL}  already {len(done)} / {12 * len(pipes)}\n")

    for i in range(1, 13):
        stem = f"disc_{i:02d}"
        truth = GROUND_TRUTH[stem]
        for name in pipes:
            key = (stem, name)
            if key in done:
                continue
            path = os.path.join(OUT_DIR, f"{stem}_{name}.png")
            gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print(f"missing {path}")
                continue
            print(f"  {stem} {name} ...", flush=True)
            try:
                raw = ask_gemma(path)
            except urllib.error.URLError as e:
                raw = f"[error] {e}"
            parsed = parse_disc_text(raw)
            ok = canonical(parsed) == canonical(truth)
            mark = "OK" if ok else "MISS"
            print(f"    {mark}  '{parsed}'  gt '{truth}'  raw={raw!r}")
            row = {
                "stem": stem,
                "pipeline": name,
                "raw": raw,
                "parsed": parsed,
                "gt": truth,
                "ok": str(int(ok)),
            }
            rows.append(row)
            done[key] = row
            write_all(rows)

    scores = {n: 0 for n in pipes}
    misses = {n: [] for n in pipes}
    for r in rows:
        n = r["pipeline"]
        if str(r["ok"]) in ("1", "True"):
            scores[n] += 1
        else:
            misses[n].append(
                f"{r['stem']} got '{r['parsed']}' gt '{r['gt']}' raw={r['raw']!r}"
            )

    print("\nGemma accuracy")
    for name in pipes:
        print(f"  {name:40s}  {scores[name]}/12")
        for m in misses[name]:
            print(f"      {m}")
    print(f"\nWrote {CSV_PATH}")


if __name__ == "__main__":
    main()
