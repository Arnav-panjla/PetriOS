"""
ocr_gemma3.py
=============
Last step after Hausdorff glyph OCR.

Sends morph+upright disc crops to local Gemma 3 (Ollama `gemma3:4b`).
Default: every disc (Hausdorff is a weak prior on these stamps).
Pass --unsure-only to skip discs Hausdorff already formatted cleanly.

No antibiotic-name dictionary. Layout hint only (letters above digits).

Requires: Ollama running, `ollama pull gemma3:4b`

Run:  python ocr_gemma3.py
      python ocr_gemma3.py --unsure-only
"""
import argparse
import base64
import csv
import json
import os
import re
import sys
import urllib.error
import urllib.request

import cv2
import numpy as np

from align_binary_discs import ink_mask, strip_disc_rim
from align_letters_above_digits import split_two_rows
from pick_final import canonical

OLLAMA = "http://127.0.0.1:11434"
MODEL = os.environ.get("GEMMA3_MODEL", "gemma3:4b")
HAUS_CSV = os.path.join("test1_ocr_hausdorff", "results.csv")
OUT_DIR = "test1_ocr_gemma3"
PROMPT = (
    "This is a high-contrast photo of stamped text on a round antibiotic disc. "
    "The text is black on white. When upright, letters are on the TOP row and "
    "digits (the dose) on the BOTTOM row. Some discs have letters only, no dose. "
    "Read the characters exactly as printed. "
    "Reply with ONLY the label: 2 to 4 uppercase letters, then a space and the "
    "dose digits if a dose is present. No other words, no punctuation."
)

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


def should_drop_dose(gray):
    """True when the second 'row' is rim junk, not a real dose line."""
    rows = split_two_rows(ink_mask(gray), min_area=40)
    if len(rows) != 2:
        return False
    hs = [float(np.median([b["h"] for b in r])) for r in rows]
    return min(hs) < 0.45 * max(hs)


def parse_label(raw, gray=None):
    if not raw:
        return ""
    text = raw.upper()
    text = text.replace("µG", "").replace("UG", "")
    flat = re.sub(r"[^A-Z0-9]", "", text)
    drop_dose = bool(gray is not None and should_drop_dose(gray))
    if drop_dose:
        m = re.search(r"[A-Z]{2,4}", flat)
        return m.group(0) if m else ""
    m = re.search(r"[A-Z]{2,4}\d{1,3}", flat)
    if m:
        s = m.group(0)
        i = next(k for k, ch in enumerate(s) if ch.isdigit())
        return f"{s[:i]} {s[i:]}"
    m = re.search(r"[A-Z]{2,4}", flat)
    return m.group(0) if m else ""


def ollama_ok():
    try:
        urllib.request.urlopen(OLLAMA + "/api/tags", timeout=3)
        return True
    except Exception:
        return False


def models():
    try:
        with urllib.request.urlopen(OLLAMA + "/api/tags", timeout=5) as r:
            data = json.loads(r.read().decode())
        return [m.get("name", "") for m in data.get("models", [])]
    except Exception:
        return []


def prep_for_gemma(src_path, dst_path, side=None):
    """Upright morph disc with rim arcs removed; native resolution (no 896 zoom)."""
    gray = cv2.imread(src_path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        return src_path
    gray = strip_disc_rim(gray)
    parent = os.path.dirname(dst_path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    cv2.imwrite(dst_path, gray)
    return dst_path


def ask_gemma(image_path):
    with open(image_path, "rb") as fh:
        b64 = base64.b64encode(fh.read()).decode("ascii")
    body = json.dumps({
        "model": MODEL,
        "stream": False,
        "options": {"temperature": 0.0, "num_predict": 32},
        "messages": [{
            "role": "user",
            "content": PROMPT,
            "images": [b64],
        }],
    }).encode()
    req = urllib.request.Request(
        OLLAMA + "/api/chat",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=600) as resp:
        data = json.loads(resp.read().decode())
    return (data.get("message") or {}).get("content", "").strip()


def load_hausdorff_rows():
    if not os.path.isfile(HAUS_CSV):
        raise SystemExit(f"Missing {HAUS_CSV} — run ocr_hausdorff_glyphs.py first")
    with open(HAUS_CSV, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        r["unsure"] = str(r.get("unsure", "")).lower() in ("1", "true", "yes")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--unsure-only",
        action="store_true",
        help="Gemma only on discs Hausdorff marked unsure",
    )
    args = ap.parse_args()

    if not ollama_ok():
        print(
            "Ollama is not running. Install from https://ollama.com , then:\n"
            "  ollama pull gemma3:4b\n"
            "  ollama serve\n"
            "Then re-run this script. Hausdorff results are left unchanged.",
            file=sys.stderr,
        )
        raise SystemExit(2)

    names = models()
    if not any(MODEL in n or n.startswith(MODEL.split(":")[0]) for n in names):
        print(f"Model {MODEL} not pulled yet. Names: {names}")
        print(f"Run: ollama pull {MODEL}")
        raise SystemExit(2)

    os.makedirs(os.path.join(OUT_DIR, "crops"), exist_ok=True)
    rows = load_hausdorff_rows()
    n_ok = 0
    out_rows = []
    print(f"Gemma 3 ({MODEL}) after Hausdorff\n")
    for r in rows:
        stem = r["stem"]
        use = (not args.unsure_only) or r["unsure"]
        gemma_raw, gemma = "", ""
        if use:
            crop = os.path.join(OUT_DIR, "crops", f"{stem}.png")
            prep_for_gemma(r["path"], crop)
            print(f"  {stem}: asking Gemma ...", flush=True)
            try:
                gemma_raw = ask_gemma(crop)
            except urllib.error.URLError as e:
                gemma_raw = f"[error] {e}"
            gray = cv2.imread(crop, cv2.IMREAD_GRAYSCALE)
            gemma = parse_label(gemma_raw, gray)
            final = gemma or r["hausdorff"]
            src = "gemma" if gemma else "hausdorff"
        else:
            final = r["hausdorff"]
            src = "hausdorff"
        truth = GROUND_TRUTH[stem]
        ok = canonical(final) == canonical(truth)
        n_ok += int(ok)
        print(
            f"  {'OK  ' if ok else 'MISS'} {stem:8s}  "
            f"haus '{r['hausdorff']}'  gemma '{gemma}'  "
            f"final '{final}' ({src})  gt '{truth}'"
        )
        if gemma_raw:
            print(f"           raw: {gemma_raw!r}")
        out_rows.append({
            **{k: r[k] for k in r},
            "gemma_raw": gemma_raw,
            "gemma": gemma,
            "final": final,
            "source": src,
            "ok": ok,
        })
    print(f"\nAccuracy: {n_ok}/{len(rows)}")
    out_csv = os.path.join(OUT_DIR, "results.csv")
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(out_rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)
    print(f"Wrote {out_csv}")


if __name__ == "__main__":
    main()
