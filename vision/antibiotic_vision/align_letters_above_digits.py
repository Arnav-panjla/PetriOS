"""
align_letters_above_digits.py
=============================
After method-C (baseline horizontal, but 0/180 ambiguous):

Assumption (for discs that print a dose): when upright, LETTERS are on the
TOP row and NUMBERS on the BOTTOM row.

  - Split ink into two horizontal rows
  - Score 0 vs 180: prefer letters-up / digits-down
  - Letters-only discs (e.g. SXT): leave as-is (no flip)

Input:  morph open 5x5 x2 after C  (test1_morph/disc_XX_04_open_5x5_x2.png)
        If missing, C from test1_aligned/ then open 5x5 x2.
Out:    test1_letters_above/

Run:  python align_letters_above_digits.py
"""
import os
import re

import cv2
import numpy as np

from align_binary_discs import ink_mask, list_discs, rotate, sheet
from morph_stroke_separate import stroke_separate

ALIGNED_DIR = "test1_aligned"
MORPH_DIR = "test1_morph"
MORPH_NAME = "04_open_5x5_x2"
OUT_DIR = "test1_letters_above"
ALLOWLIST = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"


def load_c(stem):
    best, best_mtime = None, -1.0
    if os.path.isdir(ALIGNED_DIR):
        for f in os.listdir(ALIGNED_DIR):
            if f.startswith(f"{stem}_C_two_line_") and f.endswith(".png"):
                p = os.path.join(ALIGNED_DIR, f)
                m = os.path.getmtime(p)
                if m > best_mtime:
                    best_mtime, best = m, p
    if best is None:
        return None, None
    return cv2.imread(best, cv2.IMREAD_GRAYSCALE), os.path.basename(best)


def load_morph_after_c(stem):
    """C-aligned disc after open 5x5 x2 — this is the input for 0/180 flip."""
    morph_path = os.path.join(MORPH_DIR, f"{stem}_{MORPH_NAME}.png")
    if os.path.isfile(morph_path):
        img = cv2.imread(morph_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            return img, os.path.basename(morph_path)
    c_img, src = load_c(stem)
    if c_img is None:
        return None, None
    return stroke_separate(c_img, "open", 5, 2), f"{src} + open5x5x2"


def _row_blobs(mask, min_area=80):
    """Letter-sized connected components with centres."""
    h, w = mask.shape
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    img_area = h * w
    cx0, cy0 = w / 2, h / 2
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        bw = int(stats[i, cv2.CC_STAT_WIDTH])
        bh = int(stats[i, cv2.CC_STAT_HEIGHT])
        if area < min_area or area > img_area * 0.15:
            continue
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        dist = float(np.hypot(cents[i][0] - cx0, cents[i][1] - cy0))
        # drop long rim arcs
        if aspect > 4.0 and dist > min(h, w) * 0.28:
            continue
        blobs.append({
            "cx": float(cents[i][0]),
            "cy": float(cents[i][1]),
            "x": int(stats[i, cv2.CC_STAT_LEFT]),
            "y": int(stats[i, cv2.CC_STAT_TOP]),
            "w": bw,
            "h": bh,
        })
    return blobs


def split_two_rows(mask, min_area=80):
    """
    Cluster blob centres into 1–2 horizontal rows (by y).
    Returns list of rows; each row is list of blobs, rows sorted top→bottom.
    """
    blobs = _row_blobs(mask, min_area=min_area)
    if not blobs:
        return []
    hs = [b["h"] for b in blobs]
    letter_h = float(np.median(hs)) if hs else 20.0
    blobs = sorted(blobs, key=lambda b: b["cy"])
    rows = [[blobs[0]]]
    for b in blobs[1:]:
        if abs(b["cy"] - rows[-1][-1]["cy"]) <= letter_h * 0.7:
            rows[-1].append(b)
        else:
            rows.append([b])
    # if >2 rows, keep the two with most ink (blob count)
    if len(rows) > 2:
        rows = sorted(rows, key=len, reverse=True)[:2]
        rows = sorted(rows, key=lambda r: np.mean([b["cy"] for b in r]))
    return rows


def row_crop(gray, row, pad=6):
    h, w = gray.shape
    xs = [b["x"] for b in row] + [b["x"] + b["w"] for b in row]
    ys = [b["y"] for b in row] + [b["y"] + b["h"] for b in row]
    x1 = max(min(xs) - pad, 0)
    y1 = max(min(ys) - pad, 0)
    x2 = min(max(xs) + pad, w)
    y2 = min(max(ys) + pad, h)
    crop = gray[y1:y2, x1:x2]
    if crop.size == 0:
        return gray
    # mild upscale for OCR
    return cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)


def classify_row_text(crop, reader):
    """Return (n_letters, n_digits, raw_text) from a row crop."""
    hits = reader.readtext(crop, detail=1, allowlist=ALLOWLIST)
    if not hits:
        inv = 255 - crop
        hits = reader.readtext(inv, detail=1, allowlist=ALLOWLIST)
    text = "".join(h[1] for h in hits).upper()
    flat = re.sub(r"[^A-Z0-9]", "", text)
    letters = sum(ch.isalpha() for ch in flat)
    digits = sum(ch.isdigit() for ch in flat)
    return letters, digits, flat


def score_letters_above_digits(gray, reader):
    """
    Higher score => looks upright under letters-top / digits-bottom prior.
    Letters-only (one row): return None-like neutral (0) and flag no_dose.
    """
    mask = ink_mask(gray)
    rows = split_two_rows(mask)

    if len(rows) < 2:
        return 0.0, "single_row", "", "", False

    top, bot = rows[0], rows[1]
    top_L, top_D, top_t = classify_row_text(row_crop(gray, top), reader)
    bot_L, bot_D, bot_t = classify_row_text(row_crop(gray, bot), reader)

    has_dose_signal = (top_D + bot_D) >= 1 and (top_L + bot_L) >= 1

    # Ideal: top mostly letters, bottom mostly digits
    score = 0.0
    score += 2.0 * top_L + 3.0 * bot_D
    score -= 3.0 * top_D + 2.0 * bot_L

    clean_upright = top_L >= 2 and top_D == 0 and bot_D >= 1 and bot_L == 0
    clean_inverted = top_D >= 1 and top_L == 0 and bot_L >= 2 and bot_D == 0
    if clean_upright:
        score += 10.0
    if clean_inverted:
        score -= 10.0

    # No clear dose row → don't prefer either orientation
    if not has_dose_signal:
        score = 0.0

    return score, f"top={top_t}|bot={bot_t}", top_t, bot_t, has_dose_signal


def flip_to_letters_above(gray, reader):
    """
    Try 0 and 180; keep the orientation with better letters-above-digits score.
    If neither orientation shows a clear dose row, keep 0 (C as-is).
    """
    scored = []
    for ang in (0, 180):
        rot = rotate(gray, float(ang))
        score, info, top_t, bot_t, has_dose = score_letters_above_digits(rot, reader)
        scored.append((ang, score, info, top_t, bot_t, has_dose, rot))

    # If no orientation has letters+digits signal, do not flip
    if not any(s[5] for s in scored):
        ang0 = next(s for s in scored if s[0] == 0)
        return ang0[6], 0, ang0[1], ang0[2] + " (no_dose_keep)", scored

    # Letters-only at 0 (no digits read): keep 0 — avoid flipping on OCR junk digits at 180
    s0 = next(s for s in scored if s[0] == 0)
    top0, bot0 = s0[3], s0[4]
    if top0 and bot0:
        flat0 = (top0 + bot0)
        if flat0.isalpha():
            return s0[6], 0, s0[1], s0[2] + " (letters_only_keep)", scored
    elif (top0 or bot0) and (top0 + bot0).isalpha() and not (s0[3] and s0[4]):
        # one row of letters only
        if (top0 + bot0).isalpha():
            return s0[6], 0, s0[1], s0[2] + " (letters_only_keep)", scored

    best = max(scored, key=lambda s: s[1])
    ang, score, info, top_t, bot_t, has_dose, rot = best
    return rot, ang, score, info, scored


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    if not os.path.isdir(ALIGNED_DIR):
        raise SystemExit(f"Missing {ALIGNED_DIR}/ — run align_binary_discs.py first")

    print("Loading EasyOCR (row classification only)...")
    import easyocr
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)

    files = list_discs()
    before_sheet = []
    after_sheet = []

    print(f"Letters-above-digits flip on morph {MORPH_NAME} (after C)\n")
    for f in files:
        stem = os.path.splitext(f)[0]
        img, src = load_morph_after_c(stem)
        if img is None:
            print(f"[{stem}] no morph/C image — skip")
            continue

        upright, ang, score, info, details = flip_to_letters_above(img, reader)
        print(f"[{stem}] source={src}")
        for item in details:
            a, s, inf = item[0], item[1], item[2]
            mark = " <--" if a == ang else ""
            print(f"  {a:3d}: score={s:+.2f}  {inf}{mark}")
        print(f"  -> choose {ang} deg  ({info})\n")

        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_in_morph.png"), img)
        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_out_{ang}.png"), upright)
        before_sheet.append((stem, img))
        after_sheet.append((f"{stem} {ang}", upright))

    sheet(before_sheet, os.path.join(OUT_DIR, "_00_before_morph.png"))
    sheet(after_sheet, os.path.join(OUT_DIR, "_01_letters_above.png"))
    print(f"Done. Compare:\n  {OUT_DIR}/_00_before_morph.png\n  {OUT_DIR}/_01_letters_above.png")


if __name__ == "__main__":
    main()
