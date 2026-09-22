"""
extract_binary_discs.py
=======================
Detect discs the same way as antibiotic_ocr_v2 / antibiotic_ocr10,
then for each disc:
  - crop + upscale to TARGET_DIAMETER
  - paint outside the circle white
  - Otsu -> black letters, white disc

No OCR. No rotation.

Run:  python extract_binary_discs.py
Out:  test1_binary_discs/
"""
import math
import os

import cv2
import numpy as np

IMAGE = "test1.png"
OUT_DIR = "test1_binary_discs"
TARGET_DIAMETER = 300  # same as antibiotic_ocr_v2
INNER_FRAC = 0.94      # slight inset from detected rim; keep letters intact
RIM_BAND_FRAC = 0.92   # only strip blobs whose center sits outside this


def detect_discs(image):
    """Copied from antibiotic_ocr_v2.detect_discs — the version that worked."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 1.5)
    h, w = gray.shape

    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=30,
        param1=80,
        param2=18,
        minRadius=8,
        maxRadius=22,
    )

    if circles is None:
        return []

    circles = np.round(circles[0]).astype(int)
    discs = []

    for x, y, r in circles:
        margin = 2
        if (x - r < margin or y - r < margin
                or x + r >= w - margin or y + r >= h - margin):
            continue

        inner_r = max(3, int(r * 0.5))
        roi = gray[y - inner_r: y + inner_r, x - inner_r: x + inner_r]
        if roi.size == 0 or np.mean(roi) < 130:
            continue

        is_dup = any(
            math.hypot(x - xx, y - yy) < min(r, rr) * 1.5
            for xx, yy, rr in discs
        )
        if not is_dup:
            discs.append((x, y, r))

    discs.sort(key=lambda c: (c[1], c[0]))
    return discs


def make_binary_disc(image, x, y, r):
    """Crop, upscale, white-out outside circle, Otsu (black letters)."""
    pad = int(r * 1.4)
    x1, y1 = max(0, x - pad), max(0, y - pad)
    x2, y2 = min(image.shape[1], x + pad), min(image.shape[0], y + pad)
    crop = image[y1:y2, x1:x2]

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)

    scale = TARGET_DIAMETER / (2.0 * r)
    new_w = max(64, int(gray.shape[1] * scale))
    new_h = max(64, int(gray.shape[0] * scale))
    up = cv2.resize(gray, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
    color_up = cv2.resize(crop, (new_w, new_h), interpolation=cv2.INTER_CUBIC)

    cx, cy = new_w // 2, new_h // 2
    # Shrink below the physical pad edge so the dark rim never enters the mask.
    # Detected r hugs the outer rim; text sits well inside (~0.7-0.85 of r).
    disc_r = int(r * scale * INNER_FRAC)
    mask = np.zeros((new_h, new_w), np.uint8)
    cv2.circle(mask, (cx, cy), disc_r, 255, -1)

    # outside circle -> white
    masked = np.where(mask > 0, up, 255).astype(np.uint8)

    disc_vals = up[mask > 0]
    thr, _ = cv2.threshold(disc_vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    binary = np.full((new_h, new_w), 255, np.uint8)
    binary[(mask > 0) & (up < thr)] = 0

    ink_frac = float((binary[mask > 0] == 0).sum()) / float((mask > 0).sum())
    if ink_frac > 0.45:
        binary = np.where(mask > 0, 255 - binary, 255).astype(np.uint8)

    # Drop any leftover arc fragments hugging the cut edge.
    binary = _strip_rim_arcs(binary, cx, cy, disc_r)

    return color_up, masked, binary


def _strip_rim_arcs(binary, cx, cy, disc_r):
    """Remove thin black arcs on the very outer edge only — leave letter strokes."""
    ink = (binary == 0).astype(np.uint8) * 255
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ink, 8)
    if n <= 1:
        return binary

    yy, xx = np.ogrid[:binary.shape[0], :binary.shape[1]]
    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)
    out = binary.copy()
    rim_band = disc_r * RIM_BAND_FRAC

    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        # only tiny/skinny leftovers — never large letter blobs
        if area > 80:
            continue
        ys, xs = np.where(labels == i)
        if float(dist[ys, xs].mean()) >= rim_band:
            out[labels == i] = 255
    return out


def main():
    img = cv2.imread(IMAGE)
    if img is None:
        raise SystemExit(f"Could not read {IMAGE}")

    os.makedirs(OUT_DIR, exist_ok=True)
    discs = detect_discs(img)
    print(f"{IMAGE}: found {len(discs)} discs  (antibiotic_ocr_v2 detector)")

    annotated = img.copy()
    thumbs = []

    for i, (x, y, r) in enumerate(discs, 1):
        color_up, masked, binary = make_binary_disc(img, x, y, r)
        stem = f"disc_{i:02d}"
        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}.png"), binary)
        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_masked.png"), masked)
        cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_color.png"), color_up)

        cv2.circle(annotated, (x, y), r, (0, 255, 0), 2)
        cv2.putText(annotated, str(i), (x - 8, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
        thumbs.append((stem, binary))
        print(f"  {stem}  ({x},{y}) r={r}  out={binary.shape[1]}x{binary.shape[0]}")

    cv2.imwrite(os.path.join(OUT_DIR, "_annotated.png"), annotated)

    if thumbs:
        cols, cell = 6, 140
        n_rows = (len(thumbs) + cols - 1) // cols
        sheet = np.full((n_rows * (cell + 24), cols * cell, 3), 40, np.uint8)
        for idx, (name, binary) in enumerate(thumbs):
            rr, cc = divmod(idx, cols)
            big = cv2.resize(binary, (cell - 8, cell - 8),
                             interpolation=cv2.INTER_AREA)
            big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
            oy, ox = rr * (cell + 24) + 4, cc * cell + 4
            sheet[oy:oy + big.shape[0], ox:ox + big.shape[1]] = big
            cv2.putText(sheet, name, (ox, oy + big.shape[0] + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1,
                        cv2.LINE_AA)
        cv2.imwrite(os.path.join(OUT_DIR, "_contact_sheet.png"), sheet)

    print(f"Wrote {len(thumbs)} binary discs to {OUT_DIR}/")


if __name__ == "__main__":
    main()
