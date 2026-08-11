"""Detect a petri dish, antibiotic disks, and their zones of inhibition in a single image.

Usage: python3 petri_detect.py [path/to/image.png]
Defaults to sample/image.png. Writes <name>_detected.png next to the input.
"""
import sys
import os
import math
import cv2
import numpy as np

DISH_DIAMETER_MM = 90.0


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return img


def _hough(gray, w, min_frac, max_frac, param1, param2, dp=1.0):
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=dp, minDist=w * 0.05,
        param1=param1, param2=param2,
        minRadius=int(w * min_frac), maxRadius=int(w * max_frac),
    )
    if circles is None:
        return []
    return [tuple(c) for c in circles[0]]


def detect_dish(gray, w):
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    candidates = _hough(blurred, w, 0.35, 0.50, param1=80, param2=40)
    if not candidates:
        return None
    # biggest radius = the dish rim
    return max(candidates, key=lambda c: c[2])


def detect_disks(gray, w, dish):
    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    candidates = _hough(blurred, w, 0.02, 0.06, param1=60, param2=25)
    dx, dy, dr = dish
    disks = []
    for x, y, r in candidates:
        if math.hypot(x - dx, y - dy) >= dr - r:
            continue
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (int(x), int(y)), max(int(r * 0.7), 1), 255, -1)
        region = gray[mask == 255]
        if region.mean() < 160 or region.std() > 45:  # disks are uniform near-white paper
            continue
        # non-max suppression: skip if too close to an already-accepted disk
        if any(math.hypot(x - ax, y - ay) < 1.3 * max(r, ar) for ax, ay, ar in disks):
            continue
        disks.append((x, y, r))
    return disks


def find_zone_radius(gray, x, y, r_disk, max_r):
    """Radial intensity profile around a disk center; the zone boundary is the
    radius with the sharpest tonal transition (largest local derivative)."""
    lo = int(r_disk * 1.3)
    if max_r - lo < 10:
        return None
    radii = list(range(lo, int(max_r), 2))
    means = []
    for r in radii:
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (int(x), int(y)), r, 255, 2)
        vals = gray[mask == 255]
        means.append(float(vals.mean()) if vals.size else 0.0)
    means = np.array(means, dtype=np.float32)
    smoothed = np.convolve(means, np.ones(5) / 5, mode="same")
    deriv = np.abs(np.diff(smoothed))
    if deriv.size == 0:
        return None
    return radii[int(np.argmax(deriv))]


def detect_zones(gray, w, dish, disks):
    dx, dy, dr = dish
    zones = []
    for x, y, r in disks:
        room_left_in_dish = dr - math.hypot(x - dx, y - dy)
        # zones of inhibition are a few disk-widths across; capping the search
        # range keeps the derivative peak from latching onto the dish rim or a
        # neighboring disk's halo instead of this disk's own boundary
        max_r = min(room_left_in_dish, r * 5)
        zr = find_zone_radius(gray, x, y, r, max_r)
        if zr is not None:
            zones.append((x, y, zr))
    return zones


def annotate(img, dish, zones, disks, mm_per_px):
    out = img.copy()
    dx, dy, dr = dish
    cv2.circle(out, (int(dx), int(dy)), int(dr), (255, 0, 0), 2)  # blue
    cv2.putText(out, f"dish R={dr * mm_per_px:.1f}mm", (int(dx - dr), int(dy - dr) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

    for x, y, r in zones:
        cv2.circle(out, (int(x), int(y)), int(r), (0, 200, 0), 2)  # green
        cv2.putText(out, f"R={r * mm_per_px:.1f}mm", (int(x - r), int(y - r) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 0), 1, cv2.LINE_AA)

    for x, y, r in disks:
        cv2.circle(out, (int(x), int(y)), int(r), (0, 0, 255), 2)  # red
        cv2.putText(out, f"R={r * mm_per_px:.1f}mm", (int(x + r), int(y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
    return out


def process(path):
    img = load_image(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    w = img.shape[1]

    dish = detect_dish(gray, w)
    assert dish is not None and dish[2] > 0, "failed to detect petri dish outline"

    mm_per_px = DISH_DIAMETER_MM / (2 * dish[2])
    disks = detect_disks(gray, w, dish)
    zones = detect_zones(gray, w, dish, disks)

    print(f"dish: r={dish[2]:.1f}px -> R={dish[2] * mm_per_px:.1f}mm")
    for x, y, r in zones:
        print(f"zone at ({x:.0f},{y:.0f}): r={r:.1f}px -> R={r * mm_per_px:.1f}mm")
    for x, y, r in disks:
        print(f"disk at ({x:.0f},{y:.0f}): r={r:.1f}px -> R={r * mm_per_px:.1f}mm")

    return annotate(img, dish, zones, disks, mm_per_px)


def main():
    default = os.path.join(os.path.dirname(__file__), "sample", "image.png")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    result = process(path)
    root, ext = os.path.splitext(path)
    out_path = f"{root}_detected{ext}"
    cv2.imwrite(out_path, result)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
