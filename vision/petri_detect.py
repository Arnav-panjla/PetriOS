"""Detect a petri dish, antibiotic disks, and their zones of inhibition in a single image.

Usage: python3 petri_detect.py [path/to/image.png]
Defaults to sample/s1.png. Writes <name>_detected.png next to the input.

Colors: dish = blue, antibiotic disk = red, zone of inhibition = purple.
A disk with no measurable zone (resistant organism, or a blank control disk)
gets no purple circle at all -- that is a real result, not a failure.
"""
import sys
import os
import math
import cv2
import numpy as np

DISH_DIAMETER_MM = 90.0

# Physical priors for a Kirby-Bauer plate. Anchoring to mm (via the known 90mm
# dish) instead of pixel fractions is what keeps false positives down.
DISK_MIN_R_MM = 1.6          # standard disk is 6mm dia; allow for perspective
DISK_MAX_R_MM = 4.5
DISK_MIN_BRIGHTNESS = 150    # disks are white paper
DISK_MAX_STDDEV = 45         # ...and flat/uniform, unlike textured agar
DISK_MIN_LOCAL_CONTRAST = 12 # ...and clearly brighter than the agar around them

ZONE_MAX_R_MM = 20.0         # a 40mm-dia zone is about the largest ever reported
ZONE_MIN_R_MM = 4.0          # smaller than this is indistinguishable from the disk
ZONE_SEARCH_FROM_DISK = 2.0  # start searching this many disk-radii out, past its halo
# The two knobs that trade false positives against missed zones. Loosening
# them much further starts re-admitting blank control disks, which score up to
# ~0.56 agreement on these plates; tighten them if you see spurious circles.
ZONE_MIN_SLOPE = 0.5         # gray levels per px: a boundary must be a real step
ZONE_MIN_AGREEMENT = 0.60    # fraction of angular sectors that must see that step
ZONE_SECTORS = 16
MIN_RING_COVERAGE = 0.30     # fraction of a ring that must lie inside the dish

BLUE, RED, PURPLE = (255, 0, 0), (0, 0, 255), (211, 0, 148)


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return img


def detect_dish(gray, w):
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=w * 0.05,
        param1=80, param2=40,
        minRadius=int(w * 0.35), maxRadius=int(w * 0.50),
    )
    if circles is None:
        return None
    return max((tuple(c) for c in circles[0]), key=lambda c: c[2])  # rim = biggest


def _ring_stats(gray, mask, x, y, r_inner, r_outer):
    """mean, stddev and pixel count of the annulus [r_inner, r_outer)."""
    ring = np.zeros(gray.shape, dtype=np.uint8)
    cv2.circle(ring, (int(x), int(y)), int(r_outer), 255, -1)
    cv2.circle(ring, (int(x), int(y)), int(r_inner), 0, -1)
    sel = (ring == 255) & mask
    vals = gray[sel]
    if vals.size == 0:
        return 0.0, 0.0, 0
    return float(vals.mean()), float(vals.std()), int(vals.size)


def detect_disks(gray, dish, mm_per_px):
    """Small, bright, uniform circles that are clearly brighter than the agar
    surrounding them -- the local-contrast test is what rejects gloves, labels
    and dish-rim glare, which are bright but sit on bright backgrounds."""
    dx, dy, dr = dish
    inside = np.zeros(gray.shape, dtype=np.uint8)
    cv2.circle(inside, (int(dx), int(dy)), int(dr), 255, -1)
    inside = inside == 255

    blurred = cv2.GaussianBlur(gray, (5, 5), 1)
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=DISK_MIN_R_MM / mm_per_px * 2,
        param1=60, param2=25,
        minRadius=int(DISK_MIN_R_MM / mm_per_px),
        maxRadius=int(DISK_MAX_R_MM / mm_per_px),
    )
    if circles is None:
        return []

    disks = []
    for x, y, r in (tuple(c) for c in circles[0]):
        # must sit well inside the dish, not on the rim where glare lives
        if math.hypot(x - dx, y - dy) > dr * 0.88:
            continue
        core_mean, core_std, _ = _ring_stats(gray, inside, x, y, 0, r * 0.7)
        if core_mean < DISK_MIN_BRIGHTNESS or core_std > DISK_MAX_STDDEV:
            continue
        surround_mean, _, n = _ring_stats(gray, inside, x, y, r * 1.6, r * 2.6)
        if n == 0 or core_mean - surround_mean < DISK_MIN_LOCAL_CONTRAST:
            continue
        if any(math.hypot(x - ax, y - ay) < 1.5 * max(r, ar) for ax, ay, ar in disks):
            continue
        disks.append((x, y, r))
    return disks


def _radial_profile(gray, valid, x, y, max_r):
    """Per-radius mean intensity and the fraction of each ring that was sampled.

    Only pixels in `valid` are sampled, so a disk near the dish wall is measured
    over the arc that still lies on agar instead of being dragged outward by the
    rim -- its zone is then estimated from the part that is actually visible,
    and the coverage fraction records how much of the ring that was.
    """
    h, w = gray.shape
    yy, xx = np.ogrid[:h, :w]
    rbin = np.sqrt((xx - x) ** 2 + (yy - y) ** 2).astype(np.int32)

    n = max_r + 1
    in_range = rbin <= max_r
    sel = valid & in_range
    idx = rbin[sel]
    counts = np.bincount(idx, minlength=n).astype(np.float64)
    total = np.bincount(rbin[in_range], minlength=n).astype(np.float64)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.bincount(idx, weights=gray[sel].astype(np.float64), minlength=n) / counts
        coverage = counts / np.maximum(total, 1)
    return np.nan_to_num(mean), coverage


def _sector_agreement(gray, valid, x, y, r, k=5):
    """Fraction of angular sectors that see the same brightness step at radius r.

    This is the test that separates a real zone boundary from an artifact. A
    zone is a *circle*, so nearly every sector around the disk must show the
    same clear-agar-to-lawn step. A printed label on the plate or the camera's
    lighting gradient also produces a step in the radial average, but only on
    one side, so it scores low and gets thrown out.
    """
    h, w = gray.shape
    yy, xx = np.ogrid[:h, :w]
    dy, dx = yy - y, xx - x
    dist = np.sqrt(dx * dx + dy * dy)
    sector = (((np.arctan2(dy, dx) + np.pi) / (2 * np.pi)) * ZONE_SECTORS).astype(np.int32)
    sector %= ZONE_SECTORS
    g = gray.astype(np.float64)

    def sector_means(r_from, r_to):
        m = valid & (dist >= r_from) & (dist < r_to)
        counts = np.bincount(sector[m], minlength=ZONE_SECTORS).astype(np.float64)
        sums = np.bincount(sector[m], weights=g[m], minlength=ZONE_SECTORS)
        with np.errstate(invalid="ignore", divide="ignore"):
            return sums / counts, counts

    inner, n_in = sector_means(r - k, r)
    outer, n_out = sector_means(r, r + k)
    ok = (n_in >= 6) & (n_out >= 6)
    if ok.sum() < 4:
        return 0.0
    return float(((outer - inner)[ok] > 1.5).mean())


def find_zone(gray, valid, x, y, r_disk, mm_per_px):
    """Locate this disk's zone boundary, or return None if it has no zone.

    Candidates are local maxima of the outward-brightening radial gradient --
    cleared agar is darker than the bacterial lawn beyond it. Each candidate
    must be both a real step (ZONE_MIN_SLOPE) and actually circular
    (ZONE_MIN_AGREEMENT); the innermost survivor wins, since Kirby-Bauer
    measures the zone of complete inhibition rather than any outer halo.
    Returns None when nothing qualifies -- a resistant organism or a blank
    control disk genuinely has no zone.
    """
    lo = max(int(r_disk * ZONE_SEARCH_FROM_DISK), int(ZONE_MIN_R_MM / mm_per_px))
    hi = int(ZONE_MAX_R_MM / mm_per_px)
    if hi - lo < 6:
        return None

    mean, coverage = _radial_profile(gray, valid, x, y, hi)
    usable = coverage >= MIN_RING_COVERAGE
    smoothed = cv2.GaussianBlur(mean.astype(np.float32).reshape(-1, 1), (0, 0), 3).ravel()
    slope = np.gradient(smoothed)

    for r in range(lo, min(hi, len(slope) - 1)):
        if not usable[r] or slope[r] < ZONE_MIN_SLOPE:
            continue
        if not (slope[r] >= slope[r - 1] and slope[r] >= slope[r + 1]):
            continue  # must be a local maximum, not the tail of the disk's halo
        if _sector_agreement(gray, valid, x, y, r) < ZONE_MIN_AGREEMENT:
            continue
        return r, bool(coverage[r] < 0.95)
    return None


def detect_zones(gray, dish, disks, mm_per_px):
    """At most one zone per disk; disks with no measurable zone are skipped."""
    dx, dy, dr = dish
    zones = []
    for x, y, r in disks:
        valid = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(valid, (int(dx), int(dy)), int(dr * 0.97), 255, -1)
        # blank out the other disks so their bright paper cannot be mistaken
        # for this disk's zone boundary
        for ox, oy, orad in disks:
            if (ox, oy) != (x, y):
                cv2.circle(valid, (int(ox), int(oy)), int(orad * 1.6), 0, -1)
        found = find_zone(gray, valid == 255, x, y, r, mm_per_px)
        if found is not None:
            zr, clipped = found
            zones.append((x, y, zr, clipped))
    return zones


def annotate(img, dish, zones, disks, mm_per_px):
    out = img.copy()
    dx, dy, dr = dish
    cv2.circle(out, (int(dx), int(dy)), int(dr), BLUE, 2)
    cv2.putText(out, f"dish R={dr * mm_per_px:.1f}mm", (int(dx - dr) + 5, int(dy - dr) + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, BLUE, 1, cv2.LINE_AA)

    for x, y, r, clipped in zones:
        cv2.circle(out, (int(x), int(y)), int(r), PURPLE, 2)
        label = f"R>={r * mm_per_px:.1f}mm*" if clipped else f"R={r * mm_per_px:.1f}mm"
        cv2.putText(out, label, (int(x - r) + 4, int(y - r) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, PURPLE, 1, cv2.LINE_AA)

    for x, y, r in disks:
        cv2.circle(out, (int(x), int(y)), int(r), RED, 2)
        cv2.putText(out, f"{r * mm_per_px:.1f}mm", (int(x + r) + 2, int(y) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, RED, 1, cv2.LINE_AA)
    return out


def process(path):
    img = load_image(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    dish = detect_dish(gray, img.shape[1])
    assert dish is not None and dish[2] > 0, "failed to detect petri dish outline"

    mm_per_px = DISH_DIAMETER_MM / (2 * dish[2])
    disks = detect_disks(gray, dish, mm_per_px)
    zones = detect_zones(gray, dish, disks, mm_per_px)
    zone_at = {(x, y): (r, clipped) for x, y, r, clipped in zones}

    print(f"dish: R={dish[2] * mm_per_px:.1f}mm  ({mm_per_px:.4f} mm/px)")
    for x, y, r in disks:
        zone = zone_at.get((x, y))
        if zone is None:
            desc = "no zone of inhibition"
        else:
            zr, clipped = zone
            desc = f"zone R={zr * mm_per_px:.1f}mm"
            if clipped:
                desc += " (clipped by dish edge -- lower bound)"
        print(f"disk at ({x:.0f},{y:.0f}) R={r * mm_per_px:.1f}mm -> {desc}")
    print(f"{len(disks)} disk(s), {len(zones)} with a measurable zone")

    return annotate(img, dish, zones, disks, mm_per_px)


def main():
    default = os.path.join(os.path.dirname(__file__), "sample", "s1.png")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    result = process(path)
    root, ext = os.path.splitext(path)
    out_path = f"{root}_detected{ext}"
    cv2.imwrite(out_path, result)
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
