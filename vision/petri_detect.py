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
import threading
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

# Tunable values: key -> (default, min, max, step, label). The GUI builds sliders from this.
SPEC = {
    "DISH_DIAMETER_MM": (DISH_DIAMETER_MM, 30, 150, 1, "dish diameter mm"),
    "DISH_MIN_R_FRAC": (0.30, 0.10, 0.60, 0.01, "dish min radius (x image width)"),
    "DISH_MAX_R_FRAC": (0.52, 0.20, 0.80, 0.01, "dish max radius (x image width)"),
    "DISH_HOUGH_P2": (25, 8, 80, 1, "dish Hough sensitivity (low = more candidates)"),
    "DISH_BLUR": (4, 1, 12, 1, "dish blur"),
    "DISK_MIN_R_MM": (DISK_MIN_R_MM, 1.0, 6.0, 0.1, "disk min radius mm"),
    "DISK_MAX_R_MM": (DISK_MAX_R_MM, 2.0, 8.0, 0.1, "disk max radius mm"),
    "DISK_HOUGH_P2": (25, 8, 60, 1, "disk Hough sensitivity (low = more)"),
    "DISK_MIN_BRIGHTNESS": (DISK_MIN_BRIGHTNESS, 60, 250, 1, "disk min brightness"),
    "DISK_MAX_STDDEV": (DISK_MAX_STDDEV, 5, 120, 1, "disk max spread (letters ignored)"),
    "DISK_MIN_LOCAL_CONTRAST": (DISK_MIN_LOCAL_CONTRAST, 0, 80, 1, "disk min contrast vs agar"),
    "ZONE_MIN_R_MM": (ZONE_MIN_R_MM, 2.0, 15.0, 0.5, "zone min radius mm"),
    "ZONE_MAX_R_MM": (ZONE_MAX_R_MM, 8.0, 40.0, 0.5, "zone max radius mm"),
    "ZONE_MIN_SLOPE": (ZONE_MIN_SLOPE, 0.1, 5.0, 0.1, "zone edge strength"),
    "ZONE_MIN_AGREEMENT": (ZONE_MIN_AGREEMENT, 0.2, 1.0, 0.05, "zone roundness"),
}
INT_KEYS = {"DISH_HOUGH_P2", "DISH_BLUR", "DISK_HOUGH_P2"}
P = {k: v[0] for k, v in SPEC.items()}
P["ZONE_SECTORS"] = ZONE_SECTORS
P["MIN_RING_COVERAGE"] = MIN_RING_COVERAGE
P["ZONE_SEARCH_FROM_DISK"] = ZONE_SEARCH_FROM_DISK
_lock = threading.Lock()


def load_image(path):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"could not read image: {path}")
    return img


def _edge_score(g, x, y, r, n=120, band=3):
    """(support, mean contrast): how much of the circle sits on a real brightness step."""
    h, w = g.shape
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(t), np.sin(t)

    def sample(rad):
        xs, ys = np.rint(x + rad * c).astype(int), np.rint(y + rad * s).astype(int)
        ok = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        out = np.zeros(n)
        out[ok] = g[ys[ok], xs[ok]]
        return out, ok

    inner, ok1 = sample(r - band)
    outer, ok2 = sample(r + band)
    ok = ok1 & ok2
    if ok.sum() < n * 0.4:  # mostly outside the frame
        return 0.0, 0.0
    d = np.abs(inner - outer)[ok]
    return float((d > 10).mean()) * ok.sum() / n, float(d.mean())


def _refine(g, x, y, r):
    best = _edge_score(g, x, y, r)
    best_v = best[0] * best[1]
    for _ in range(12):
        moved = False
        for dx in (-2, 0, 2):
            for dy in (-2, 0, 2):
                for dr in (-2, 0, 2):
                    if dx == dy == dr == 0:
                        continue
                    sc = _edge_score(g, x + dx, y + dy, r + dr)
                    if sc[0] * sc[1] > best_v + 1e-6:
                        best_v, x, y, r, moved = sc[0] * sc[1], x + dx, y + dy, r + dr, True
        if not moved:
            break
    return x, y, r, best_v


def detect_dish(gray, w):
    """Rim of the dish. Hough and Otsu-contour candidates are each scored by how much of
    the circle lies on a real edge, refined locally, and the best one wins."""
    k = int(P["DISH_BLUR"]) * 2 + 1
    blurred = cv2.GaussianBlur(gray, (k, k), P["DISH_BLUR"] / 2.0)
    rmin, rmax = w * P["DISH_MIN_R_FRAC"], w * P["DISH_MAX_R_FRAC"]
    cands = []
    circles = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=w * 0.02,
        param1=80, param2=P["DISH_HOUGH_P2"], minRadius=int(rmin), maxRadius=int(rmax),
    )
    if circles is not None:
        cands += [tuple(map(float, c)) for c in circles[0][:12]]
    for inv in (False, True):  # bright dish on dark table, or dark dish on bright table
        flag = cv2.THRESH_BINARY_INV if inv else cv2.THRESH_BINARY
        _, th = cv2.threshold(blurred, 0, 255, flag | cv2.THRESH_OTSU)
        cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            (cx, cy), cr = cv2.minEnclosingCircle(max(cnts, key=cv2.contourArea))
            if rmin <= cr <= rmax * 1.2:
                cands.append((cx, cy, cr))
    best = None
    for x, y, r in cands:
        x, y, r, v = _refine(blurred, x, y, r)
        if rmin * 0.9 <= r <= rmax * 1.1 and (best is None or v * r * r > best[3]):
            best = (x, y, r, v * r * r)  # slight preference for the outer rim
    return None if best is None else (best[0], best[1], best[2])


def _ring_vals(gray, mask, x, y, r_inner, r_outer):
    ring = np.zeros(gray.shape, dtype=np.uint8)
    cv2.circle(ring, (int(x), int(y)), int(r_outer), 255, -1)
    cv2.circle(ring, (int(x), int(y)), int(r_inner), 0, -1)
    return gray[(ring == 255) & mask]


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
        blurred, cv2.HOUGH_GRADIENT, dp=1.0, minDist=P["DISK_MIN_R_MM"] / mm_per_px * 2,
        param1=60, param2=P["DISK_HOUGH_P2"],
        minRadius=int(P["DISK_MIN_R_MM"] / mm_per_px),
        maxRadius=int(P["DISK_MAX_R_MM"] / mm_per_px),
    )
    if circles is None:
        return []

    disks = []
    for x, y, r in (tuple(c) for c in circles[0]):
        # must sit well inside the dish, not on the rim where glare lives
        if math.hypot(x - dx, y - dy) > dr * 0.88:
            continue
        core = _ring_vals(gray, inside, x, y, 0, r * 0.7)
        if core.size < 10:
            continue
        # ink letters are a dark minority: judge the paper by its brighter pixels
        core_mean = float(np.percentile(core, 75))
        paper = core[core >= np.percentile(core, 30)]
        core_std = float(paper.std())
        if core_mean < P["DISK_MIN_BRIGHTNESS"] or core_std > P["DISK_MAX_STDDEV"]:
            continue
        surround_mean, _, n = _ring_stats(gray, inside, x, y, r * 1.6, r * 2.6)
        if n == 0 or core_mean - surround_mean < P["DISK_MIN_LOCAL_CONTRAST"]:
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
    sector = (((np.arctan2(dy, dx) + np.pi) / (2 * np.pi)) * P["ZONE_SECTORS"]).astype(np.int32)
    sector %= P["ZONE_SECTORS"]
    g = gray.astype(np.float64)

    def sector_means(r_from, r_to):
        m = valid & (dist >= r_from) & (dist < r_to)
        counts = np.bincount(sector[m], minlength=P["ZONE_SECTORS"]).astype(np.float64)
        sums = np.bincount(sector[m], weights=g[m], minlength=P["ZONE_SECTORS"])
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
    must be both a real step (P["ZONE_MIN_SLOPE"]) and actually circular
    (P["ZONE_MIN_AGREEMENT"]); the innermost survivor wins, since Kirby-Bauer
    measures the zone of complete inhibition rather than any outer halo.
    Returns None when nothing qualifies -- a resistant organism or a blank
    control disk genuinely has no zone.
    """
    lo = max(int(r_disk * P["ZONE_SEARCH_FROM_DISK"]), int(P["ZONE_MIN_R_MM"] / mm_per_px))
    hi = int(P["ZONE_MAX_R_MM"] / mm_per_px)
    if hi - lo < 6:
        return None

    mean, coverage = _radial_profile(gray, valid, x, y, hi)
    usable = coverage >= P["MIN_RING_COVERAGE"]
    smoothed = cv2.GaussianBlur(mean.astype(np.float32).reshape(-1, 1), (0, 0), 3).ravel()
    slope = np.gradient(smoothed)

    for r in range(lo, min(hi, len(slope) - 1)):
        if not usable[r] or slope[r] < P["ZONE_MIN_SLOPE"]:
            continue
        if not (slope[r] >= slope[r - 1] and slope[r] >= slope[r + 1]):
            continue  # must be a local maximum, not the tail of the disk's halo
        if _sector_agreement(gray, valid, x, y, r) < P["ZONE_MIN_AGREEMENT"]:
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


def set_params(params=None):
    """Reset to defaults, then apply overrides (unknown keys ignored, values clamped)."""
    for k, spec in SPEC.items():
        v = (params or {}).get(k, spec[0])
        v = min(spec[2], max(spec[1], float(v)))
        P[k] = int(round(v)) if k in INT_KEYS else v


def find_dish(img, params=None):
    """Dish only (cheap), for the live outline."""
    with _lock:
        set_params(params)
        return detect_dish(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), img.shape[1])


def analyze(img, params=None, manual_dish=None):
    """Detect dish, disks and zones in a BGR image. Raises ValueError when no dish is found.

    params: overrides for SPEC keys. manual_dish: (x, y, r) in pixels replaces the auto dish.
    Returns dict: dish (x, y, r px), mm_per_px, disks [(x, y, r)], zones [(x, y, r, clipped)],
    annotated image.
    """
    with _lock:
        set_params(params)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        dish = tuple(manual_dish) if manual_dish else detect_dish(gray, img.shape[1])
        if dish is None or dish[2] <= 0:
            raise ValueError("failed to detect petri dish outline")
        mm_per_px = P["DISH_DIAMETER_MM"] / (2 * dish[2])
        disks = detect_disks(gray, dish, mm_per_px)
        zones = detect_zones(gray, dish, disks, mm_per_px)
    return {
        "dish": dish, "mm_per_px": mm_per_px, "disks": disks, "zones": zones,
        "annotated": annotate(img, dish, zones, disks, mm_per_px),
    }


def process(path):
    img = load_image(path)
    res = analyze(img)
    dish, mm_per_px, disks, zones = res["dish"], res["mm_per_px"], res["disks"], res["zones"]
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

    return res["annotated"]


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
