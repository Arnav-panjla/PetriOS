"""
detect_discs_v2.py
==================
New disc finder. Does not change extract_binary_discs.py / run_stock_plates.py.

Old finder used Hough radii 8–22 (test1 pixels) and dropped interiors darker
than 130. That missed whole plates (p06, larger discs) and the MEM disc in a
dark inhibition zone, and it treated dish-rim glare as discs.

Here: find the petri dish first, search only inside it, scale disc radius to
the dish, reject rim hits, keep paper-like spots that are brighter than the
agar ring around them.

Run:  python detect_discs_v2.py
Out:  detect_v2_out/<plate>/_annotated.png
"""
import math
import os

import cv2
import numpy as np

PLATES = [
    ("p01_five_discs", os.path.join("selected_plates", "p01_five_discs.png")),
    ("p02_ctx_caz_cla", os.path.join("selected_plates", "p02_ctx_caz_cla.png")),
    ("p03_caz_ctx_cla_pair", os.path.join("selected_plates", "p03_caz_ctx_cla_pair.png")),
    ("p04_cfp_met_ipm_van", os.path.join("selected_plates", "p04_cfp_met_ipm_van.png")),
    ("p05_gen_rif_cot", os.path.join("selected_plates", "p05_gen_rif_cot.png")),
    ("p06_seven_discs", os.path.join("selected_plates", "p06_seven_discs.png")),
    ("p07_mem10_x4", os.path.join("selected_plates", "p07_mem10_x4.png")),
    ("test1", "test1.png"),
]

OUT = "detect_v2_out"
WORK_MAX = 900


def crop_footer(img):
    h, w = img.shape[:2]
    band = 28
    if h <= band + 40:
        return img
    if float(np.mean(img[-band:])) > 220:
        return img[:-band]
    return img


def _resize_max(img, max_side):
    h, w = img.shape[:2]
    s = min(1.0, max_side / float(max(h, w)))
    if s >= 0.999:
        return img, 1.0
    small = cv2.resize(img, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_AREA)
    return small, s


def find_plate(gray):
    """Agar dish, not an inhibition-zone circle."""
    h, w = gray.shape
    cx, cy = w // 2, h // 2
    blur = cv2.GaussianBlur(gray, (9, 9), 2)
    _, bw0 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    candidates = []

    def consider(mask, circ_min, rlo, rhi):
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            peri = cv2.arcLength(cnt, True)
            if peri < 1 or area < 0.08 * h * w:
                continue
            circ = 4 * math.pi * area / (peri * peri)
            if circ < circ_min:
                continue
            (x, y), rad = cv2.minEnclosingCircle(cnt)
            r = int(rad)
            if r < rlo * min(h, w) or r > rhi * min(h, w):
                continue
            dmid = math.hypot(x - cx, y - cy) / max(min(h, w), 1)
            candidates.append((circ * math.sqrt(area) - 90.0 * dmid, int(round(x)), int(round(y)), r))

    consider(bw0 if bw0[cy, cx] else 255 - bw0, 0.65, 0.30, 0.52)
    if not candidates:
        consider(bw0, 0.55, 0.28, 0.56)
        consider(255 - bw0, 0.55, 0.28, 0.56)

    if candidates:
        candidates.sort(reverse=True)
        px, py, pr = candidates[0][1], candidates[0][2], candidates[0][3]
    else:
        min_r = int(min(h, w) * 0.36)
        max_r = int(min(h, w) * 0.52)
        circles = cv2.HoughCircles(
            blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=min(h, w) // 2,
            param1=80, param2=35, minRadius=min_r, maxRadius=max_r,
        )
        px, py, pr = cx, cy, int(min(h, w) * 0.46)
        best_sc = -1e9
        if circles is not None:
            for x, y, r in np.round(circles[0]).astype(int):
                dmid = math.hypot(x - cx, y - cy) / max(min(h, w), 1)
                sc = float(r) - 280.0 * dmid
                if sc > best_sc:
                    best_sc = sc
                    px, py, pr = int(x), int(y), int(r)

    # Inhibition zones sit off-center; the dish does not.
    if math.hypot(px - cx, py - cy) > 0.16 * min(h, w):
        return cx, cy, int(min(h, w) * 0.52)
    return px, py, pr


def _mean_circle(gray, x, y, r, frac_lo=0.0, frac_hi=1.0):
    h, w = gray.shape
    yy, xx = np.ogrid[:h, :w]
    d2 = (xx - x) ** 2 + (yy - y) ** 2
    lo = (r * frac_lo) ** 2
    hi = (r * frac_hi) ** 2
    m = (d2 >= lo) & (d2 <= hi)
    if not np.any(m):
        return 0.0
    return float(gray[m].mean())


def _paper_score(gray, x, y, r):
    """Paper disc: bright core vs agar ring. No global 130 cutoff."""
    core = _mean_circle(gray, x, y, r, 0.0, 0.55)
    ring = _mean_circle(gray, x, y, r, 1.15, 1.55)
    return core, ring, core - ring


def _to_gray(bgr):
    """Yellow discs (FT 300) stay bright; gray() alone can dunk them."""
    b, g, r = cv2.split(bgr)
    return np.maximum(np.maximum(r, g), b)


def _ink_frac(gray, x, y, r):
    h, w = gray.shape
    yy, xx = np.ogrid[:h, :w]
    m = (xx - x) ** 2 + (yy - y) ** 2 <= (0.7 * r) ** 2
    vals = gray[m]
    if vals.size < 20:
        return 0.0
    thr, _ = cv2.threshold(vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return float((vals < thr).mean())


def _ring_edge(gray, x, y, r):
    """Mean gradient on the disc circumference — lawn blobs are weak circles."""
    h, w = gray.shape
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)
    n = max(24, int(2 * math.pi * r))
    acc = 0.0
    ok = 0
    for i in range(n):
        a = 2 * math.pi * i / n
        px = int(round(x + r * math.cos(a)))
        py = int(round(y + r * math.sin(a)))
        if 1 <= px < w - 1 and 1 <= py < h - 1:
            acc += float(mag[py, px])
            ok += 1
    return acc / max(ok, 1)


def _hough(gray, min_r, max_r, min_dist, param2):
    blur = cv2.GaussianBlur(gray, (7, 7), 1.5)
    c = cv2.HoughCircles(
        blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=min_dist,
        param1=70, param2=param2, minRadius=min_r, maxRadius=max_r,
    )
    if c is None:
        return []
    return [(int(x), int(y), int(r)) for x, y, r in np.round(c[0]).astype(int)]


def _nms(cands, frac=1.1):
    """cands: (score, x, y, r, ...). Highest score first."""
    cands = sorted(cands, key=lambda t: -t[0])
    used = []
    out = []
    for t in cands:
        x, y, r = t[1], t[2], t[3]
        if any(math.hypot(x - xx, y - yy) < frac * max(r, rr) for xx, yy, rr in used):
            continue
        used.append((x, y, r))
        out.append(t)
    return out


def detect_discs_v2(image):
    """Return list of (x, y, r) in original image pixels."""
    img = crop_footer(image)
    small, scale = _resize_max(img, WORK_MAX)
    gray = _to_gray(small)
    px, py, pr = find_plate(gray)

    min_r = max(6, int(0.045 * pr))
    max_r = max(min_r + 3, int(0.15 * pr))
    min_dist = max(int(2.4 * min_r), 14)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    hits = _hough(gray, min_r, max_r, min_dist, 18)
    hits += _hough(clahe, min_r, max_r, min_dist, 16)

    def collect(hit_list, dlt_min, edge_min, ink_hi):
        out = []
        for x, y, r in hit_list:
            dist = math.hypot(x - px, y - py)
            if dist > 0.92 * pr:
                continue
            if r < min_r or r > max_r:
                continue
            core, ring, delta = _paper_score(gray, x, y, r)
            if delta < dlt_min or core < 148:
                continue
            ink = _ink_frac(gray, x, y, r)
            if ink < 0.05 or ink > ink_hi:
                continue
            edge = _ring_edge(gray, x, y, r)
            if edge < edge_min:
                continue
            exp_r = 0.065 * pr
            prior = math.exp(-((r - exp_r) / max(0.04 * pr, 1.0)) ** 2)
            out.append(((edge + 2.0 * delta) * prior, x, y, r, delta, ink, edge, core))
        return out

    seeds = _nms(collect(hits, 12, 85, 0.48))
    if not seeds:
        seeds = _nms(collect(hits, 8, 60, 0.50))
    if not seeds:
        return []

    best = seeds[0][0]
    strong = [t for t in seeds if t[0] >= 0.42 * best]
    med = float(np.median([t[3] for t in strong]))

    r_lo = max(min_r, int(0.82 * med))
    r_hi = min(max_r, int(1.18 * med))
    md = max(int(2.0 * med), 10)
    hits2 = _hough(gray, r_lo, r_hi, md, 14) + _hough(clahe, r_lo, r_hi, md, 12)
    merged = _nms(strong + collect(hits2, 6, 55, 0.48))
    merged = [t for t in merged if abs(t[3] - med) <= 0.22 * med]
    if merged:
        best2 = max(t[0] for t in merged)
        merged = [t for t in merged if t[0] >= 0.22 * best2]
        edges = [t[6] for t in merged]
        if len(merged) >= 4 and float(np.median(edges)) > 170:
            cut = 0.55 * float(np.median(edges))
            merged = [t for t in merged if t[6] >= cut]

    merged.sort(key=lambda t: (t[2], t[1]))
    inv = 1.0 / scale
    return [(int(round(t[1] * inv)), int(round(t[2] * inv)), int(round(t[3] * inv))) for t in merged]


def annotate(img, discs):
    out = img.copy()
    for i, (x, y, r) in enumerate(discs, 1):
        cv2.circle(out, (x, y), r, (0, 255, 0), 2)
        cv2.putText(out, str(i), (x - 8, y + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    for stem, path in PLATES:
        img = cv2.imread(path)
        if img is None:
            print(f"missing {path}")
            continue
        discs = detect_discs_v2(img)
        dest = os.path.join(OUT, stem)
        os.makedirs(dest, exist_ok=True)
        vis = annotate(crop_footer(img), discs)
        cv2.imwrite(os.path.join(dest, "_annotated.png"), vis)
        print(f"{stem:24s}  n={len(discs):2d}  " +
              " ".join(f"({x},{y},r={r})" for x, y, r in discs))
    print(f"Wrote {OUT}/")


if __name__ == "__main__":
    main()
