"""
align_binary_discs.py
=====================
Align black-on-white binary discs from test1_binary_discs/.

Four independent angle estimators:

  A. Projection profile  - maximize row-sum variance over angle sweep
  B. PCA / moments       - principal axis of black ink pixels
  C. Two-line centroids  - k-means into 2 clusters; vector between centres
                           is perpendicular to the text baseline
  D. Letter-box packing  - estimate avg letter size from blobs; find angle
                           where those letter-sized boxes line up in a row
  E. Centroid baseline   - PCA on *letter centres* only (not every ink pixel),
                           so individual stroke directions don't dominate
  F. Horizontal runs     - maximize total length of horizontal black runs
                           (letter strokes become long when text is flat)
  G. Dual-row fit        - same two-line idea as C, but score every angle:
                           two tight horizontal rows, well separated vertically
  H. Size-matched blobs  - eye prior: letter/word height ≈ 0.18 of disc;
                           keep only black blobs whose *width* is near that
                           size (letters ≈ square; short words ≈ 2–3×), then
                           two-line align on those blobs only

No OCR. Saves rotated images + a contact sheet per method.

Run:  python align_binary_discs.py
Out:  test1_aligned/
"""
import math
import os
import re

import cv2
import numpy as np

SRC_DIR = "test1_binary_discs"
OUT_DIR = "test1_aligned"
# Coarse then fine: 15 deg is enough to get near; refine for a cleaner horizontal.
COARSE_STEP = 5
FINE_SPAN = 5
FINE_STEP = 0.5

# From looking at test1_binary_discs (~400px): letters and words share about the
# same height — roughly 1/5 of the disc (~70–80px). Scale with image size.
LETTER_HEIGHT_FRAC = 0.18


def list_discs():
    files = []
    for f in sorted(os.listdir(SRC_DIR)):
        if re.fullmatch(r"disc_\d+\.png", f):
            files.append(f)
    return files


def ink_mask(gray):
    """Black letters -> 255, white bg -> 0."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    return (gray < 128).astype(np.uint8) * 255


def strip_disc_rim(gray):
    """
    Paint leftover disc-edge arcs white. Never erase a blob that touches
    the letter cluster (so stamps are not cropped or nicked).
    """
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    ink = (gray < 128).astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(ink, 8)
    if n <= 1:
        return gray

    img_area = h * w
    cx, cy = w / 2.0, h / 2.0
    r_img = min(h, w) / 2.0
    seed = np.zeros(ink.shape, np.uint8)
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        bw = int(stats[i, cv2.CC_STAT_WIDTH])
        bh = int(stats[i, cv2.CC_STAT_HEIGHT])
        if area < 50 or area > img_area * 0.12:
            continue
        if bh > 0.40 * h or bw > 0.55 * w:
            continue
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        dist = float(np.hypot(cents[i][0] - cx, cents[i][1] - cy))
        # only the far edge: a '1' is skinny but sits with the stamp, not here
        if aspect > 3.5 and dist > 0.50 * r_img:
            continue
        seed[labels == i] = 1

    if int(seed.sum()) == 0:
        return gray

    protect = cv2.dilate(
        seed, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    )
    out = gray.copy()
    for i in range(1, n):
        if int((protect[labels == i] > 0).sum()) == 0:
            out[labels == i] = 255
    return out


def rotate(img, angle):
    h, w = img.shape[:2]
    M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
    border = 255 if img.ndim == 2 or (img.ndim == 3 and img.mean() > 127) else 0
    return cv2.warpAffine(
        img, M, (w, h), flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=border,
    )


def projection_score(mask, angle):
    rot = rotate(mask, angle)
    # Dilate a little so a text line merges into a horizontal band
    ker = cv2.getStructuringElement(cv2.MORPH_RECT, (max(3, mask.shape[1] // 20), 1))
    smeared = cv2.dilate(rot, ker)
    row_sums = smeared.sum(axis=1).astype(np.float64)
    return float(row_sums.var())


def method_projection(mask):
    """Angle that makes horizontal text-line bands peaky."""
    best_a, best_s = 0.0, -1.0
    for a in np.arange(0, 180, COARSE_STEP):  # 180: projection is 180-periodic
        s = projection_score(mask, float(a))
        if s > best_s:
            best_s, best_a = s, float(a)
    for a in np.arange(best_a - FINE_SPAN, best_a + FINE_SPAN + 1e-6, FINE_STEP):
        a = a % 180
        s = projection_score(mask, float(a))
        if s > best_s:
            best_s, best_a = s, float(a)
    return best_a


def method_pca(mask):
    """Principal axis of ink; return angle that makes that axis horizontal."""
    ys, xs = np.where(mask > 0)
    if len(xs) < 20:
        return 0.0
    pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
    pts -= pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts, full_matrices=False)
    vx, vy = vt[0]
    # angle of principal axis; we want it horizontal -> rotate by -angle
    ang = math.degrees(math.atan2(vy, vx))
    # snap to [-90, 90) then to [0, 180) for consistency with projection
    return ang % 180


def method_two_line_centroids(mask):
    """
    Split ink into 2 clusters (code line vs dose line).
    Vector between centroids is ~perpendicular to baseline when both lines exist.
    """
    ys, xs = np.where(mask > 0)
    if len(xs) < 40:
        return method_pca(mask)

    data = np.column_stack([xs, ys]).astype(np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)
    compactness = None
    best_centers = None
    # try a few random inits
    for _ in range(5):
        _, _, centers = cv2.kmeans(
            data, 2, None, criteria, 3, cv2.KMEANS_PP_CENTERS
        )
        # compactness proxy: separation of centers
        sep = float(np.linalg.norm(centers[0] - centers[1]))
        if compactness is None or sep > compactness:
            compactness, best_centers = sep, centers

    c0, c1 = best_centers
    dx, dy = float(c1[0] - c0[0]), float(c1[1] - c0[1])
    # vector between lines is perpendicular to baseline
    # baseline angle = atan2(dy, dx) + 90
    line_ang = math.degrees(math.atan2(dy, dx)) + 90.0
    return line_ang % 180


def _letter_blobs(mask):
    """Connected black components that look like letters (not rim arcs)."""
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    h, w = mask.shape
    img_area = h * w
    cx0, cy0 = w / 2, h / 2
    blobs = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        bw = stats[i, cv2.CC_STAT_WIDTH]
        bh = stats[i, cv2.CC_STAT_HEIGHT]
        if area < img_area * 0.001 or area > img_area * 0.12:
            continue
        if bw < 3 or bh < 3:
            continue
        # rim arcs: long thin, far from centre
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        dist = math.hypot(cents[i][0] - cx0, cents[i][1] - cy0)
        if aspect > 4.0 and dist > min(h, w) * 0.28:
            continue
        blobs.append({
            "x": float(stats[i, cv2.CC_STAT_LEFT]),
            "y": float(stats[i, cv2.CC_STAT_TOP]),
            "w": float(bw),
            "h": float(bh),
            "cx": float(cents[i][0]),
            "cy": float(cents[i][1]),
            "area": float(area),
        })
    return blobs


def _letter_box_score(mask, letter_w, letter_h):
    """
    After a candidate rotation: do letter blobs form 1-2 horizontal rows
    where spacing along x is multiples of letter_w?
    """
    blobs = _letter_blobs(mask)
    if len(blobs) < 2:
        return -1e9

    # cluster into rows by cy (gap ~ half a letter height)
    blobs = sorted(blobs, key=lambda b: b["cy"])
    rows = [[blobs[0]]]
    for b in blobs[1:]:
        if abs(b["cy"] - rows[-1][-1]["cy"]) <= letter_h * 0.65:
            rows[-1].append(b)
        else:
            rows.append([b])

    # prefer 1 or 2 rows (code / dose)
    if not (1 <= len(rows) <= 3):
        return -1e9

    score = 0.0
    for row in rows:
        if len(row) < 1:
            continue
        row = sorted(row, key=lambda b: b["cx"])
        # row should be flat
        ys = [b["cy"] for b in row]
        score -= 0.5 * float(np.std(ys)) / max(letter_h, 1)

        # consecutive gaps should be ~ k * letter_w (k = 0.5..1.5 typically)
        for a, b in zip(row, row[1:]):
            gap = b["cx"] - a["cx"]
            # how many letter-widths apart (centres)
            units = gap / max(letter_w, 1)
            nearest = round(units)
            if nearest < 1:
                nearest = 1
            err = abs(units - nearest)
            score += max(0.0, 1.5 - err)  # reward packing on a letter grid

        # row width should be ~ n_letters * letter_w
        span = (row[-1]["cx"] + row[-1]["w"] / 2) - (row[0]["cx"] - row[0]["w"] / 2)
        expected = len(row) * letter_w
        score += max(0.0, 2.0 - abs(span - expected) / max(letter_w, 1))

        score += 0.3 * len(row)  # more letters in a clean row = better

    # bonus for exactly 2 rows (typical disc label)
    if len(rows) == 2:
        score += 1.0
    return score


def method_letter_boxes(mask):
    """
    1) Measure average letter box from current blobs.
    2) Sweep angles; keep the one where letter-sized boxes line up in rows.
    """
    blobs = _letter_blobs(mask)
    if len(blobs) < 2:
        return method_projection(mask)

    letter_w = float(np.median([b["w"] for b in blobs]))
    letter_h = float(np.median([b["h"] for b in blobs]))
    # letters are roughly square-ish; if one side is weird, blend
    if letter_w < 3 or letter_h < 3:
        return method_projection(mask)

    best_a, best_s = 0.0, -1e18
    for a in np.arange(0, 180, COARSE_STEP):
        rot = rotate(mask, float(a))
        s = _letter_box_score(rot, letter_w, letter_h)
        if s > best_s:
            best_s, best_a = s, float(a)

    for a in np.arange(best_a - FINE_SPAN, best_a + FINE_SPAN + 1e-6, FINE_STEP):
        a = float(a % 180)
        rot = rotate(mask, a)
        s = _letter_box_score(rot, letter_w, letter_h)
        if s > best_s:
            best_s, best_a = s, a
    return best_a


def method_centroid_baseline(mask):
    """
    PCA on letter *centroids* only. One point per letter, so a tall 'I' or
    wide 'M' stroke cannot pull the principal axis off the text line.
    """
    blobs = _letter_blobs(mask)
    if len(blobs) < 3:
        return method_pca(mask)
    pts = np.array([[b["cx"], b["cy"]] for b in blobs], dtype=np.float64)
    pts -= pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts, full_matrices=False)
    vx, vy = vt[0]
    return math.degrees(math.atan2(vy, vx)) % 180


def method_horizontal_runs(mask):
    """
    At the right angle, black letter strokes form long horizontal runs.
    Score = sum of (run_length^2) over horizontal runs — peaks when flat.
    """
    best_a, best_s = 0.0, -1.0
    for a in np.arange(0, 180, COARSE_STEP):
        rot = rotate(mask, float(a))
        s = _run_score(rot)
        if s > best_s:
            best_s, best_a = s, float(a)
    for a in np.arange(best_a - FINE_SPAN, best_a + FINE_SPAN + 1e-6, FINE_STEP):
        a = float(a % 180)
        s = _run_score(rotate(mask, a))
        if s > best_s:
            best_s, best_a = s, a
    return best_a


def _run_score(mask):
    score = 0.0
    for row in mask:
        run = 0
        for v in row:
            if v:
                run += 1
            elif run:
                score += run * run
                run = 0
        if run:
            score += run * run
    return score


def method_dual_row_fit(mask):
    """
    Like C, but instead of one k-means shot, sweep angles and score:
      - letter blobs form exactly 2 horizontal rows
      - small y-spread inside each row
      - large gap between the two row means
    This is C's geometry with A's search discipline.
    """
    blobs0 = _letter_blobs(mask)
    if len(blobs0) < 3:
        return method_two_line_centroids(mask)

    letter_h = float(np.median([b["h"] for b in blobs0]))
    best_a, best_s = 0.0, -1e18

    for a in np.arange(0, 180, COARSE_STEP):
        s = _dual_row_score(rotate(mask, float(a)), letter_h)
        if s > best_s:
            best_s, best_a = s, float(a)
    for a in np.arange(best_a - FINE_SPAN, best_a + FINE_SPAN + 1e-6, FINE_STEP):
        a = float(a % 180)
        s = _dual_row_score(rotate(mask, a), letter_h)
        if s > best_s:
            best_s, best_a = s, a
    return best_a


def _dual_row_score(mask, letter_h):
    blobs = _letter_blobs(mask)
    if len(blobs) < 3:
        return -1e9
    blobs = sorted(blobs, key=lambda b: b["cy"])
    rows = [[blobs[0]]]
    for b in blobs[1:]:
        if abs(b["cy"] - rows[-1][-1]["cy"]) <= letter_h * 0.7:
            rows[-1].append(b)
        else:
            rows.append([b])
    if len(rows) != 2:
        # soft penalty: 1 or 3 rows still usable but worse
        if len(rows) < 1 or len(rows) > 3:
            return -1e9
        penalty = 3.0
    else:
        penalty = 0.0

    score = -penalty
    means = []
    for row in rows:
        ys = np.array([b["cy"] for b in row], dtype=np.float64)
        xs = np.array([b["cx"] for b in row], dtype=np.float64)
        means.append(ys.mean())
        score -= float(ys.std())  # tight rows
        score += 0.5 * len(row)
        # row should be wide (letters spread horizontally, not stacked)
        if len(row) >= 2:
            score += float(xs.max() - xs.min()) / max(letter_h, 1)

    if len(means) >= 2:
        # want clear vertical separation between lines
        score += abs(means[0] - means[-1]) / max(letter_h, 1)
    return score


def _size_matched_blobs(mask):
    """
    Keep black blobs whose width is near the observed letter/word height.
    (Letters are roughly square; short words are ~2–3 letter-widths wide,
    same height. Rim arcs are long/thin or huge — dropped.)
    """
    h, w = mask.shape
    target = LETTER_HEIGHT_FRAC * min(h, w)  # ~72 on a 400px disc
    n, _, stats, cents = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        bw = int(stats[i, cv2.CC_STAT_WIDTH])
        bh = int(stats[i, cv2.CC_STAT_HEIGHT])
        if area < 200:
            continue
        # height must also look letter-like (words share letter height)
        if not (0.55 * target <= bh <= 1.7 * target):
            continue
        # width ≈ 1 letter, or ≈ 2–3 letters (a short word blob)
        letter_ok = 0.55 * target <= bw <= 1.45 * target
        word_ok = 1.4 * target <= bw <= 3.6 * target
        if not (letter_ok or word_ok):
            continue
        aspect = max(bw, bh) / max(min(bw, bh), 1)
        if aspect > 4.0:
            continue
        blobs.append({
            "cx": float(cents[i][0]),
            "cy": float(cents[i][1]),
            "w": float(bw),
            "h": float(bh),
            "area": float(area),
        })
    return blobs, target


def method_size_matched(mask):
    """
    H: filter to letter/word-sized blobs (width ~ eye-measured height),
    then two-line centroid angle on those centres only.
    """
    blobs, _ = _size_matched_blobs(mask)
    if len(blobs) < 2:
        return method_two_line_centroids(mask)

    pts = np.array([[b["cx"], b["cy"]] for b in blobs], dtype=np.float32)
    if len(pts) == 2:
        dx = float(pts[1, 0] - pts[0, 0])
        dy = float(pts[1, 1] - pts[0, 1])
        return (math.degrees(math.atan2(dy, dx)) + 90.0) % 180

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.5)
    best_sep, best_centers = -1.0, None
    for _ in range(5):
        _, _, centers = cv2.kmeans(
            pts, 2, None, criteria, 3, cv2.KMEANS_PP_CENTERS
        )
        sep = float(np.linalg.norm(centers[0] - centers[1]))
        if sep > best_sep:
            best_sep, best_centers = sep, centers

    c0, c1 = best_centers
    dx, dy = float(c1[0] - c0[0]), float(c1[1] - c0[1])
    return (math.degrees(math.atan2(dy, dx)) + 90.0) % 180


def apply_and_pick_flip(img, angle_180):
    """
    Projection/PCA give angle mod 180. Try angle and angle+180; keep the one
    whose ink mass sits higher (heuristic: code line usually above dose, but
    mostly we just prefer ink in the upper half as a stable default).
    Returns (rotated_image, final_angle).
    """
    candidates = [angle_180 % 360, (angle_180 + 180) % 360]
    best_img, best_a, best_score = None, candidates[0], -1e9
    for a in candidates:
        rot = rotate(img, a)
        m = ink_mask(rot)
        ys, xs = np.where(m > 0)
        if len(ys) == 0:
            score = -1e9
        else:
            # prefer ink concentrated in fewer horizontal bands (already
            # horizontal) AND slightly prefer centroid in upper half
            row = m.sum(axis=1).astype(np.float64)
            score = float(row.var()) - 0.01 * abs(ys.mean() - m.shape[0] * 0.4)
        if score > best_score:
            best_score, best_img, best_a = score, rot, a
    return best_img, best_a


def sheet(pairs, path, cols=4, cell=160):
    if not pairs:
        return
    rows = (len(pairs) + cols - 1) // cols
    canvas = np.full((rows * (cell + 28), cols * cell, 3), 40, np.uint8)
    for i, (name, img) in enumerate(pairs):
        r, c = divmod(i, cols)
        g = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        big = cv2.resize(g, (cell - 8, cell - 8), interpolation=cv2.INTER_AREA)
        big = cv2.cvtColor(big, cv2.COLOR_GRAY2BGR)
        oy, ox = r * (cell + 28) + 4, c * cell + 4
        canvas[oy:oy + big.shape[0], ox:ox + big.shape[1]] = big
        cv2.putText(canvas, name, (ox, oy + big.shape[0] + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(path, canvas)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    methods = [
        ("A_projection", method_projection),
        ("B_pca", method_pca),
        ("C_two_line", method_two_line_centroids),
        ("D_letter_boxes", method_letter_boxes),
        ("E_centroid_baseline", method_centroid_baseline),
        ("F_horizontal_runs", method_horizontal_runs),
        ("G_dual_row_fit", method_dual_row_fit),
        ("H_size_matched", method_size_matched),
    ]
    sheets = {name: [] for name, _ in methods}
    sheets["original"] = []

    files = list_discs()
    if not files:
        raise SystemExit(f"No disc_XX.png in {SRC_DIR}")

    print(f"Aligning {len(files)} discs from {SRC_DIR}/")
    for f in files:
        path = os.path.join(SRC_DIR, f)
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        mask = ink_mask(img)
        stem = os.path.splitext(f)[0]
        sheets["original"].append((stem, img))

        print(f"\n[{stem}]")
        for mname, fn in methods:
            ang180 = fn(mask)
            aligned, ang = apply_and_pick_flip(img, ang180)
            out = os.path.join(OUT_DIR, f"{stem}_{mname}_{ang:.1f}.png")
            cv2.imwrite(out, aligned)
            sheets[mname].append((f"{stem} {ang:.0f}", aligned))
            print(f"  {mname:12s}  angle={ang:+.1f} deg  -> {os.path.basename(out)}")

    sheet(sheets["original"], os.path.join(OUT_DIR, "_00_original.png"))
    for mname, _ in methods:
        sheet(sheets[mname], os.path.join(OUT_DIR, f"_{mname}.png"))

    print(f"\nDone. Compare contact sheets in {OUT_DIR}/")
    print("  _00_original.png")
    for mname, _ in methods:
        print(f"  _{mname}.png")


if __name__ == "__main__":
    main()
