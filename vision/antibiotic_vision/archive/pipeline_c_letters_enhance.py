"""
pipeline_c_letters_enhance.py
=============================
Parallel path (no morphology):

  1) C-aligned disc from test1_aligned/
  2) letters-above / digits-below  (0 vs 180)
  3) Gonzalez & Woods 4ed methods on those upright discs
       Ch.3 intensity + spatial filters
       Ch.4 frequency-domain filters
       Ch.9 morphology
       Ch.10 edges / thresholding

No OCR of the label — images only.

Run:  python pipeline_c_letters_enhance.py
Out:  test1_c_letters_above/     upright C discs
      test1_c_letters_enhance/   one image per (disc, method) + contact sheets
"""
import os

import cv2
import numpy as np

from align_binary_discs import ink_mask, list_discs, rotate, sheet
from align_letters_above_digits import flip_to_letters_above, load_c

UPRIGHT_DIR = "test1_c_letters_above"
OUT_DIR = "test1_c_letters_enhance"


# ---------------------------------------------------------------------------
# Ch.3 intensity transformations
# ---------------------------------------------------------------------------
def negative(gray):
    return 255 - gray


def log_transform(gray):
    g = gray.astype(np.float64)
    out = np.log1p(g)
    m = out.max()
    if m > 0:
        out = out * (255.0 / m)
    return out.astype(np.uint8)


def gamma_correct(gray, gamma=0.4):
    g = gray.astype(np.float64) / 255.0
    out = np.power(np.clip(g, 0, 1), gamma) * 255.0
    return np.clip(out, 0, 255).astype(np.uint8)


def contrast_stretch(gray):
    lo, hi = np.percentile(gray, (2, 98))
    if hi <= lo:
        return gray.copy()
    out = (gray.astype(np.float64) - lo) * 255.0 / (hi - lo)
    return np.clip(out, 0, 255).astype(np.uint8)


def histogram_eq(gray):
    return cv2.equalizeHist(gray)


def clahe(gray):
    return cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)


# ---------------------------------------------------------------------------
# Ch.3 spatial smoothing / sharpening
# ---------------------------------------------------------------------------
def box_mean(gray):
    return cv2.blur(gray, (5, 5))


def gaussian_smooth(gray):
    return cv2.GaussianBlur(gray, (5, 5), 1.2)


def median_smooth(gray):
    return cv2.medianBlur(gray, 5)


def laplacian_sharpen(gray):
    # GW: g = f - Laplacian when the kernel centre is negative (OpenCV default)
    lap = cv2.Laplacian(gray, cv2.CV_64F, ksize=3)
    return np.clip(gray.astype(np.float64) - lap, 0, 255).astype(np.uint8)


def unsharp_mask(gray):
    blur = cv2.GaussianBlur(gray, (0, 0), 1.5)
    return cv2.addWeighted(gray, 1.5, blur, -0.5, 0)


def highboost(gray, k=2.0):
    blur = cv2.GaussianBlur(gray, (0, 0), 1.5)
    out = gray.astype(np.float64) * (1.0 + k) - blur.astype(np.float64) * k
    return np.clip(out, 0, 255).astype(np.uint8)


def sobel_edges(gray):
    gx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    mag = np.hypot(gx, gy)
    m = mag.max()
    if m > 0:
        mag = mag * (255.0 / m)
    # black edges on white, same polarity as the stamps
    return (255.0 - mag).astype(np.uint8)


def laplacian_then_stretch(gray):
    """Ch.3 'combining spatial enhancement methods'."""
    return contrast_stretch(laplacian_sharpen(gray))


# ---------------------------------------------------------------------------
# Ch.4 frequency-domain filters
# ---------------------------------------------------------------------------
def _fft_shifted(gray):
    f = np.fft.fft2(gray.astype(np.float64))
    return np.fft.fftshift(f)


def _ifft_abs(Fshift):
    out = np.real(np.fft.ifft2(np.fft.ifftshift(Fshift)))
    out -= out.min()
    m = out.max()
    if m > 0:
        out = out * (255.0 / m)
    return np.clip(out, 0, 255).astype(np.uint8)


def _radius_grid(h, w):
    cy, cx = h / 2.0, w / 2.0
    y, x = np.ogrid[:h, :w]
    return np.sqrt((y - cy) ** 2 + (x - cx) ** 2)


def fft_gaussian_lp(gray, d0_frac=0.08):
    h, w = gray.shape
    d0 = max(4.0, d0_frac * min(h, w))
    d2 = _radius_grid(h, w) ** 2
    H = np.exp(-d2 / (2.0 * d0 * d0))
    return _ifft_abs(_fft_shifted(gray) * H)


def fft_gaussian_hp(gray, d0_frac=0.08):
    h, w = gray.shape
    d0 = max(4.0, d0_frac * min(h, w))
    d2 = _radius_grid(h, w) ** 2
    H = 1.0 - np.exp(-d2 / (2.0 * d0 * d0))
    return _ifft_abs(_fft_shifted(gray) * H)


def fft_butterworth_lp(gray, d0_frac=0.08, n=2):
    h, w = gray.shape
    d0 = max(4.0, d0_frac * min(h, w))
    d = _radius_grid(h, w)
    H = 1.0 / (1.0 + (d / d0) ** (2 * n))
    return _ifft_abs(_fft_shifted(gray) * H)


def homomorphic(gray, d0_frac=0.06, gh=1.8, gl=0.4):
    img = np.log1p(gray.astype(np.float64))
    h, w = img.shape
    d0 = max(4.0, d0_frac * min(h, w))
    d2 = _radius_grid(h, w) ** 2
    H = (gh - gl) * (1.0 - np.exp(-d2 / (2.0 * d0 * d0))) + gl
    F = np.fft.fftshift(np.fft.fft2(img))
    out = np.real(np.fft.ifft2(np.fft.ifftshift(F * H)))
    out = np.expm1(out)
    out = np.clip(out, 0, None)
    m = out.max()
    if m > 0:
        out = out * (255.0 / m)
    return np.clip(out, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Ch.9 morphology (black ink on white)
# ---------------------------------------------------------------------------
def _morph(gray, op, ksize=5, iterations=1):
    ink = ink_mask(gray)
    ker = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
    if op == "erode":
        out_ink = cv2.erode(ink, ker, iterations=iterations)
    elif op == "dilate":
        out_ink = cv2.dilate(ink, ker, iterations=iterations)
    elif op == "open":
        out_ink = cv2.morphologyEx(ink, cv2.MORPH_OPEN, ker, iterations=iterations)
    elif op == "close":
        out_ink = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, ker, iterations=iterations)
    else:
        raise ValueError(op)
    out = np.full_like(gray, 255)
    out[out_ink > 0] = 0
    return out


def morph_erode(gray):
    return _morph(gray, "erode", 5, 1)


def morph_dilate(gray):
    return _morph(gray, "dilate", 5, 1)


def morph_open(gray):
    return _morph(gray, "open", 5, 2)


def morph_close(gray):
    return _morph(gray, "close", 5, 1)


# ---------------------------------------------------------------------------
# Ch.10 edges / thresholding
# ---------------------------------------------------------------------------
def otsu(gray):
    _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return th


def canny_edges(gray):
    edges = cv2.Canny(gray, 60, 160)
    return 255 - edges


METHODS = [
    ("00_original", lambda g: g.copy()),
    ("01_negative", negative),
    ("02_log", log_transform),
    ("03_gamma", gamma_correct),
    ("04_contrast_stretch", contrast_stretch),
    ("05_histogram_eq", histogram_eq),
    ("06_clahe", clahe),
    ("07_box_mean", box_mean),
    ("08_gaussian", gaussian_smooth),
    ("09_median", median_smooth),
    ("10_laplacian", laplacian_sharpen),
    ("11_unsharp", unsharp_mask),
    ("12_highboost", highboost),
    ("13_sobel", sobel_edges),
    ("14_canny", canny_edges),
    ("15_otsu", otsu),
    ("16_fft_lowpass", fft_gaussian_lp),
    ("17_fft_highpass", fft_gaussian_hp),
    ("18_fft_butterworth_lp", fft_butterworth_lp),
    ("19_homomorphic", homomorphic),
    ("20_erode", morph_erode),
    ("21_dilate", morph_dilate),
    ("22_open", morph_open),
    ("23_close", morph_close),
    ("24_laplacian_then_stretch", laplacian_then_stretch),
]


def _all_methods_sheet(stem, results, path, cols=5, cell=140):
    pairs = [(name, img) for name, img in results]
    sheet(pairs, path, cols=cols, cell=cell)


def main():
    os.makedirs(UPRIGHT_DIR, exist_ok=True)
    os.makedirs(OUT_DIR, exist_ok=True)

    files = list_discs()
    if not files:
        raise SystemExit("No disc_XX.png — run extract_binary_discs.py first")

    print("Loading EasyOCR (0/180 letters-above decision only)...")
    import easyocr
    reader = easyocr.Reader(["en"], gpu=False, verbose=False)

    uprights = []
    print("\nC-aligned -> letters above / digits below (no morph)\n")
    for f in files:
        stem = os.path.splitext(f)[0]
        img, src = load_c(stem)
        if img is None:
            print(f"[{stem}] missing C image in test1_aligned/ — skip")
            continue
        upright, ang, score, info, details = flip_to_letters_above(img, reader)
        print(f"[{stem}] {src}")
        for item in details:
            a, s, inf = item[0], item[1], item[2]
            mark = " <--" if a == ang else ""
            print(f"  {a:3d}: score={s:+.2f}  {inf}{mark}")
        print(f"  -> {ang} deg  ({info})\n")
        cv2.imwrite(os.path.join(UPRIGHT_DIR, f"{stem}.png"), upright)
        uprights.append((stem, upright, ang))

    sheet(
        [(s, im) for s, im, _ in uprights],
        os.path.join(UPRIGHT_DIR, "_01_letters_above.png"),
    )

    print(f"Applying {len(METHODS)} Gonzalez-Woods methods...\n")
    method_sheets = {name: [] for name, _ in METHODS}

    for stem, upright, ang in uprights:
        disc_results = []
        for name, fn in METHODS:
            out = fn(upright)
            cv2.imwrite(os.path.join(OUT_DIR, f"{stem}_{name}.png"), out)
            method_sheets[name].append((stem, out))
            disc_results.append((name, out))
        _all_methods_sheet(
            stem, disc_results, os.path.join(OUT_DIR, f"_{stem}_all_methods.png")
        )
        print(f"  {stem}  ({ang} deg)  {len(METHODS)} images")

    for name, _ in METHODS:
        sheet(method_sheets[name], os.path.join(OUT_DIR, f"_{name}.png"))

    print(f"\nUpright discs:  {UPRIGHT_DIR}/_01_letters_above.png")
    print(f"All methods:    {OUT_DIR}/_disc_XX_all_methods.png  (one disc, every method)")
    print(f"Per method:     {OUT_DIR}/_00_original.png ... (all discs, one method)")


if __name__ == "__main__":
    main()
