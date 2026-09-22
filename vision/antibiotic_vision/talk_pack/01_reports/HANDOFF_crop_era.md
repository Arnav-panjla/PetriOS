# Antibiotic Disc Text Recognition — Handoff Notes

## Goal
Read the printed code + dose off tiny antibiotic susceptibility disc images
(e.g. "GM 10", "MEM 10", "TZP 110"). Images are ~50-70px square crops of a
round disc, text can be at any rotation, font is an unknown embossed/printed
style. Reference material: Gonzalez & Woods, *Digital Image Processing*
(4th ed.) — the PDF is in this folder (`Image Processing 4ed.pdf`).

## Data
- `discs/disc_01.png` ... `disc_12.png` — the 12 test images.
- Ground truth (read by eye, **not independently verified** — re-check before
  quoting any accuracy number from this in a report):
  `disc_01` IPM 10, `disc_02` CZ 30, `disc_03` MEM 10, `disc_04` SXT,
  `disc_05` CIP 5, `disc_06` GM 10, `disc_07` NN 10, `disc_08` CAZ 30,
  `disc_09` TZP 110, `disc_10` CRO 30, `disc_11` AN 30, `disc_12` AM 10.
- `gm10.png` / `mem10.png` — two earlier single test images (superseded by
  the `discs/` set, kept for history).

## Current best result
**12/12 discs read correctly** via `batch_ocr.py` → `pick_final.py`.
Run: `python batch_ocr.py` then `python pick_final.py` (needs Tesseract
installed for `ocr_read_text.py` only; the main pipeline uses EasyOCR, pure
pip install, no external binary).

---

## Chronological log: what was tried, what happened, why

### 1. Geometric deskew — projection profile / Hough / PCA (`deskew_methods.py`)
Three classic angle estimators, each sweeps/analyzes a rim-filtered binary
mask of the text:
- **A. Projection profile**: rotate through angles, pick the one maximizing
  variance of row-sum of a horizontally-dilated mask.
- **B. Hough transform**: `cv2.HoughLinesP` on mask edges, median angle of
  near-horizontal lines.
- **C. PCA / moments**: principal eigenvector of the foreground-pixel
  covariance matrix.

**Result:** worked on `mem10.png` (all three agreed within ~1° of a manually
verified correct angle). **Failed badly on `gm10.png`** — all three
converged on ≈ -5°, but the true correct angle (found by manually testing
candidates and reading the result) was **-90°**. Cross-checked with a full
±30° visual sweep — the pipeline had locked onto a wrong local optimum.

**Why:** these images are tiny (~65×65px) with only a few dozen foreground
text pixels after masking. Not enough signal for a reliable single-shot
geometric estimate. Confirmed again later (§5) on the full 12-image set with
more rigor: none of these methods, nor variants, reliably find the angle.

### 2. Enhancement methods — Ch.2-4 GW techniques (`text_extraction_methods.py`)
15 standalone (never combined) methods: negative, log transform, gamma,
histogram equalization, contrast stretch, mean/gaussian/median smoothing,
Laplacian sharpening, unsharp masking, Sobel gradient, Otsu global
thresholding, FFT lowpass, FFT highpass, homomorphic filtering. Saved to
`<image>_text_methods/`. This step only makes text *visually* clearer — it
does not read the text into a string.

### 3. OCR — Tesseract, then EasyOCR
- `ocr_read_text.py` (Tesseract, installed via winget/manual installer since
  it wasn't present) — **failed almost completely** on gm10/mem10 crops
  (mostly empty or garbage output). Tesseract is built for page-sized text
  blocks, not few-character tiny stamps.
- `ocr_easyocr.py` / `ocr_cropped.py` (EasyOCR, pure pip install) — much
  better. Once the crop was tightened to just the text bbox (excluding the
  disc rim) and upscaled 10x, `mem10` read correctly (`global_thresholding`
  → 'MEM 10', 98% conf). `gm10` still failed everywhere until its rotation
  was manually corrected to -90° (see §1) — after that fix it also read
  correctly.

### 4. Scaling up to 12 images, naive angle search (`batch_ocr.py`, early versions)
First attempt: search full 360° in coarse+fine steps, score each candidate
angle purely by **OCR confidence** on an Otsu-binarized crop.
**Result: 3/12 correct.** EasyOCR reports high confidence on garbage (e.g.
`conf 1.99` on `'8 8'`), so raw confidence is not a trustworthy objective —
it doesn't distinguish "confidently read a real word" from "confidently read
noise as two digits."

### 5. Geometry-proposes / OCR-decides, single angle (`batch_ocr.py`, v2)
Used projection-profile peaks (see §1) to propose a handful of candidate
angles, then OCR + a **label-format prior** (score boosted if the reading
looks like `LETTERS`+`DIGITS`, penalized if digits-only) to pick among them.
Committing to a **single** winning angle: **9/12**, but which 9 discs
succeeded *changed* between runs with minor tuning (widening the candidate
set moved failures from {02,05,11} to {04,07}) — a strong sign the selection
was brittle, not that the underlying data was unsolvable.

### 6. Consensus voting instead of single-best (`pick_final.py`)
Insight: correct readings recur across different enhancement methods/angles;
garbage readings scatter. Instead of picking the single highest-scoring
reading, let every (method, angle) reading vote, and also **canonicalize**
OCR lookalike-character confusions (`J/I/L → 1`, `O/D/Q → 0`, `S → 5`,
`B → 8`) *inside the numeric dose only*, snapping to the nearest valid
disc-dose from a known set `{1,2,5,10,15,20,23,25,30,50,75,100,110,120,300}`.
**Result on top of the §5 angle search: 10/12**, then with more angle
candidates retried, hit **12/12** — but margins were uneven (one disc won by
only 1.08 vs 10-70+ for the others), flagging it as fragile.

### 7. Root-cause the rotation failures (`analyze_failures.py`, `analyze_tolerance.py`)
Measured, don't guess: for each disc, counted how many of the 15 enhancement
methods succeeded at each shortlisted angle.
**Finding: rotation correctness is a hard gate, not a gradient.** The
correct angle yields several successful methods; angles even 45-90° off
yield **0/15** every time. Then measured the actual tolerance window by
fine-sweeping ±30° around a known-good angle in 3° steps: OCR keeps working
across roughly **±15-20°**, and fails completely outside that. This explains
§5's brittleness — the 45° candidate grid used there has a 22.5° worst-case
error, which is *outside* the measured tolerance.

### 8. Guaranteed-coverage angle search (`batch_ocr.py`, final version)
Replaced the ad-hoc peak-picking with a **15° grid over the full 360°**
(worst-case error 7.5°, inside the measured ±15° tolerance) + a ±10°
refinement pass at 2.5° steps around the winner. Kept the top-2 non-adjacent
candidates (not just 1) into the full 15-method sweep, then consensus-voted.
**Result: 12/12**, and notably **`12_global_thresholding` (Otsu) alone hit
12/12** — it didn't need the ensemble once the angle search was fixed. This
was the biggest single accuracy jump in the whole project and is attributed
entirely to fixing the angle-search grid density, not to any enhancement
method change.

### 9. "Which single method is 100%?" (`analyze_methods.py`)
Direct measurement, run twice (once before, once after the §8 fix):
- **Before the grid fix:** no method reached 12/12 alone; best was
  `unsharp_masking` 9/12 / `contrast_stretching` 10/12 (correct angle
  required for even that ceiling). Smallest consensus-set to reach 12/12 was
  3 methods (5 different valid triples, e.g. contrast_stretch + unsharp_mask
  + global_threshold).
- **After the grid fix:** `12_global_thresholding` alone = 12/12. This is
  the current answer to "which method works for all cases" — but the
  precondition (correct rotation already found) still requires the whole
  search pipeline; Otsu is not a standalone fix for unrotated images.

### 10. Tried to find a better *geometric* angle estimator (failed, 4 variants)
Prompted by "is the disc's circular rim the problem?" Web-searched and found
a real, different classical technique — **gradient-orientation histogram**
(Sun & Si, ICDAR 1997) — which doesn't need character segmentation, unlike
Hough/PCA. Implemented and tested 4 variants against known-good angles:
| variant | accuracy within ±15° |
|---|---|
| raw histogram, whole image | 2/12 |
| + hard rectangular crop to exclude rim | 1/12 (crop boundary itself creates a fake horizontal/vertical edge, worse than the rim) |
| + weighted by the trusted rim-free text mask | 0/12 |
| + 6x upsampling before gradient computation | 1/12 |
Also tried **`cv2.minAreaRect`** (min-area bounding box) directly on the
rim-free mask: **1/12**.
**Conclusion, proven not assumed:** the disc's circular rim was *not* the
real bottleneck — even with it completely removed from the mask, every
classical geometric method still failed. The actual limit is that each label
is only 4-7 characters (~200-400 px), too little shape information for
PCA/Hough/minAreaRect/gradient-histogram to find the *line's* direction
rather than one dominant letter's stroke direction.

### 11. Tried closed-set template matching (failed, but informative)
Read the actual GW PDF (all 1022 pages, keyword-scanned) for anything
missed. Found Ch.12 correlation-based template matching (the book's own
example: matching a known hurricane-eye template against a satellite image).
Reasoned that disc labels are a **closed vocabulary** (~30 known antibiotic
codes × ~15 standard doses) rather than open-ended text, so tried rendering
each candidate label as a text template (PIL, Arial Bold / Calibri Bold, a
few sizes), rotating it through 36 angles, and sliding it across each disc's
Otsu-binarized image with `cv2.matchTemplate` (normalized cross-correlation).
**Result: 1/12**, and it degenerately always matched the shortest word
(`SXT`) regardless of the actual disc.
**Why:** visually compared a rendered template against the real disc — the
actual font is a compressed, rounded, stamped/embossed style, nothing like
Arial Bold. Pixel correlation needs the template to closely match the real
font; without knowing the true font, every template is wrong everywhere, so
the matcher just picks whichever template has the fewest pixels to be wrong
about. This is exactly why trained OCR beats hand-picked templates: OCR
generalizes across thousands of fonts, a single template doesn't generalize
at all. `template_match.py` is left in the repo as a documented negative
result, not deleted.

---

## Current production pipeline (as of this handoff)

1. `batch_ocr.py`:
   - `text_bbox()` — rim-rejecting contour filter (size + solidity + distance
     from disc center) isolates just the text blob.
   - `shortlist_angles()` — 15° grid over 360° (guarantees ≤7.5° worst-case
     error, inside the measured ±15° OCR tolerance) + ±10° refinement at
     2.5° steps around the coarse winner; keeps top-2 non-adjacent angle
     candidates, scored by `label_score()` (OCR confidence weighted by how
     much the reading looks like a real `LETTERS`+`DIGITS` label).
   - For each shortlisted angle, run all 15 methods from
     `text_extraction_methods.py`, OCR (EasyOCR) each result, log everything
     to `results.csv`.
2. `pick_final.py`:
   - `canonical()` — snaps a raw OCR reading to `LETTERS`+`valid dose` when
     one exists, fixing common lookalike confusions.
   - `choose()` — consensus vote (sum of `label_score` per canonical label)
     across every (method, angle) reading for that disc; the label with the
     most vote weight wins. Also reports the vote margin (winner minus
     runner-up) as a rough confidence signal — **not yet wired into an
     automatic accept/reject threshold**, see Open Items.
3. Analysis/report scripts (read `results.csv`, don't re-run OCR):
   `analyze_methods.py` (per-method accuracy, smallest consensus sets),
   `analyze_failures.py` (rotation vs. image-quality failure attribution),
   `analyze_tolerance.py` (measures the ±15° OCR readable window directly).

## Known limitations / open items for the next person
1. **Ground truth is unverified** (read by eye by the assistant). Get an
   independent read before quoting any accuracy % in a report, especially
   given this is a clinical/biological context.
2. **12 images is not a statistically meaningful sample.** 12/12 supports
   roughly "error rate below ~25% at 95% confidence" (rule of three), not
   "high accuracy." Would need hundreds-to-thousands of verified samples to
   claim anything like 99.9%+.
3. **Vote margin as confidence gate is designed but not implemented.**
   `pick_final.py` computes `margin` per disc; add a threshold (discs from
   the current run clear margin ≥ 4.3, disc_09 was previously the fragile
   one at 1.08 with a coarser grid) to auto-flag low-margin results for
   human review instead of silently trusting them. This is the recommended
   next step for reliability, not another accuracy-chasing pass.
4. **The angle search is expensive** (~24 angle probes + 2×15 method/OCR
   calls per disc, all on CPU EasyOCR — a 12-image run takes ~15-30 min).
   If this needs to scale to many more discs, look at batching EasyOCR calls
   or running on GPU.
5. **Runtime cost could likely be cut** by dropping the enhancement methods
   that never contributed (`02_log_transform`, `14_frequency_highpass` were
   0/12 in every run) and possibly relying on `12_global_thresholding` +
   1-2 backups instead of all 15, per the §9 findings — but re-verify this
   on a larger sample before trusting it, since it's only been checked on 12
   images.
6. **Not tried:** per-line separate rotation for multi-line labels was
   attempted early (`rotate_lines_ocr.py`, since deleted) using per-blob PCA
   — failed because PCA on 1-2 character blobs measures the letter's own
   stroke direction, not the line's baseline. Was deleted rather than fixed;
   if multi-line labels become common, revisit with a **centroid-line-fit**
   approach instead (fit a line through *letter centroids*, not raw pixels)
   which wasn't fully tried.
7. **Not tried:** deep-learning scene-text-angle regressors, or an
   OCR engine with built-in orientation detection (Tesseract has `--psm 0`
   OSD mode, untested here — likely too page-oriented for these crops but
   worth a quick check before ruling out).
