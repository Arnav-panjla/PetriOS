# BTP-1 process report  
## Reading stamped codes on Kirby–Bauer antibiotic discs

**Student project folder:** `antibiotic_vision`  
**Compiled:** 22 September 2026  
**Period covered:** 2–9 September 2026 (plus one earlier unrecorded session)

This is the **story of the work**: what we tried, why it failed, why we changed direction, and how we arrived at the last working pipeline. A method-by-method catalogue already exists in `BTP_FULL_REPORT.md` and `archive/HANDOFF.md`. Use this document to **present the process**.

---

## How this report was built

Cursor still has **three saved chats** in this folder (plus this one). Many `.py` files and the original `discs/` crops were later deleted. The missing first session (empty folder, Gonzalez & Woods Ch. 1–4, `gem.png` / `mem10.png`) is **not** in Cursor’s transcript store; its results were written into `archive/HANDOFF.md` so the next model could continue.

| Chat | When | What it actually is |
|------|------|---------------------|
| Unsaved first LLM | before 2 Sep | Textbook enhancements only. Results recovered from HANDOFF. |
| [Crop and plate era](b73b33f1-b9a7-4f45-86ca-b711c251d52a) | 2 Sep – 9 Sep | Almost the entire project: OCR, rotation, models, `test1.png`, Gemma, stock plates. |
| [Open-source OCR models](f3bb57e1-e8be-486f-9f57-187d4c6a0dbd) | 4 Sep, short | “Is there a pretrained reader for any orientation?” |
| [Cleanup and enhance](b5327203-6681-41b1-8a50-f0214f299311) | 5 Sep, short | Confirm pipeline; delete unused files; start a parallel letters-then-enhance path. |

**Ground truth was read by eye** in chat. It is **not** independently verified. Do not quote “12/12” as a clinical accuracy claim.

---

## The problem, in one paragraph

A Kirby–Bauer plate is a petri dish with round paper discs. Each disc is stamped with a short **antibiotic code** (letters) and often a **dose in µg** (digits), for example `GM 10`, `MEM 10`, `TZP 110`, or letters-only `SXT`. The stamp is thick, embossed, not Times/Arial. Text can sit at **any rotation**. The deploy target is a **Raspberry Pi 5 (16 GB)**; a heavier model may SSH to an **RTX 3060**. Constraint from the student: **do not train a custom recogniser if off-the-shelf + classical CV can work**, and **do not snap wrong letters to a list of drug names** (that is hardcoding, not reading).

This is **not** page OCR. The useful signal is a few fat glyphs on a circle, often with leftover rim ink.

---

## Two datasets — never mix the scores

There are **two different “12 discs”** with **two different numberings**. Mixing them is how “Otsu is 12/12” later collided with “not a single disc passed.”

### A. Tiny grey crops (`discs/`, `gm10.png`, `mem10.png`) — **deleted from disk**

About 50–70 px. Eye-read labels:

| disc | GT | disc | GT |
|------|-----|------|-----|
| 01 | IPM 10 | 07 | NN 10 |
| 02 | **CZ 30** | 08 | CAZ 30 |
| 03 | MEM 10 | 09 | TZP 110 |
| 04 | SXT | 10 | CRO 30 |
| 05 | CIP 5 | 11 | AN 30 |
| 06 | GM 10 | 12 | AM 10 |

**This is the set where a 15° rotation search + Otsu + EasyOCR hit 12/12.**

### B. Whole plate `test1.png` — **still on disk**

Hough extraction order is **not** the same as set A (`disc_02` here is `TZP 110`, not `CZ 30`).

| disc | GT | disc | GT |
|------|-----|------|-----|
| 01 | IPM 10 | 07 | SXT |
| 02 | **TZP 110** | 08 | NN 10 |
| 03 | GM 10 | 09 | AM 10 |
| 04 | MEM 10 | 10 | CRO 30 |
| 05 | AN 30 | 11 | CZ 30 |
| 06 | CAZ 30 | 12 | CIP 5 |

**This is the set where EasyOCR / Tesseract / Hausdorff failed, and Gemma 3 4B reached 12/12** after upright morph + rim strip.

Scoring rule used later: `canonical()` only to match lookalike **digits** (O/0, I/1 **in the number**). Snapping **codes** to a drug dictionary was **rejected**.

---

## Phase 0 — Textbook pictures, no reader  
*(unrecorded first chat + start of 2 Sep)*

**Ask:** implement Gonzalez & Woods Chapters 1–4, **one method at a time, never stacked**, on `gem.png` / `mem10.png`, save each result as an image.

**What we did:** invert, log, gamma, histogram equalisation, contrast stretch, mean / Gaussian / median blur, Laplacian / unsharp, Sobel, Otsu, FFT low-pass / high-pass, homomorphic filter.

**Why we moved on:** these files only change how the photo **looks**. They do not output the string `GM 10`. The student asked: *“I can only see images, where is the text result?”*

---

## Phase 1 — “Just OCR it”  
*(2 Sep afternoon)*

**Tesseract** (classic page OCR). Failed: empty or garbage. It expects paragraphs of printed type, not a 60 px stamp.

**EasyOCR** (neural scene OCR: CRAFT + CRNN). Better, but the **circular rim** looks like extra strokes, and text is often **tilted**. Engines expect roughly **horizontal** characters.

Tight crop around letters + ~10× upscale: `mem10` read `MEM 10` on the Otsu image. `gm10` still failed — it was rotated about **90°**. Cropping does not fix orientation.

**Manual −90° on `gm10`:** EasyOCR then worked. That proved the real bug was **angle**, not “EasyOCR is useless.” Manual rotation is not a method.

---

## Phase 2 — One-shot geometry to find the angle  
*(2 Sep)*

Tried the usual textbook deskew ideas on a rim-filtered binary mask:

| Method | Idea | Result |
|--------|------|--------|
| Projection profile | Spin; pick angle where row-sums of ink are peaky | Worked on `mem10`. On `gm10` locked near **−5°**; truth was **−90°**. |
| Hough lines | Median angle of short edges | Same failure: letter strokes are not a baseline. |
| PCA / moments | Longest axis of ink pixels | Longest axis is often **one fat stroke** (Z diagonal, T stem), not the line of words. |
| Per-blob PCA (`rotate_lines_ocr.py`) | Straighten each letter blob | A blob is one character; PCA measures the **letter’s** shape. Deleted. |

**Why they fail (kept as a thesis negative):** each label is only 4–7 characters (~200–400 ink pixels). There is not enough shape to estimate **line** direction. Later we proved the **rim was not the cause**: even on a circle-free mask, gradient-orientation histogram (Sun & Si, 1997) was **0–2/12** within ±15°, and `minAreaRect` was **1/12**.

---

## Phase 3 — Brute-force rotation + EasyOCR on 12 crops  
*(2 Sep evening)*

Student added 12 crop images. Pipeline: spin, OCR, pick a winner.

| Attempt | Rule | Score |
|---------|------|-------|
| Spin 360°, keep **highest EasyOCR confidence** | Trust the engine’s number | **3/12** |
| Geometry proposes a few angles; prefer readings that look like `LETTERS` + digits | Format prior | **9/12**, but **which 9 changed** when the candidate list was tweaked |
| Consensus vote across (enhancement × angle); dose lookalikes only (`J/I/L→1`, `O/D/Q→0`), snap dose to known µg values — **not** drug names | `pick_final.py` | **10/12**, then **12/12** with more angles — one disc won by margin **1.08** (fragile) |

**Why confidence failed:** EasyOCR can be **very sure** of `'8 8'` (noise). Confidence is not “this is a real disc label.”

**Measurement that changed the project:** at the **correct** angle, several of the 15 enhancers OCR correctly. **45–90° off: 0/15 every time.** Rotation is a **hard gate**, not a gentle slope. Fine sweep: OCR stays alive about **±15–20°** around truth. A 45° candidate grid can miss by **22.5°**, which is **outside** that window. That is why 9/12 was brittle.

**Fix:** **15° grid over 360°** (worst miss 7.5°, inside the window) + ±10° refine at 2.5° + keep two distant candidates + 15 methods + vote.

**Result on set A only: 12/12.** After this grid, **Otsu alone was 12/12**. Log transform and FFT high-pass were **0/12 always**. The jump was the **grid density**, not a new filter. Otsu still does **not** find the angle; it only reads once the crop is already upright.

Student then asked for 99.999% accuracy because this is biology. Honest answer: **n = 12** is not that. Rule of three: 0 errors in 12 supports “error rate probably below ~25%,” not 99.9%.

---

## Phase 4 — Templates and “is there a last resort in the book?”  
*(2 Sep night – 3 Sep morning)*

Gonzalez Ch. 12 correlation: render every candidate label in Arial/Calibri, rotate, `matchTemplate`. **1/12**, always **`SXT`**. Real stamps are round and fat; Arial is the wrong font. The **shortest** word has the fewest pixels that can mismatch, so it wins.

Tesseract OSD (`--psm 0`) and neural scene-text angle regressors: **never run**.

HANDOFF was written so the next model would not repeat this loop.

---

## Phase 5 — “Find something Sonnet did not try”  
*(3 Sep midday)*

Student conclusion: Otsu works **if** orientation is correct; we still lack a reliable **one-shot** rotator. Two forks: (1) a method that does not need rotation, or (2) better orientation then reuse OCR.

Looked at **PARSeq**, **PaddleOCR**, **TrOCR / ABINet / SATRN / SVTR**, **YOLO**, **Qwen2.5-VL**, specialist OCR-VLMs. None was finished as a bake-off in remaining files. Python 3.14 blocked several installs. Scene-text nets still want a **horizontal word crop**. YOLO draws boxes; it does not output `IPM 10`. Training our own net was **not done** (student: no custom training). Deploy: Pi 5 16 GB, optional RTX 3060. **Gemma 3 4B** was flagged as a local VLM option.

**Student ran the old 15° + Otsu recipe on `discs_dataset/clear/`:** **0/12**. That folder is **not** the original `discs/` set (other plates, extra crop, different lighting). This does **not** prove the crop-era 12/12 was a lie. It **does** prove that recipe is **not universal**. Confusion in chat came from mixing the two folders.

---

## Phase 6 — Whole plate: find the discs, then upright them  
*(4 Sep)*

New plan from a recovered Otsu crop: from `test1.png`, cut discs, paint agar white, **black letters on white**. No OCR yet.

**Hough circles → crop → upscale → white outside circle → Otsu** (`extract_binary_discs.py`). Worked on this photo. `minRadius` / `maxRadius` are **tuned to this plate**.

Rim: shrinking the keep-circle too far **cut letters**. Mild inset + wipe outer specks still left **arcs** (GM, AN, SXT) that later fooled Gemma (`GMI`).

**Alignment bake-off A–H** on binary discs. Winner by eye: **C — two-line k-means centroids** (code row vs dose row; rotate so the line between centres is vertical). Still **180° ambiguous**. PCA, projection, letter-box packing, longest black runs lost to C. Re-running C on fused `ME` (MEM) **drifts the angle** — later rule: **load the saved C PNG, never recompute k-means**.

Thinning / skeleton: student said it **looked bad**. Dropped.

EasyOCR on C at 0/90/180/270: **6/12**. Old 15-method vote on these 400 px **1-bit** stamps: **4/12**. Making the image “clearer” **hurt** classical OCR. The crop-era recipe was for **small grey** photos.

---

## Phase 7 — Morphology, then “letters above digits”  
*(4 Sep evening – 5 Sep morning)*

Gemini suggestion: morphological **open** to separate touching thick strokes. Strong kernels destroy letters. Compromise: **open 5×5, 2 iterations** (`_04_open_5x5_x2`).

Four-way upright on morph **failed** (and one run **recomputed C**, so the image was not the C the student had inspected).

Web / observation: when a dose exists, **letters on top, digits below** is how these discs are printed. After C, rows are horizontal; EasyOCR counts letters vs digits; prefer letters on **top**. Letter-only (`SXT`): do not flip on junk. This **fixed 0 vs 180**. Student caught a loader bug: letters-above had been applied to **raw C** instead of the morph image. **Order locked in:** saved C → open 5×5 ×2 → letters-above.

Stamps now **look obvious to a human**. Then:

| Reader on upright 1-bit stamps | Score |
|--------------------------------|-------|
| EasyOCR whole disc | 5/12 |
| Tesseract per row | 2/12 |
| Tesseract whole disc | 5/12 |
| RapidOCR | 4/12 |
| EasyOCR per row | 0/12 |
| EasyOCR on original grey (not binary) | 4/12 |
| Tesseract on grey | 0/12 |
| Snap codes to drug list | **rejected** (hardcoding) |
| Hausdorff to Hershey A–Z / 0–9 | **1/12** (`CIP 5` only); distances looked “good” while labels were `IPH 78` |

**Why this felt absurd:** the letters are obvious to us. CRNN / Tesseract were trained on **thin printed fonts**, not fat 1-bit stamps. Humans use shape + layout; those engines do not.

---

## Phase 8 — Gemma 3 4B becomes the reader  
*(5 Sep)*

Student pasted Gemma 3 specs (SigLIP, 896×896, pan-and-scan). Local **Ollama `gemma3:4b`**, **no antibiotic dictionary**, layout hint only.

| Gemma input | Score | What went wrong |
|-------------|-------|-----------------|
| 896×896 nearest-neighbour zoom | **10/12** | `TZP` → **`ZTP`** (order mix from zoom / pan-and-scan). Isolated glyphs still T then Z. Native crop later said `TZP 110`. |
| Native upright morph | **11/12** | TZP correct. `GM` → **`GMI 10`** because a **rim arc** looked like `I`. |
| Native + strip leftover rim blobs (keep letter-sized seeds even if they **do not touch**; wipe ink that never touches that zone) | **12/12** | 0 letter pixels lost on the check. SXT dashes stay (too close); fake `SXT 120` dropped if the second “row” is junk specks. |

Gemma runs **offline after** `ollama pull gemma3:4b` (localhost, no internet at inference).

**Last working reader on `test1.png` (re-run 7 Sep still 12/12):**

```
test1.png
  extract_binary_discs.py         → binary discs
  align_binary_discs.py           → saved C (two-line) PNG
  morph_stroke_separate.py        → open 5×5 ×2
  align_letters_above_digits.py   → 0° vs 180°
  ocr_gemma3.py                   → strip rim, Gemma 3 4B, parse fake dose
                                  → test1_ocr_gemma3/results.csv
```

Do **not** recompute C. EasyOCR is only used for the 0/180 layout check, not as the final reader.

Per-disc on this plate:

| Disc | Hausdorff | Gemma final | Truth |
|------|-----------|-------------|--------|
| 01 | IPH 78 | IPM 10 | IPM 10 |
| 02 | TZP 7117 | TZP 110 | TZP 110 |
| 03 | WA 78 | GM 10 | GM 10 |
| 04 | WEW 78 | MEM 10 | MEM 10 |
| 05 | W 0 | AN 30 | AN 30 |
| 06 | W 0 | CAZ 30 | CAZ 30 |
| 07 | GIWFQ 807 | SXT | SXT |
| 08 | HH 15 | NN 10 | NN 10 |
| 09 | W 75 | AM 10 | AM 10 |
| 10 | CW 0 | CRO 30 | CRO 30 |
| 11 | IIIIQI 0 | CZ 30 | CZ 30 |
| 12 | CIP 5 | CIP 5 | CIP 5 |

---

## Phase 9 — Serial Gonzalez–Woods pipelines P0–P12  
*(6–7 Sep)*

Student wanted **pure image processing** (no Gemma at first), methods **in series** (not 25 filters in parallel), morph **last**. Built P0 (shared prefix) then tails P1–P12.

**P0:** plate → Hough → Otsu → **saved C** → letters-above → rim strip.  
Then extra median / Gaussian / unsharp / CLAHE / close / erode / open combinations.

**EasyOCR on the same 13 galleries:** best **5/12**. Never read MEM, AN, CAZ, AM, CRO. Always got CIP 5 and CZ 30.

**Gemma on the same galleries:** **10/12 on every tail.** Same two misses: SXT invented a dose; CRO already **erased** by rim-strip because letters-above ran on **raw C** (no morph first). Extra enhance steps **did not help**. That 2-disc gap vs the 12/12 reader is **loader + parser**, not “need more filters.”

Student reminder: **before** these pipelines, Gemma on the morph-first path was already 12/12. P0–P12 are a negative result for “stack more GW ops instead of the working order.”

---

## Phase 10 — Other plates (watermarked stock photos)  
*(7 Sep)*

Same C39 pipeline on unique plates in `selected_plates/` (duplicates and colony-crowded dishes dropped; watermark **not** inpainted — it sits on stamps).

**Detection (Hough), not OCR** — corrected 7 Sep: **33 real discs**, **25 proper circles**, **8 missed**, **14 false** (rim / table specks). `test2` (seven discs) was **0 proper / 8 false / 7 missed**. Detector radii do not transfer.

Gemma on all 39 crops often said `TAB` / `AM 500` on watermarks and false circles. On a **hand-filtered “good only”** subset, still **not** 12/12 (e.g. `VA 30` and `MET 5` and `MEM 10` hit; `E 15` → `AP`, `CTX 30` → `CTX 33`). The gap is **detector + watermark + domain shift**, not “Gemma forgot `test1`.”

A separate detector script (`detect_discs_v2.py`) was started so the original Hough code would stay untouched.

---

## What actually worked (say this out loud)

1. **Finding discs** on a clean plate photo: Hough circles + Otsu. Fragile across cameras.  
2. **Making rows horizontal:** method C (two-line centroids), **frozen PNG**.  
3. **Upright vs upside-down:** letters above, digits below, **after** morph open 5×5 ×2.  
4. **Reading tiny grey crops (deleted set):** dense **15° search + Otsu + EasyOCR**. Did **not** transfer to 1-bit plate stamps.  
5. **Reading 1-bit plate stamps on `test1.png`:** **local Gemma 3 4B** at **native** size, **rim blobs stripped**, fake dose dropped when the extra row is specks.

Classical one-shot deskew **cannot** be trusted on 5–7 character stamps. Classical OCR **cannot** be trusted on these thick binary glyphs. A VLM still **hallucinates** if you over-zoom or leave a rim that looks like a letter.

---

## Constraints we refused to break

- No antibiotic-**name** dictionary as the reader.  
- Do not re-run k-means for C.  
- Order: C → morph → letters-above, not letters-above on raw C.  
- Prefer not to train a custom net for the BTP “method.”

---

## Open items (honest)

- There is still **no single `run_plate.py`** from an arbitrary petri photo.  
- Hough must be **retuned** per plate; stock plates showed miss/false-circle rates.  
- Independent ground truth; **n = 12 is not 99.9%**.  
- SXT dashed rim; Hausdorff is optional and weak.  
- Pi 5 latency not measured here; Gemma 12B on 3060 if 4B fails new plates.  
- Crop-era vote-margin reject gate was never wired to Gemma.  
- First empty-folder chat is **lost**; cite HANDOFF for that slice.

---

## One paragraph for a talk or thesis

We started from Gonzalez & Woods pixel operations on two disc crops. Those only made pictures, so we added OCR. Tesseract failed; EasyOCR worked only after the stamp was **cropped and upright**. One-shot geometry (PCA, Hough, projection, templates, gradient histograms) **cannot** find the baseline of a 5–7 character stamp. A **dense 15° search** plus Otsu/EasyOCR **did** read a **pre-cropped grey** set of 12, but that recipe **did not** transfer to Hough-extracted **1-bit** stamps from a full plate. On `test1.png` we split the job: CV **finds and uprights** (Hough, method C, morph, letters-above); Tesseract, EasyOCR, RapidOCR, and Hausdorff still failed on the thick binary font. **Local Gemma 3 4B** read all 12 after we stopped 896 nearest-neighbour zoom (which swapped T/Z) and **deleted leftover rim blobs without cropping letters**. That 12/12 is **one plate**, **offline after model download**, and **not** a single-click product from an arbitrary dish photograph.
