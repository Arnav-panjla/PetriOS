# Antibiotic Disc Label Reading — Full Experiment Report

**Project:** BTP 1, Kirby–Bauer disc stamp reading (`CODE` + optional dose)  
**Folder:** `antibiotic_vision`  
**This version:** 6 September 2026 (plus serial enhance pipelines P0–P12; gallery measured on 1-bit stamps)

**How this was compiled.** The empty-folder “first LLM” chat is **not** in Cursor’s saved transcripts for this project. Crop-era detail comes from `HANDOFF.md` (written so the next model could continue) plus the chat that starts 2 Sep 2026. Plate-era detail comes from remaining code, CSVs, and later chats. Early `.py` files were **deleted from the folder**; scores below are historical, not re-run today.

**How to read scores.** There are **two** 12/12 results on **two different inputs**. Mixing them is how “Otsu is 12/12” collided with “not a single disc passed” on another folder.

---

## 1. Problem

Read the stamped antibiotic **code** (letters) and **dose** (digits, µg) from disc photos. Text can be at any angle. Font is thick, stamped, not Times/Arial. Target deploy: Raspberry Pi 5 16 GB; heavy models via SSH to RTX 3060.

Not page OCR. Short two-line (or letters-only) text on a circle, plus leftover rim ink.

---

## 2. Two datasets — never mix accuracy

### 2.1 Small crops (`discs/`, `gm10.png`, `mem10.png`) — **gone from disk**

~50–70 px. Eye-read GT in `HANDOFF.md` (not independently verified):

| disc | GT | disc | GT |
|------|-----|------|-----|
| 01 | IPM 10 | 07 | NN 10 |
| 02 | CZ 30 | 08 | CAZ 30 |
| 03 | MEM 10 | 09 | TZP 110 |
| 04 | SXT | 10 | CRO 30 |
| 05 | CIP 5 | 11 | AN 30 |
| 06 | GM 10 | 12 | AM 10 |

**This is the set where 15° search + Otsu + EasyOCR hit 12/12.**

### 2.2 Whole plate `test1.png` — **still on disk**

Hough numbering is **not** the same as §2.1 (`disc_02` here is `TZP 110`, not `CZ 30`).

| file | GT | file | GT |
|------|-----|------|-----|
| disc_01 | IPM 10 | disc_07 | SXT |
| disc_02 | TZP 110 | disc_08 | NN 10 |
| disc_03 | GM 10 | disc_09 | AM 10 |
| disc_04 | MEM 10 | disc_10 | CRO 30 |
| disc_05 | AN 30 | disc_11 | CZ 30 |
| disc_06 | CAZ 30 | disc_12 | CIP 5 |

**This is the set where EasyOCR/Tesseract/Hausdorff failed and Gemma 3 4B reached 12/12** after upright morph + rim strip.

Also on disk: `test.jpg`, `test2.png`, `test4.png`–`test9.png`, `discs_dataset/labels.csv` (clear vs marginal). `test3.png` is missing.

Scoring on the plate used `canonical()` **only to match** (dose O/0, I/1 **in the number only**). Snapping **codes** to a drug list (`IPA`→`IPM`) was **rejected** as hardcoding. Layout prior (letters above digits) was allowed.

---

## 3. Constraints (user)

- Prefer not to train a custom recogniser if off-the-shelf + CV can work.  
- **No antibiotic-name dictionary** for the reader.  
- After method C is chosen: **load saved C PNGs**, do not re-run k-means (drift, especially fused `ME` on MEM).  
- Order: **C → open 5×5 ×2 → then letters-above**, not letters-above on raw C.  
- Dev: CPU Iris Xe ~16 GB, Python 3.14 (`transformers` / Paddle often unavailable).  
- Reference: Gonzalez & Woods DIP 4ed PDF in the folder.

---

## 4. Every method we tried — what it is, then why it failed (or worked)

Format for each item: **what it is** (plain language), then **why it failed**. If it actually helped, that is said instead of a fail reason.

---

### 4.1 Crop era (`gm10`, `mem10`, `discs/`)

#### A1. Fifteen Gonzalez & Woods enhancements (never combined)

**What it is.** Textbook pixel tricks, each run **alone** and saved as pictures: invert colours; log/gamma stretch; histogram equalisation; contrast stretch; blur (mean / Gaussian / median); sharpen (Laplacian, unsharp); Sobel edges; Otsu threshold (make it strictly black/white); FFT low-pass / high-pass; homomorphic filter. No two tricks stacked.

**Why it failed as a reader.** These only change how the photo *looks*. They do not output `GM 10`. You still need something that maps shapes to letters. (Later, **Otsu** helped EasyOCR *after* the crop was rotated — see A15–A16 — but A1 by itself produced folders of images, not labels.)

#### A2. Tesseract

**What it is.** The classic open-source page OCR engine (LSTM). You give it a bitmap; it prints text.

**Why it failed.** It is trained and tuned for **printed pages** (paragraphs, even lighting, thin fonts). A 60 px disc stamp is a few fat blobs. Output was empty or garbage.

#### A3. EasyOCR on the whole disc (rim included)

**What it is.** A neural OCR (CRAFT finds text, CRNN reads it). Stronger than Tesseract on photos.

**Why it failed.** The **circular rim** looks like extra strokes, and the stamp is often **tilted**. The recogniser expects roughly **horizontal** characters.

#### A4. EasyOCR after a tight box around the letters + upscale (~10×)

**What it is.** Same engine, but first cut away the disc edge and blow the letters up so they have more pixels.

**Why it partly worked / partly failed.** `mem10` read `MEM 10` on the Otsu version. `gm10` still failed because it was still **rotated about 90°** — cropping does not fix orientation.

#### A5. Manually rotate `gm10` by −90°, then EasyOCR

**What it is.** A human picked the angle; then the same OCR as A4.

**Why this is not a method — and what it proved.** It is not automatic. It **worked** on that one image and proved the real bug was **angle**, not “EasyOCR is useless.”

#### A6. Projection-profile deskew

**What it is.** Spin the binary mask through angles. At each angle, sum blackness along each row. When text is horizontal, those row-sums are peaky (some rows full of ink, some empty). Pick the angle with maximum variance.

**Why it failed.** On `mem10` it agreed with other methods. On `gm10` it locked near **−5°** while the truth was **−90°**. With only dozens of ink pixels, the score has a **wrong local peak**. It does not “see” words; it sees a lumpy histogram.

#### A7. Hough line deskew

**What it is.** Detect short straight edges (`HoughLinesP`), take the median angle, rotate so those lines are horizontal.

**Why it failed.** Letter strokes are short, curved, thick. There are not enough **baseline** lines. The median angle follows random stroke bits, same failure mode as A6 on `gm10`.

#### A8. PCA / moments on every black pixel

**What it is.** Treat ink pixels as a cloud. The longest axis of that cloud is assumed to be the text direction; rotate it to horizontal.

**Why it failed.** On a stamp, the longest axis is often **one fat letter stroke** (e.g. the diagonal of Z, the stem of T), not the line of words. Later confirmed on 12 crops: still unreliable.

#### A9. PCA on each connected letter, rotate each line (`rotate_lines_ocr.py`)

**What it is.** Split ink into blobs, PCA each blob, try to straighten each row separately.

**Why it failed.** A blob is often **one character**. PCA of an `I` or `S` is the letter’s own shape, not “the line goes this way.” Script was deleted.

#### A10. Spin 360°, OCR each step, keep the angle with **highest EasyOCR confidence**

**What it is.** Brute-force rotation. Trust the engine’s confidence number as “this angle is best.”

**Why it failed.** **3/12.** EasyOCR can be **very confident** of `'8 8'` (noise). Confidence is not “this is a real disc label.”

#### A11. Geometry suggests a few angles; pick the one whose OCR looks like `LETTERS` + digits

**What it is.** Don’t search all 360°. Use projection peaks as candidates. Prefer readings that look like a code plus a dose, not digits-only garbage.

**Why it failed as a finished system.** **9/12**, but **which 9** changed when you tweaked the candidate list. A coarse (e.g. 45°) grid can miss the true angle by up to **22.5°**, and OCR only works inside about **±15–20°** (A14). So the pipeline was **brittle**, not “9 discs are unreadable.”

#### A12. Consensus voting + dose lookalikes (`pick_final.py`)

**What it is.** Run many (enhancement × angle) OCRs. The **same** correct string tends to reappear; garbage strings are all different. Sum scores per canonical label. In the **dose only**, map I/J/L→1, O/D/Q→0, etc., and snap to standard µg values. **Not** a list of drug names.

**Why it did not fail — with a caveat.** Reached **12/12** on `discs/` once angles were dense enough (A15). One disc won by a **tiny margin (1.08)** — fragile. This is a **picker**, not a rotator.

#### A13. Count successful methods vs angle (`analyze_failures.py`)

**What it is.** For each disc and each tried angle, how many of the 15 enhancers OCR correctly?

**Why we ran it (measurement, not a reader).** At the **right** angle, several methods work. **45–90° off: 0/15.** Rotation is a hard gate.

#### A14. Fine angle sweep around a known-good angle (`analyze_tolerance.py`)

**What it is.** From a correct angle, step ±30° in 3° steps and see when OCR dies.

**Result, not a failure.** Window ≈ **±15–20°**. Outside: dead. This is **why A11 was brittle**.

#### A15. 15° grid over 360° + fine refine + top-2 angles + 15 methods + vote

**What it is.** Sample every 15° (worst miss **7.5°**, inside the ±15° window), refine, keep two distant candidates, OCR all enhancers, vote as A12.

**Why it worked (on `discs/` only).** The search **cannot skip** the readable window. **12/12.** The jump was the **grid**, not a new filter. **Does not** automatically work on other folders (see B8) or on later 1-bit plate stamps (see C16).

#### A16. “Which single enhancer is 100%?” (`analyze_methods.py`)

**What it is.** After OCR logs exist, score each of the 15 methods alone.

**What we learned.** Before dense angles: no method 12/12 (unsharp 9/12, contrast 10/12). After A15: **Otsu alone 12/12**. Log transform and FFT high-pass were **0/12 always**.

**Why Otsu is not “the whole solution.”** It only wins when the crop is **already upright**. It does not find the angle.

#### A17. Gradient-orientation histogram (Sun & Si, 1997)

**What it is.** Compute edge gradients, histogram of their directions. Text lines should create a peak; rotate to that peak. Variants: whole image; rectangular crop; weight by text mask; 6× upsample.

**Why it failed.** Scores **0–2/12** within ±15°. A hard crop **adds fake horizontal/vertical edges**. Even with the rim removed: **0/12**. Too few characters to estimate **line** direction.

#### A18. Minimum-area rectangle (`cv2.minAreaRect`) on the text mask

**What it is.** Fit the smallest rotated box around all ink; use the box angle.

**Why it failed.** **1/12.** The box follows the **blob cloud** (often one letter’s extent), not the baseline.

#### A19. Template matching (Gonzalez Ch.12)

**What it is.** Draw every candidate label in Arial/Calibri, rotate it, slide it over the Otsu disc, keep the best correlation.

**Why it failed.** **1/12**, always **`SXT`**. Real stamps are round, fat, embossed. Arial is the wrong font. The **shortest** word has the fewest pixels that can mismatch, so it wins.

#### A20. Tesseract orientation detection (`--psm 0`)

**What it is.** Tesseract’s own “which way is up” mode for documents.

**Why it failed.** **Never run.** Left as an open check. Expected to be page-oriented, same family as A2.

#### A21. Neural scene-text angle regressor

**What it is.** A network trained to output rotation of street-sign text.

**Why it failed.** **Never run.** Would need integration and likely a GPU; not used.

**Crop-era summary.** Automatic **one-shot deskew failed**. **Dense spin + EasyOCR + Otsu** worked on the **small `discs/` set**. Do not quote that 12/12 for the full plate.

---

### 4.2 Models we looked at (mostly not fully tested)

#### B1. PARSeq / parseq_tiny

**What it is.** A scene-text **recogniser**: one cropped word in, string out. Paper talks about wild / oriented text, but in practice you still usually **warp the word upright** first. Tiny weights via torch.hub. Discs have **two lines**, so you must split rows.

**Why it failed (for us).** Never finished as a bake-off in the remaining files. User did not want to train. Two-line + Python 3.14 made a clean test awkward. It was never shown to replace angle search on these stamps.

#### B2. PaddleOCR (PP-OCRv4 / v5)

**What it is.** Full Chinese/English OCR kit: detect box, optional **0° vs 180°** line flip, optional **0/90/180/270** page orientation, then recognise.

**Why it failed (for us).** It does **not** solve 37° or 15° continuously. Install on **Python 3.14** was a blocker. Never became the plate reader.

#### B3. TrOCR, ABINet, SATRN, SVTR, CRNN

**What it is.** Neural “image of a word → letters” models. CRNN is what EasyOCR already uses inside.

**Why they failed as a magic fix.** They all want a **horizontal crop**. Swapping CRNN for TrOCR does not remove the rotation problem. Not deployed as a completed bake-off here.

#### B4. YOLO

**What it is.** Object detector: draws boxes around things it was trained on (people, cars, or custom classes).

**Why it failed as a reader.** A box around a disc is not the string `IPM 10`. You would still need OCR/VLM inside the box. Not used as the text engine.

#### B5. Train our own scene-text model

**What it is.** Collect thousands of labelled stamps, train a net that maps pixels → `CODE DOSE`, maybe with random rotations in training.

**Why it was not done.** User preferred pretrained tools. Needs a large labelled set. Contradicts “don’t train” for the BTP method. So it neither failed nor succeeded — it was **not tried**.

#### B6. Qwen2.5-VL (vision-language model)

**What it is.** A multimodal LLM: image + “what does this say?” → text. Can read odd layouts without an explicit 15° loop. 7B is large.

**Why it was not the final reader.** Needs GPU or a heavy Ollama setup. Aimed as a **helper / GT tool**, not Pi-first. Gemma 3 4B was what we actually ran locally.

#### B7. HunyuanOCR, Unlimited-OCR, DeepSeek-OCR

**What it is.** Specialist “OCR VLMs”: feed the photo, get transcription.

**Why they failed (for us).** Listed, **never installed**. GPU-first; not the Pi path we implemented.

#### B8. Same 15° + Otsu + OCR, but on `discs_dataset/clear/` (you ran it)

**What it is.** A15’s recipe on a **different** folder (other plates, extra cropping, “clear” subset).

**Why it failed.** You reported **zero** correct. That folder is **not** the original 12 `discs/` files. Extra crop / other lighting / other stamps. This does **not** prove A15 was a lie on `discs/`. It **does** prove that recipe is **not universal**.

**B-era summary.** No small downloadable model is “disc at any angle.” We did not complete PARSeq/Paddle bake-offs. The working local reader later was **Gemma 3**, after CV prep.

---

### 4.3 Whole plate `test1.png`

#### C1. `antibiotic_ocr.py` through v10

**What it is.** Repeated attempts to **find discs on a full plate photo** (colour, agar, many circles).

**Why early versions failed.** Missed discs, counted agar holes, duplicates, clips at the image edge. The idea survived as Hough + filters (C2).

#### C2. Hough circles → crop → upscale → white outside the circle → Otsu

**What it is.** Detect round discs, cut each out, enlarge, paint agar white, threshold so letters are black on white.

**Why it worked (on test1).** Circles match discs. **Limitation, not a read-failure:** `minRadius`/`maxRadius` are **tuned to this photo**. Other plates need new numbers. This step does **not** read text.

#### C3. Shrink the keep-circle a lot to kill the rim

**What it is.** Only keep the inner part of the disc so the dark ring disappears.

**Why it failed.** Letters sit near the edge. Too much shrink **cuts the stamps**. You said stop.

#### C4. Mild inset (0.94× radius) + delete tiny blobs in the outer band

**What it is.** Slightly smaller circle, plus wipe specks hugging the cut.

**Why it only partly worked.** Better than C3, but **arcs remain** after rotation/morph (GM, AN, SXT). Those arcs later fooled Gemma (`GMI`).

#### C5. Align A — projection profile (same idea as A6, on binary discs)

**What it is.** Spin; pick angle where horizontal ink bands are strongest.

**Why it lost to C.** Same local-optima problem. Visually worse than two-line (C7) on this plate.

#### C6. Align B — PCA on all ink (same idea as A8)

**What it is.** Longest axis of black pixels → horizontal.

**Why it failed vs C.** Fat strokes (M, Z) dominate. Rows did not look as clean.

#### C7. Align C — two-line k-means centroids

**What it is.** Split ink into **two clusters** (usually code row vs dose row). The line between cluster centres is roughly **perpendicular** to the text. Rotate so that line is vertical → rows are horizontal. Still **180° ambiguous** (upside-down vs upright).

**Why it worked.** Best visual deskew on test1. **Why it can fail if misused:** running k-means again **changes the angle** (fused `ME` on MEM). **Must load the saved C image.**

#### C8. Align D — letter-box packing

**What it is.** Guess typical letter size, find boxes of that size, rotate until they sit in a row.

**Why it failed vs C.** Fused/touching letters make box sizes wrong. Noisier packing.

#### C9. Align E — PCA on letter **centres** only

**What it is.** Like B, but each letter is one point (its centre), so one stroke cannot dominate.

**Why it lost to C.** Cleaner than B in theory; still not as stable as two-line centroids on these discs.

#### C10. Align F — longest horizontal black runs

**What it is.** When letters are upright, horizontal parts of E, T, Z become long black runs. Maximise that.

**Why it failed vs C.** Mixed. Thick stamps and rims also create long runs.

#### C11. Align G — score every angle for two tight rows

**What it is.** Like C, but try many angles and pick the one where two rows are compact and separated.

**Why it failed vs C.** Extra complexity; visually C still won.

#### C12. Align H — keep blobs whose width ≈ 0.18 of disc height

**What it is.** Your idea: letters have a typical height; ignore huge rim arcs; then two-line align on the rest.

**Why it failed vs C.** Mixed; MEM still awkward. Rim vs letter size overlap.

#### C13. Recomputing C on MEM (`ME` fused)

**What it is.** Same C algorithm, new k-means run.

**Why it failed.** The fused blob moves the clusters; angle **drifts**. Looks like “C is random.” **Fix:** freeze the PNG you already liked.

#### C14. Thinning / skeleton (“a thin line inside each letter”)

**What it is.** Morphology that eats the letter down to a 1-pixel centreline, hoping OCR likes thin fonts.

**Why it failed.** You said it **looked bad**. Centres are broken, spiky, not like printed type. Dropped.

#### C15. EasyOCR on C at 0°, 90°, 180°, 270°

**What it is.** Only four spins; keep the best reading.

**Why it failed.** **6/12.** Missing 15°–75°. Also 1-bit **thick** letters are a bad font for CRNN.

#### C16. Old crop recipe: 15 GW methods + vote, but on **400 px binary** stamps

**What it is.** A12/A15 on the new discs.

**Why it failed.** Consensus **4/12**. The 12/12 recipe was for **small gray crops**, not these stamps. Processing “clearer” 1-bit images **hurt** classical OCR.

#### C17. Morphological erode / open (several kernel sizes)

**What it is.** Erode = peel black pixels. Open = erode then dilate (knock off thin bridges, keep fat letters). Tried 5×5/7×7/9×9.

**Why strong versions failed.** They **destroy** letters. Tiny kernels did nothing. **Open 5×5, 2 iterations** was the visual compromise (`_04_open_5x5_x2`). Morph **does not read** text; it only separates strokes.

#### C18. Four-way upright (0/90/180/270 + regex) on morph

**What it is.** EasyOCR plus rules like “letters then digits” to pick a cardinal orientation.

**Why it failed.** You said it **did not work**. That folder **recomputed C** (different image than the C you inspected). Also leftover ~90° is not solved by 180° flip.

#### C19. Letters above, digits below (0° vs 180° only)

**What it is.** After C, rows are horizontal. Split two rows. EasyOCR counts letters vs digits. Prefer letters on **top**. If there is no dose (`SXT`), do not flip on junk.

**Why it worked (as orientation, not as OCR).** Matches how these discs are printed. **Fails** if C left the text nearly vertical, or if OCR mis-counts rows.

#### C20. Letters-above applied to **raw C** instead of morph

**What it is.** Same as C19 but the input was the wrong picture.

**Why it failed.** You asked for morph **then** flip. The contact sheet looked like C, not `_04_open_5x5_x2`. **Fixed** by changing the loader.

#### C21. EasyOCR on the upright binary disc

**What it is.** Neural OCR on the stamp that looks obvious to you.

**Why it failed.** **5/12.** Thick 1-bit `M`/`H`/`W`, `I`/`1`, `O`/`0`, fused letters, rim junk. Humans use shape + context; CRNN was trained on different fonts.

#### C22. Tesseract, one crop per text row

**What it is.** Page OCR on a thin strip (the top row, then the bottom row).

**Why it failed.** **2/12.** Worse strips, still the wrong font/size for Tesseract.

#### C23. Tesseract on the whole upright disc

**What it is.** One Tesseract call on the full stamp.

**Why it failed.** **5/12** — best of that bake-off, still not usable.

#### C24. RapidOCR

**What it is.** Fast ONNX OCR (similar family to Paddle/CRNN).

**Why it failed.** **4/12.** Same 1-bit stamp problem.

#### C25. EasyOCR separately on each row

**What it is.** Cut top row, OCR; cut bottom row, OCR.

**Why it failed.** **0/12.** Row images are tiny/odd; the engine got nothing useful.

#### C26. Put the aligned **original gray** photo back (not binary)

**What it is.** Maybe OCR likes real shading better than 1-bit.

**Why it failed.** EasyOCR **4/12**, Tesseract **0/12**. Did not beat binary+Gemma.

#### C27. Snap OCR output to a list of antibiotic names

**What it is.** If the engine says `IPA`, replace with `IPM` because that drug exists.

**Why it failed (for the project).** You called it **hardcoding**. It hides errors and is not a reader.

#### C28. Hausdorff distance to A–Z / 0–9 templates

**What it is.** Cut each blob, compare its shape to drawn Hershey letters with a point-set distance. Letters on top row, digits on bottom.

**Why it failed.** **1/12** (`CIP 5`). OpenCV fonts are not the stamp font. Distance looked “good” (~1.0) while the label was wrong (`IPH 78`). Same lesson as A19.

#### C29. Gemma 3 4B on a **896×896 nearest-neighbour** zoom of the stamp

**What it is.** Local vision LLM (SigLIP + Gemma). Tight crop, blow up to the encoder size with jagged pixels.

**Why it failed as that input.** **10/12.** `TZP` became **`ZTP`** (order mix from zoom/pan-and-scan, not because the stamp was ZTP). `SXT` got a fake dose from rim dashes.

#### C30. If the “dose row” is tiny specks, drop the digits Gemma invented

**What it is.** Layout rule: those specks are rim, not `100`.

**Why a loose version failed.** It also turned `GM10` into `GM` (thought there was no dose row). Tightened: only drop dose when the extra row is **junk-sized**.

#### C31. Gemma on the **native** upright morph (no 896 zoom)

**What it is.** Same model, same disc file humans look at.

**Why it still failed on one disc.** **11/12.** TZP **correct**. `GM` → **`GMI 10`** because a **rim arc** was still in the picture and looked like `I`.

#### C32. Cut a smaller circle to remove the rim

**What it is.** Same as C3 after Gemma failed.

**Why it failed.** The GM arc reaches **inward**. A circle small enough to remove it can **hit letters**.

#### C33. Delete any blob whose centre is far from the disc centre

**What it is.** Assume letters sit in the middle; far blobs are rim.

**Why it failed.** **Ate real letters.** `T` and `P` in TZP, `C` in CRO sit **off-centre**. Output like `Z` + `11`.

#### C34. Keep every letter-sized blob (they need not touch); delete blobs that never touch that group

**What it is.** Mark stamps as “letters.” Slightly expand that region. Paint white any ink that never touches it.

**Why this did not fail as the rim cleaner.** Removed the GM arc and AN’s `)`. **0 letter pixels lost** on the check. Gemma then **12/12**. SXT dashes stay (too close); fake `SXT 120` still handled by C30.

#### C35. 25 Gonzalez–Woods methods in **parallel** on upright C discs (no morph)

**What it is.** After C + letters-above (not after morph), each textbook trick was saved as its own picture (`pipeline_c_letters_enhance.py` → `test1_c_letters_enhance/`). Intensity maps, blurs, sharpeners, FFT, Sobel/Canny, Otsu, four morph ops.

**Why most of them failed as a stack.** The input is already **1-bit**. On all 12 discs, log / gamma / contrast stretch / hist-eq / Laplacian / unsharp / highboost / Otsu / Laplacian-then-stretch were **pixel-identical** to the original (mean MSE = 0). Negative inverts polarity (OCR wants dark on light). Sobel/Canny turn filled letters into outlines (ink area ~1–2%). Homomorphic paints almost the whole disc black. FFT high-pass wrecks polarity. Closing and dilating **glue** TZP/MEM. The only operators that actually change a 1-bit stamp in a useful way: **median**, **Gaussian** (then Otsu again), **opening**.

#### C36. Serial tails after a shared prefix (P0–P4), images only

**What it is.** Do **not** run the 25 methods in parallel. Shared prefix **P0**: `test1.png` → Hough crop → Otsu → **saved C** → letters-above 0/180 → rim strip. Then short tails: P1 open; P2 median→open; P3 median→Gaussian→Otsu; P4 median→Gaussian→Otsu→open. Script: `compare_enhance_pipelines.py`. Pictures: `test1_pipeline_bakeoff/`.

**Gemma bake-off (P0–P4 only, old parse).** First run used Gemma, then user asked for EasyOCR on **all** P0–P12, then Gemma again on **all** P0–P12 with the **same** EasyOCR parser (`parse_disc_text`). Scores: §7 table and C38.

#### C37. Longer tails P5–P12 (5 or 6 steps after P0)

**What it is.** Same P0 prefix. Extra combinations: unsharp or highboost after Gaussian; CLAHE; box mean; Laplacian+stretch; close-then-open-then-erode; open-then-close; Gauss–median–Gauss. Morph stays at or near the **end**.

**Why extra steps mostly failed.** Unsharp/highboost/CLAHE/Laplacian barely change 1-bit stamps (P10 identical to P4 on 10/12 discs). Close after open re-fuses. Erode (P7/P11/P12) thins letters and EasyOCR drops.

#### C38. EasyOCR vs Gemma 3 4B on the **same** P0–P12 images

**What it is.** `ocr_pipeline_bakeoff.py` (EasyOCR) and `ocr_pipeline_gemma.py` (local Gemma). Same 12×13 pictures in `test1_pipeline_bakeoff/`. Same `canonical()` match. CSVs: `easyocr_results.csv`, `gemma_results.csv`.

**EasyOCR result.** Best **5/12** (P2, P4, P6, P8, P9, P10). P3 **2/12**. Nobody got MEM, AN, CAZ, AM, or CRO. CIP 5 and CZ 30 were correct on every pipeline.

**Gemma result.** **10/12 on every tail** (P0–P12). Same two misses: SXT fake dose, CRO gone from the image. Extra enhance steps did not help Gemma on this plate.

#### C39. Why bake-off Gemma was 10/12 but the real reader is **12/12**

**What it is.** The 12/12 run is **not** P0–P12. It is: saved C → **open 5×5 ×2** → letters-above **on that morph** (`test1_letters_above/disc_XX_out_*.png`) → rim strip → Gemma native → `parse_label` (drop a fake dose if the second row is junk specks). Re-run 7 Sep 2026: **12/12** again (`ocr_gemma3.py`).

**Why P0–P12 could not hit 12/12.** That bake-off did letters-above on **raw C** (no morph first). Rim strip then **deleted `CRO`** on disc_10 (letters were not in the seed). Parser was EasyOCR’s `parse_disc_text`, which **kept** Gemma’s fake `SXT 25`. Those two bugs are the whole 2-disc gap. The P tails cannot restore letters that rim strip already erased.

**100% on this plate.** Use the C39 path, not the P0–P12 gallery. That is 12/12 on **one** plate, not a clinical guarantee.

#### C40. Stock photos in `images/` (watermarked, duplicates)

**What we kept.** Unique plates only in `selected_plates/`. Exact copies dropped: `image (14)`=`(13)`, `(16)`=`(4)`. Low-res Shutterstock twins of `test4`–`test9` dropped. Crowded colony plates (`image (7)(8)(10)(11)(12)(15)`) skipped — Hough treats colonies as discs. Watermark **not** inpainted (it sits on stamps; inpaint would eat letters). Footer ID bar cropped when it was a white strip.

**Pipeline.** Same as C39. Script: `run_stock_plates.py`. **39** crops. Gemma often said `TAB`/`AM 500` on watermarked/false circles. Real-looking hits: `VA 30`, `MET 5`, `CTX 33`, `CAZ 100`, `MEM 10`. This is **not** 12/12. Detector + watermark are the gap, not “Gemma forgot how to read test1.”

**Plate-era summary.** CV **finds and uprights** the stamp. Classical OCR **cannot read** these 1-bit letters. **Gemma 3 4B** can, if you **don’t over-zoom** and **don’t leave a rim that looks like a letter**. Serial GW stacking is for **cleaning the bitmap**, not for reading it.


---

## 5. What actually worked (short)

| Layer | What works | On which data |
|-------|------------|----------------|
| Find discs | Hough + Otsu binary disc | test1.png |
| Make rows horizontal | Method C, **saved** PNG | test1 binary discs |
| 0 vs 180 | Letters-above on **morph open 5×5 ×2** | test1 |
| Read letters | **Not** Tesseract/EasyOCR/RapidOCR/Hausdorff on these 1-bit stamps | test1 |
| Read letters | **Local Gemma 3 4B**, native resolution, rim blobs stripped | test1 **12/12** |
| Read letters | EasyOCR + **15° search** + Otsu | old **`discs/` crops only** |
| Offline Gemma | After `ollama pull gemma3:4b` | localhost, no internet |

---

## 6. Gemma 3 — bugs that looked like “the model is dumb”

**TZP vs ZTP.** Blobs left-to-right were T, Z, P. Isolated glyphs: T and Z. Hausdorff kept order `TZP`.

| Input | Gemma |
|-------|--------|
| 896 nearest | `ZTP 110` |
| 896 bilinear | `ZLP 110` |
| Top row only | `TZP` |
| Native disc | `TZP 110` |

**GM vs GMI.** Extra **rim arc**, not a misread G. After stripping the arc: `GM 10`.

**SXT.** Gemma may add `100`/`120` from dashed rim. Dose dropped if the extra “row” is junk specks.

**Letters need not touch.** Each of C, I, P, 5 can be its own seed. Wipe only ink that **never** touches the letter zone.

---

## 7. Current runnable path (test1)

Not one command. Do not recompute C.

```
test1.png
  extract_binary_discs.py       → test1_binary_discs/
  align_binary_discs.py         → test1_aligned/*_C_two_line_*.png
  morph_stroke_separate.py      → test1_morph/*_04_open_5x5_x2.png
  align_letters_above_digits.py → test1_letters_above/*_out_0|180.png
  ocr_hausdorff_glyphs.py       → optional, 1/12
  ocr_gemma3.py                 → strip rim, Ollama gemma3:4b → test1_ocr_gemma3/results.csv
```

Needs Ollama + pulled weights. EasyOCR only for 0/180.

Parallel gallery (not the scorer): `pipeline_c_letters_enhance.py` — C → letters-above **without** morph → 25 GW methods, images only (`test1_c_letters_enhance/`).

Serial tails (no OCR): `compare_enhance_pipelines.py` → `test1_pipeline_bakeoff/`.

**P0 prefix (every P):** `test1.png` → cut discs (Hough) → Otsu → saved C rotate → letters on top (0°/180°) → remove rim.

| ID | Extra steps after P0 | How many extra |
|----|----------------------|----------------|
| P0 | (none) | 0 |
| P1 | open 5×5 ×2 | 1 |
| P2 | median → open | 2 |
| P3 | median → Gaussian → Otsu | 3 |
| P4 | median → Gaussian → Otsu → open | 4 |
| P5 | median → Gaussian → unsharp → Otsu → open | 5 |
| P6 | median → Gaussian → highboost → Otsu → open | 5 |
| P7 | median → Gaussian → Otsu → close → open → erode | 6 |
| P8 | median → Gaussian → Otsu → open → close | 5 |
| P9 | median → Gaussian → Laplacian → contrast stretch → Otsu → open | 6 |
| P10 | median → CLAHE → Gaussian → Otsu → open | 5 |
| P11 | median → box mean → Gaussian → Otsu → open → erode | 6 |
| P12 | Gaussian → median → Gaussian → Otsu → open → erode | 6 |

Scripts: `compare_enhance_pipelines.py` (images) → `ocr_pipeline_bakeoff.py` (EasyOCR) → `ocr_pipeline_gemma.py` (Gemma 3 4B). Same `canonical()` scoring.

| ID | EasyOCR | Gemma 3 4B |
|----|---------|------------|
| P0 | 4/12 | **10/12** |
| P1 | 4/12 | **10/12** |
| P2 | **5/12** | **10/12** |
| P3 | 2/12 | **10/12** |
| P4 | **5/12** | **10/12** |
| P5 | 4/12 | **10/12** |
| P6 | **5/12** | **10/12** |
| P7 | 3/12 | **10/12** |
| P8 | **5/12** | **10/12** |
| P9 | **5/12** | **10/12** |
| P10 | **5/12** | **10/12** |
| P11 | 4/12 | **10/12** |
| P12 | 4/12 | **10/12** |

EasyOCR never got MEM, AN, CAZ, AM, CRO; always got CIP 5 and CZ 30. Gemma got every disc except **SXT** (invented a dose: 25 / 123 / 125 / 500 / 625) and **CRO 30** (returned `30` only — `CRO` is already missing from the P0 crop). Tails did not change Gemma’s 10/12. CSVs: `test1_pipeline_bakeoff/easyocr_results.csv`, `gemma_results.csv`.

---

## 8. Per-disc plate result (Gemma after rim strip)

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

## 9. Files on disk vs history-only

**On disk:** extract / align (A–H + rim strip) / morph / letters-above / ocr_letters_above / hausdorff / gemma3 / pick_final / pipeline_c_letters_enhance / compare_enhance_pipelines / ocr_pipeline_bakeoff / ocr_pipeline_gemma / HANDOFF.md / this report / DIP PDF / test1 outputs / `test1_c_letters_enhance/` / `test1_pipeline_bakeoff/` (`easyocr_results.csv`, `gemma_results.csv`) / `discs_dataset/`.

**Deleted (cite this report, not the filename):** `batch_ocr.py`, `text_extraction_methods.py`, `deskew_methods.py`, `template_match.py`, `analyze_*.py`, `ocr_easyocr.py`, `ocr_read_text.py`, `otsu_angle_search.py`, `rotate_lines_ocr.py`, `antibiotic_ocr*.py`, `discs/`, `gm10.png`, `mem10.png`, thinning/debug trees, etc.

**Missing first chat:** prompts you already gave another LLM (Sonnet): GW Ch.1–4 all methods uncombined, `gem.png`/`mem10.png`, rotate then extract. That session’s **results** are in `HANDOFF.md`.

---

## 10. Open

- One `run_plate.py` for an arbitrary petri photo.  
- Retune Hough on test2–test9.  
- Independent GT; n=12 is not 99.9% accuracy.  
- SXT dashes; Hausdorff optional.  
- Pi 5 speed; Gemma 12B on 3060 if 4B fails new plates.  
- Vote-margin reject gate from crop era: never wired to Gemma.  
- EasyOCR max 5/12 on these 1-bit stamps; pick a tail using the Gemma vs EasyOCR table in §7.

---

## 11. One paragraph for a thesis

Classical geometry **cannot** reliably deskew a 5–7 character stamp with one PCA/Hough shot; a **dense 15° search** plus Otsu/EasyOCR **did** read **pre-cropped** `discs/` images. That recipe **did not** transfer to **Hough-extracted 1-bit plate stamps**. On `test1.png`, method **C** (two-line centroids) plus **open 5×5 ×2** plus **letters-above** made the stamps look upright to a human; Tesseract, EasyOCR, RapidOCR, and Hausdorff templates still failed because the glyphs are thick binary stamps, not fonts those engines were trained on. **Local Gemma 3 4B** read them **after** we stopped 896 nearest-neighbour zoom (which swapped T/Z) and **deleted leftover rim blobs without cropping letters**. That 12/12 is **one plate**, **offline after model download**, and **not** a single-click product from an arbitrary dish photo.
