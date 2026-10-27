# Antibiotic disc stamp reading

BTP vision work: read the printed **code + dose** on Kirby–Bauer discs from a plate photo.

The project front page, with the plate photos and the 12/12 table, is the [repository README](../../README.md). This folder is the working tree (textbook PDF omitted — copyright).

## For a talk

Start here: [`talk_pack/README.md`](talk_pack/README.md)  
Process write-up: [`PROCESS_REPORT.md`](PROCESS_REPORT.md)

## Last pipeline (`test1.png`)

1. `extract_binary_discs.py` — Hough circles → Otsu binary discs  
2. `align_binary_discs.py` — method C (two-line centroids); **load the saved PNG**, do not recompute  
3. `morph_stroke_separate.py` — open 5×5 ×2  
4. `align_letters_above_digits.py` — 0° vs 180°  
5. `ocr_gemma3.py` — rim strip + local Ollama `gemma3:4b`

Needs Ollama with `gemma3:4b` pulled. OpenCV + EasyOCR (0/180 only). No antibiotic-name dictionary.

**12/12 on `test1.png` is one plate.** Do not mix that score with the older tiny-crop EasyOCR 12/12.

Existing PetriOS dish detector stays at [`../petri_detect.py`](../petri_detect.py).

## Ten later plates

`new_images_for_testing/` holds 10 full-dish photos, 86 discs. Eye-read labels are in `new_images_pipeline/ground_truth_read.csv`. Keep this score separate from the test1 12/12.

`detect_discs_v2.py` found those discs. The September open (5×5, twice) breaks stamps that were still readable after alignment. On plate `1b55f49e`, `new_images_steps/.../02_aligned/disc_04.png` is TE 30 and `disc_05.png` is VA 30. The matching files under `03_morph/` are fragments. Only those four crops are in git.

Morphology stays off for this folder:

- `run_pipeline_no_morph.py` — method C on. Parsed **71/86**. CSV: `new_images_pipeline/results.csv`.
- `run_pipeline_no_align.py` — method C off. Parsed **74/86**. CSV: `new_images_pipeline_no_align/results.csv`.

The test1 order above is unchanged, open included. `save_plate_steps.py` is the script that wrote the before/after.
