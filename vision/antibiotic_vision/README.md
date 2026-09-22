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
