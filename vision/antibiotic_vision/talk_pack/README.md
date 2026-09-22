# Talk pack — antibiotic disc reading (BTP 1)

Copies of the files you need for a PPT or a verbal walkthrough.  
Originals stay in the project folder; this pack is **read-only for presenting**.

**Open this first:** `01_reports/PROCESS_REPORT.md`  
(method catalogue: `BTP_FULL_REPORT.md` · crop-era log: `HANDOFF_crop_era.md`)

---

## Suggested slides (drop images in this order)

| Slide | File | What to say |
|-------|------|-------------|
| Problem | `02_slides_images/pipeline/00_plate_test1.png` | Read code + dose on stamps, any angle. Not page OCR. |
| Find discs | `01_hough_annotated.png` then `02_binary_discs.png` | Hough + Otsu. Black letters, white disc. |
| Align | `03b_before_align.png` then `03_align_method_C.png` | Method C (two-line). PCA/Hough failed. Still 0° vs 180°. |
| Morph | `04_morph_open.png` | Open 5×5 ×2. Separate thick strokes. Does not read text. |
| Upright | `05a_before_letters_above.png` then `05_letters_above.png` | Letters on top, digits below. |
| Classical OCR died | `03_results/test1_hausdorff.csv` | Hausdorff 1/12. EasyOCR best 5/12. Letters look obvious to us. |
| Gemma zoom bug | `gemma_fixes/TZP_896_nearest.png` | TZP became ZTP. Do not blow up to 896 nearest. |
| Gemma rim bug | `GM_rim_before.png` → `GM_rim_after.png` | Arc read as I (GMI). Strip rim, keep letters. |
| Final 12 | `pipeline/final_crops_gemma/` + `03_results/test1_gemma_12of12.csv` | Gemma 3 4B native, 12/12 on **this plate**. |
| Not universal | `03_results/stock_plates_hough.txt` | Other photos: detector, not Gemma, is the gap. |

**Do not mix two 12/12 scores.** Old tiny grey crops ≠ `test1.png`. `disc_02` is CZ 30 on the old list, TZP 110 on the plate.

---

## Folder map

```
talk_pack/
  README.md                 ← you are here
  01_reports/               ← speak from PROCESS_REPORT.md
  02_slides_images/         ← drag into PowerPoint
    pipeline/               ← one plate, step by step
    gemma_fixes/            ← ZTP and GMI stories
  03_results/               ← CSVs for a results table slide
  04_pipeline_code/         ← last working scripts (not for slides)
```

`04_pipeline_code` is only if someone asks “what do we actually run?”  
Order: extract → align C (saved PNG) → morph → letters-above → `ocr_gemma3.py`.

---

## Numbers you can put on one slide

| Attempt | Score | Data |
|---------|-------|------|
| EasyOCR confidence spin | 3/12 | deleted grey crops |
| 15° + Otsu + EasyOCR | 12/12 | those same grey crops only |
| EasyOCR on upright 1-bit stamps | 5/12 | test1 |
| Hausdorff templates | 1/12 | test1 |
| Gemma 896 zoom | 10/12 | test1 |
| Gemma native + rim strip | **12/12** | test1 |
| Hough on stock plates | 25 proper / 33 real | not OCR |

Ground truth was read by eye. n = 12 is not 99.9% clinical accuracy.
