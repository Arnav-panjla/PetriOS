"""
Final label selection.

batch_ocr.py produces many readings per disc (one per enhancement method, at
each shortlisted rotation). Picking the single highest-confidence reading is
unreliable -- the engine is often confidently wrong. Two things fix it:

  1. Canonicalization. OCR mixes up characters that look alike (J/I/L vs 1,
     O/D/Q vs 0, S vs 5, B vs 8). Disc labels are always an antibiotic code
     followed by a dose from a small standard set, so a reading is snapped to
     "letters + valid dose" when such a reading exists ('NNJ0' -> 'NN10').

  2. Consensus. Correct readings recur across different methods and angles;
     garbage readings scatter. So the label with the highest summed score
     across all readings wins, not the single best reading.

Run directly to (re)compute final labels from an existing results.csv.
"""
import collections
import csv
import sys

# characters OCR commonly confuses for digits, applied only inside the dose
CONFUSIONS = str.maketrans({
    "J": "1", "I": "1", "L": "1",
    "O": "0", "D": "0", "Q": "0",
    "S": "5", "B": "8",
})

# standard antibiotic disc potencies (micrograms) seen on Kirby-Bauer discs
DOSES = {1, 2, 5, 10, 15, 20, 23, 25, 30, 50, 75, 100, 110, 120, 300}


def normalize(text):
    return "".join(text.split()).upper()


def canonical(text):
    """Snap a raw reading to 'CODE' or 'CODE<dose>' when possible."""
    flat = normalize(text)
    if not flat or flat.isalpha():
        return flat
    for split in range(1, len(flat)):
        code, dose = flat[:split], flat[split:].translate(CONFUSIONS)
        if code.isalpha() and dose.isdigit() and int(dose) in DOSES:
            return code + dose
    return flat


def choose(rows):
    """Pick the consensus label for one disc from its candidate readings.
    Each row needs 'text_read' and 'label_score'.
    Returns (canonical_label, display_label, votes), where display_label is the
    best raw reading behind the winner -- canonicalization is good for matching
    but mangles codes ending in O ('CRO 30' -> 'CR030'), so it is not shown."""
    votes = collections.defaultdict(float)
    best_raw = {}
    for r in rows:
        label = canonical(r["text_read"])
        if not label:
            continue
        score = float(r["label_score"])
        votes[label] += score
        if label not in best_raw or score > best_raw[label][0]:
            best_raw[label] = (score, r["text_read"].strip())

    if not votes:
        return "", "", {}
    winner = max(votes.items(), key=lambda kv: kv[1])[0]
    return winner, best_raw[winner][1], dict(votes)


def main(results_path="results.csv", out_path="final_labels.csv"):
    with open(results_path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))

    by_image = collections.defaultdict(list)
    for r in rows:
        by_image[r["image"]].append(r)

    out_rows = []
    correct = 0
    for image, image_rows in sorted(by_image.items()):
        label, display, votes = choose(image_rows)
        truth_raw = image_rows[0].get("ground_truth", "")
        truth = canonical(truth_raw)
        ok = bool(truth) and label == truth
        correct += ok

        runner_up = sorted(votes.items(), key=lambda kv: -kv[1])[1:2]
        margin = votes[label] - (runner_up[0][1] if runner_up else 0.0)

        print(f"  {'OK  ' if ok else 'MISS'} {image}  '{display}'"
              f"   truth '{truth_raw}'   vote {votes[label]:.2f} (margin {margin:.2f})")
        out_rows.append({
            "image": image,
            "final_label": display,
            "matched_form": label,
            "ground_truth": truth_raw,
            "correct": int(ok),
            "vote_score": f"{votes[label]:.3f}",
            "margin": f"{margin:.3f}",
        })

    with open(out_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(out_rows[0].keys()))
        writer.writeheader()
        writer.writerows(out_rows)

    print(f"\nFinal accuracy: {correct}/{len(out_rows)}")
    print(f"Written: {out_path}")


if __name__ == "__main__":
    main(*sys.argv[1:])
