# PetriOS

Automated blood-test bot: a computer-vision pipeline + SCARA robot + climate-controlled petri dish storage chamber for automated plate imaging and handling.

**Status:** Initial research phase — defining hardware specs and system architecture before build.

## Repo structure

```
PetriOS/
├── README.md
├── research/                          # specs, architecture notes
├── vision/
│   ├── petri_detect.py                # dish / plate detection
│   └── antibiotic_vision/             # disc stamp OCR (code + dose)
├── camera/
├── hardware/
└── esp_codes/
```

## Research index

- [Hardware Specs](research/01-hardware-specs.md) — camera, lens, lighting, autofocus, chamber requirements
- [System Architecture](research/02-system-architecture.md) — imaging station, SCARA bot, storage chamber layout
- [Disc stamp reading](vision/antibiotic_vision/README.md) — Kirby–Bauer code + dose pipeline, talk pack, process report

## System overview

Three subsystems working together:

1. **Imaging station** — camera + lens + lighting + XY stage, images each petri dish
2. **SCARA bot** — handles petri dishes, moves them between storage and imaging station
3. **Storage chamber** — temperature/humidity-controlled, holds petri dishes, QR/barcode-tagged

See [System Architecture](research/02-system-architecture.md) for the full diagram and details.
