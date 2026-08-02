# PetriOS

Automated blood-test bot: a computer-vision pipeline + SCARA robot + climate-controlled petri dish storage chamber for automated plate imaging and handling.

**Status:** Initial research phase — defining hardware specs and system architecture before build.

## Repo structure

```
PetriOS/
├── README.md
├── research/       # specs, open questions, architecture notes (source of truth for now)
└── assets/         # diagrams / reference images used in research docs
```

## Research index

- [Hardware Specs](research/01-hardware-specs.md) — camera, lens, lighting, autofocus, chamber requirements
- [System Architecture](research/02-system-architecture.md) — imaging station, SCARA bot, storage chamber layout

## System overview

Three subsystems working together:

1. **Imaging station** — camera + lens + lighting + XY stage, images each petri dish
2. **SCARA bot** — handles petri dishes, moves them between storage and imaging station
3. **Storage chamber** — temperature/humidity-controlled, holds petri dishes, QR/barcode-tagged

See [System Architecture](research/02-system-architecture.md) for the full diagram and details.
