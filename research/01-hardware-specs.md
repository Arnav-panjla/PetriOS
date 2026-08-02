# Hardware Specs

Initial spec notes and open questions from early research. Values marked with `???` are unresolved.

## Petri dish storage

| Item | Value |
|---|---|
| Layout | rows x columns — **TBD** |
| Count | **TBD** |

## Camera

Target resolution driven by required pixel size on a 90 mm plate:

| Parameter | Value |
|---|---|
| Plate diameter | 90 mm |
| Required pixel size | 5 µm/pixel |
| Pixels required (linear) | 90,000 µm / 5 = 18,000 px |
| Approx sensor resolution | ~18,000 x 18,000 = 324 MP |

Candidate cameras:

| Camera | Sensor | Resolution | Notes |
|---|---|---|---|
| Raspberry Pi Camera 3 | IMX708 | 12 MP | falls well short of the 324 MP target — needs tiling/stitching or scan approach |
| Sony IMX477 HQ Camera | IMX477 | 12 MP | + quality C-mount lens |

**Open question:** 324 MP is far beyond any single-shot sensor above — likely resolved via multi-shot stitching (XY stage moves + captures tiles) rather than a single sensor. Needs decision.

## Lens

- C-mount machine vision lens (model TBD)

## Lighting

- **???** — type and placement not yet decided (diffused LED assumed per architecture diagram, see [System Architecture](02-system-architecture.md))

## Autofocus

- Autofocus varies shot-to-shot, which hurts consistency of quantitative measurements
- **Decision:** go with fixed focus, or a motorized Z-axis for repeatable focus control

## Environmental chamber

- Temperature and humidity controlled
- **Open question:** lens fogging risk inside a humidity-controlled chamber — not yet addressed

## Images

<!-- add reference photos / datasheets to ../assets and link here -->
