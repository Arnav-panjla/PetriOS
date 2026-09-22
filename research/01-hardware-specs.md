# Hardware specs

Early optics notes, plus the prototype parts locked in the mid-term bill of materials. Values marked `???` are still unresolved. The project front page is the [README](../README.md). Architecture is [02-system-architecture.md](02-system-architecture.md).

## Prototype selected for the mid-term

| Role | Part | Where it is written down |
|------|------|---------------------------|
| Vision computer | Raspberry Pi 5, 4 GB | [`hardware/BOM.csv`](../hardware/BOM.csv) |
| Plate camera | Camera Module 3, 12 MP | BOM |
| Gripper / QR camera | ESP32-CAM | BOM and [`esp_codes/esp_cam_codes/`](../esp_codes/esp_cam_codes/) |
| Joints (SCARA draft) | 5× NEMA17, drivers, belts, limit switches | BOM |
| Motion board | Arduino Nano + CNC shield | BOM. ESP32 is the control target in the deck. |
| Gripper | Micro servo | BOM |
| Carousel | High-torque servo, bearing, dish inserts | BOM |
| Power envelope | ~95 W typical, ~145 W peak | [`hardware/power_budget.md`](../hardware/power_budget.md) |

The revised deck replaces the custom SCARA with a procured 6-DOF arm. The BOM above is the SCARA shopping list and has not been rewritten for that arm.

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

- **???** — type and placement not yet decided. The mid-term deck assumes diffused light at a fixed working distance. See [System Architecture](02-system-architecture.md).

## Autofocus

- Autofocus varies shot-to-shot, which hurts consistency of quantitative measurements
- **Decision:** go with fixed focus, or a motorized Z-axis for repeatable focus control

## Environmental chamber

- Temperature and humidity controlled
- **Open question:** lens fogging risk inside a humidity-controlled chamber — not yet addressed

## Images already in the repo

Plate and zone photos used on the project front page:

- [`vision/sample/s1.png`](../vision/sample/s1.png) and [`s1_detected.png`](../vision/sample/s1_detected.png)
- Disc-reading sequence: [`vision/antibiotic_vision/talk_pack/02_slides_images/pipeline/`](../vision/antibiotic_vision/talk_pack/02_slides_images/pipeline/)

Pi camera frames at 20–45 cm (QR distance checks) are in [`camera/images/`](../camera/images/). No CAD render or chamber photo is checked in. The `assets/` folder is empty.
