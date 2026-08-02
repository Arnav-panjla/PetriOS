# System Architecture

Three subsystems: imaging station, SCARA bot, storage chamber.

![Camera setup diagram](../assets/camera-setup-diagram.png)
<!-- export the "Camera Setup" diagram from BTP RESEARCH .pdf (page 2) into assets/camera-setup-diagram.png -->

## Imaging station

| Component | Role |
|---|---|
| Diffused LED | illumination |
| 12 MP camera | image capture |
| Machine vision lens | C-mount, optics |
| Petri dish | held in place during imaging |
| XY moving stage | positions dish/camera for tiled capture |

Flow: diffused LED → 12 MP camera → machine vision lens → petri dish (on XY moving stage).

## SCARA bot

- Handles petri dishes
- Has access to the storage chamber (moves dishes in/out)

## Storage chamber

| Property | Value |
|---|---|
| Environment | temperature and humidity controlled |
| Contents | array of petri dishes |
| Identification | QR/barcode tags at the bottom of each dish slot |

## Open questions

- How do the SCARA bot and imaging station coordinate (handoff sequence, timing)?
- Tiling/stitching strategy for imaging station to reach target resolution (see [Hardware Specs](01-hardware-specs.md))
- Lighting placement and diffusion setup details

## Images

<!-- add architecture diagrams, CAD renders, etc. to ../assets and link here -->
