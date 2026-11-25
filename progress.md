# PetriOS progress log

End-to-end record of what was built, what worked, what failed, and what is still open. Written 25 Nov 2026.

PetriOS is an autonomous arm for antimicrobial-resistance (AMR) sampling: move a Petri dish, image it, measure inhibition zones, and read the antibiotic code stamped on each Kirby-Bauer disc.

Repository layout:

| Folder | What it holds |
|--------|---------------|
| `ARM_control/` | Arm control stack: PCA9685 driver, trajectories, IK, browser GUI, cameras, tests |
| `vision/` | Dish, disc and zone detector (`petri_detect.py`) and the disc-label reading work (`antibiotic_vision/`) |
| `Simulation/`, `assets/`, `research/`, `old_codes/` | Gazebo/URDF work, CAD renders, notes, earlier code |

Sections: 1 hardware and platform, 2 arm control and GUI, 3 cameras, 4 vision pipelines, 5 what failed, 6 open items, 7 how to run and test.

---

## 1. Hardware and platform

### Arm design iterations
| Iteration | Approach | Outcome |
|-----------|----------|---------|
| I | 3D-printer-style XY gantry | Rejected. The whole frame moved as one block, so the gripper could not reach every carousel slot. |
| II | SCARA arm | Reach was enough, but coupled control math and every link had to be designed and printed from scratch. Too slow. |
| Final | Ready-made 6-DOF arm | Two or three joints still need lengthening and the base needs widening so the gripper clears the carousel. |

### Compute
- Raspberry Pi 5. First installed Ubuntu Server (easier ROS 2), camera module verified there.
- Final OS is Raspberry Pi OS, because the servo HAT only ships drivers for it.
- Six servos on a PCA9685 (I2C). Open loop: there is no position feedback, so every angle shown anywhere is a commanded angle, never a measured one.

### Hardware changes during this work
- **J1 re-mounted.** The shoulder motor was re-mounted, so everything for J1 shifted by 36 degrees. The saved home for J1 went 36 -> 72 degrees, and the model zero for J1 became 75 degrees in `ik/arm_ik_v2.py` (`URDF_ZERO_SERVO`). The J1 limits stayed as they were before (soft limits 5..165 in the IK file). I first moved the lower limit to 41 by mistake; you caught it and I reverted both the IK limit and the GUI limit.
- **Gripper J5 conventions.** 167.5 degrees is fully closed, 10 degrees is fully open, maximum opening is 3.5 cm. Home is the open position.
- **ESP32-CAM on the gripper** pointing at the finger tips (J5 camera), connected by USB serial.
- **Pi CSI camera** and a **Zebronics USB webcam** (manual focus, shows up as "HD camera", YUYV 640x480 only) added for imaging.

---

## 2. Arm control stack and browser GUI

All control code is in `ARM_control/src/robot_arm/`; the GUI is `interfaces/web_gui.py`, a single stdlib `ThreadingHTTPServer` with one embedded HTML/JS page. Reached over an SSH tunnel.

### Built, in the order it happened
1. **Slider GUI over SSH.** Six joint sliders, startup snap to home, home button, soften (ms per degree) control, latest-wins move queue so dragging never builds a backlog.
2. **3D stick model from the URDF.** Joint origins and axes come from `configs/kinematics.yaml`, drawn side by side with the sliders. Shows commanded pose, slider target and IK solution in different colours.
3. **Sim/real calibration, then removed.** I built a "set sim to this pose and sync" calibration with a table of real vs sim angles. It never behaved the way you wanted (the model did not follow the hardware). I removed it when you supplied your own IK file with the exact servo-to-model mapping.
4. **IK integration (`ik/arm_ik_v2.py`).** Your IK solver is called from the GUI. Targets are in millimetres in the URDF `base_link` frame. Modes: absolute, relative, cylindrical relative, cylindrical absolute. Solving is preview-only until you press "Move arm to solution". Optional "hold tool direction".
5. **Saved home.** "Set current pose as home" writes `configs/home.txt` (servo degrees, pulse widths, TCP position). It overrides `joints.yaml` at the next start.
6. **Cylindrical IK.** Added dr, dtheta, dz (relative) and r, theta, z (absolute) from the gripper end point. The cylinder axis runs through the joint-1 origin; theta = 0 along base -Y, counter-clockwise positive seen from above.
7. **Z-floor guard.** Slider moves could drive the end point below z = 0 and hit the table. Now every move is checked with forward kinematics along the straight joint-space path from the current to the target pose (the planner uses one shared time for all joints, so the path is exactly that line). If the lowest TCP z falls under the floor (default 5 mm, editable, toggle in the top bar) the move is blocked, the sliders snap back, and a message explains why. A move that starts below the floor may still climb out. Covers slider moves, IK moves and Home.
8. **r, h, theta and tool-pitch sliders.** The J0-J3 sliders are replaced by r (mm), h (mm), theta (deg) and tool pitch (deg). J4 and J5 are unchanged. The raw J0-J3 sliders are behind a "Show raw joint sliders" checkbox. Each drag solves IK (warm-started from the commanded pose) and goes through the same floor guard.
9. **Keep gripper level.** A toggle that holds the tool approach axis horizontal (pitch 0) and sets J4 to 28 degrees (editable) so the fingers open top and bottom. With it on, raw J1/J2 moves recompute J3 to keep level, and the pitch slider is locked. The solver gained an optional `pitch_deg` argument in `arm_ik_v2.py`.
10. **ESP32 flash LED brightness.** Firmware now drives GPIO4 with LEDC PWM and a new `B`+byte command; the GUI has a 0-100% slider next to the Light button.

### Tests
`ARM_control/tests/` covers the arm state machine, kinematics, cylindrical conversions, the z-floor guard, level mode and r/h/theta solving, ESP brightness bytes, and the webcam detector. The last full run was 56 passing.

### Things that went wrong in this part
- **Unit bug.** The JS joint-angle helper returned radians while the table and FK used degrees. Fixed to degrees.
- **Stale browser state.** Earlier calibration offsets saved in `localStorage` confused things. Browser storage is no longer used.
- **Old GUI served after edits.** Several times the browser or the running process still had the old code. A restart plus a hard refresh was always needed; I told you this repeatedly.
- **Gripper drawing reversed.** The model showed the gripper opening when the real one closed, and the other way round. Redrawn so the tips meet when closed and are 3.5 cm apart when open.
- **Level mode at the saved home is impossible.** The tool is about 125 mm long, so a horizontal tool at home height would put the wrist behind the base. Level is only reachable when the arm is extended (about r >= 300 mm at h = 100 mm, r >= 200 mm at h = 200 mm). The GUI blocks the toggle there with an explanatory message instead of failing silently.
- **Visual verification never worked.** Headless Chromium screenshots failed every time, so the GUI layout and new controls were checked by reading the code and running the page script through Node, not by looking at them. Please look at them.
- **Simulation-only mode** (`--simulation-only`, a no-op PWM driver) was added by you and preserved across all later edits.

---

## 3. Cameras in the GUI

### ESP32-CAM over USB serial
- Firmware `ARM_control/esp32_cam/esp32_cam_usb/esp32_cam_usb.ino`; Pi side `vision/esp32_serial.py` (`SerialCamera`).
- Protocol: frames as `AA 55 AA 55`, a 32-bit length, then JPEG bytes.
- **Failure: corrupted frames.** At 921600 baud with one big `Serial.write` from PSRAM, about 212 bytes were lost per frame and the JPEG decoded with damaged Huffman tables. Fix: 460800 baud, 256-byte chunked writes with flush between chunks, 10 MHz camera clock, a single frame buffer, latest-frame grab mode.
- **Failure: no stream at first.** The GUI held the serial port while I tried to test, and the camera was initially not receiving; once the GUI was stopped the cause was found and it worked.
- Flash LED: on/off at first, later PWM brightness (section 2, item 10).

### Pi camera (CSI)
- `vision/pi_camera.py` reads rpicam-vid's MJPEG pipe. Settings (size, sharpness, contrast, denoise, zoom) can be changed from the GUI and restart the pipe.
- **Failure: camera busy.** Your own `qr_stream1.py` held the camera, so the GUI could not open it. Only one program can use the CSI camera at once.

### Latency
- **Failure: several seconds of lag on both cameras.** The MJPEG stream queued about 2 MB over the SSH tunnel. Fix: pull-based frames. The browser asks `/camera.jpg?src=&seq=` for a frame newer than the last one it showed, so nothing queues. The old stream endpoint has a small send buffer.

### QR detection and image quality
- QR boxes drawn thick, green when decoded and red when found but not decoded, with clickable links.
- **Failure: QR codes out of focus and noisy, especially on the Pi camera.** Added a focus meter, denoise, sharpen and CLAHE filters, a WeChat QR detector (needs `opencv-contrib`), a "robust decode" mode and 90-degree rotation steps. Every feature has a toggle and every filter has tunable sliders. The venv and `my_env` got `opencv-contrib-python-headless`.
- **Failure: QR and rotation did nothing at first** because the GUI ran in `my_env`, which had no OpenCV. Installed.
- A comparison table shows size, fps, JPEG size, processing and detect time, focus and QR found/decoded percentages for both cameras.

### USB webcam tab (new)
- A new Webcam tab (the page now has Arm and Webcam tabs). Live view, focus meter with peak hold for adjusting the manual focus ring, rotate, live dish outline, Capture button, annotated result image with a table of dish, disc and zone sizes in millimetres, and a download link.
- Camera auto-detected through `/sys/class/video4linux`, skipping the Pi's ISP and codec nodes.
- Only 640x480 YUYV is offered by this camera, so detail is limited.

---

## 4. Vision pipelines

There are two separate jobs. Zone measurement does not read the stamp, and stamp reading does not measure the zone.

### Part A: dish, disc and zone detection (`vision/petri_detect.py`)
Classical OpenCV, anchored to the known 90 mm dish so sizes are in millimetres.

1. **Dish.** Hough circle on the blurred image.
2. **Discs.** Small bright circles (radius 1.6-4.5 mm) that sit inside the dish, are bright and flat, and are clearly brighter than the agar around them. The local-contrast test rejects gloves, labels and rim glare.
3. **Zones.** For each disc a radial brightness profile is built (only over the agar that is actually visible) and the zone edge is the first strong outward-brightening step that is also round: a fraction of angular sectors must agree on it. A disc with no zone gets no purple circle, which is a result, not a failure.

Colours: dish blue, disc red, zone purple. Three sample plates are in `vision/sample/`.

#### Improvements made this month (webcam era)
You reported that the blue circle was mostly wrong and discs were missed. Using a real frame from the webcam (read from the running GUI):
- **Disc failure cause.** Only 1 of 4 discs was detected. The letters printed on the discs made the disc "not uniform", so the flatness test rejected them. The test now judges the disc by its brighter paper pixels only (75th percentile for brightness, spread of the brighter 70% of pixels). All 4 discs and 3 zones are now found on that frame (R about 12.8, 11.3 and 9.0 mm).
- **Dish rewrite.** Candidates come from Hough (low threshold, up to 12) plus Otsu contours for both a bright dish on a dark table and the reverse. Each candidate is scored by how much of its circle lies on a real brightness step, then refined with a local search. A slight preference for larger radius favours the outer rim.
- **Tunable parameters.** 15 values, shown as sliders in the Webcam tab: dish diameter, dish min/max radius, dish Hough sensitivity and blur, disc radius range, disc Hough sensitivity, brightness, spread and contrast, zone min/max radius, edge strength and roundness. Changes re-run detection on the captured frame (about 0.25 s) without re-capturing. "Reset to defaults" restores all.
- **Manual dish circle.** A checkbox with x, y and radius sliders overrides the auto dish when it is still wrong. It starts from the last detected circle.
- **Known weakness.** On `sample/s1.png` the refined dish snaps to the inner glass ring (R about 193 px) instead of the outer rim (203 px). That shifts the mm scale slightly and gives 2 zones instead of the old 3. Use the manual circle there, or tune. Thresholds are tuned on three sample plates and one webcam frame, not on a large set.
- Not changed: the CLI (`python3 vision/petri_detect.py image.png`) still works because `process()` calls the new `analyze()` function.

### Part B: reading the antibiotic code (`vision/antibiotic_vision/`)
Each disc carries a short stamp (`GM 10`, `TZP 110`, letters-only `SXT`). The stamp is thick and embossed, can sit at any angle, and often has leftover rim ink. Constraints you set: no custom-trained recogniser if off-the-shelf plus classical CV can work, and no snapping wrong letters to a list of drug names.

Two datasets that must never be mixed when quoting accuracy:
- **Set A:** about 50-70 px grey crops, now deleted from disk. A 15-degree rotation search with Otsu and EasyOCR reached 12/12 here.
- **Set B:** the whole-plate photo `test1.png`, still on disk, labels read by eye. The final pipeline reached 12/12 here with Gemma 3 4B. This is the headline result. It is one plate, not a clinical accuracy claim.

#### The pipeline that finally worked (run in order)
1. `extract_binary_discs.py`: Hough circles locate each disc, Otsu paints the agar white and the stamp black.
2. `align_binary_discs.py`: method C, two-line centroids (code row against dose row), rotated so the line between them is vertical. Load the saved PNG; do not recompute.
3. `morph_stroke_separate.py`: morphological open (5x5, two iterations) to separate touching strokes.
4. `align_letters_above_digits.py`: resolves the remaining 0/180 flip using the rule that letters print above digits.
5. `ocr_gemma3.py`: strip rim blobs that are not part of the letter mass, then read the native upright crop with local Gemma 3 4B through Ollama (offline once pulled). No dictionary.

#### What failed on the way (kept as results)
- **Textbook enhancements** (15 Gonzalez and Woods operations, one at a time): they change how the picture looks but produce no text.
- **Plain OCR** (Tesseract, EasyOCR on the whole disc, EasyOCR on a tight upscaled box): fails on rotated stamps. Cropping does not fix orientation.
- **One-shot angle estimators** (projection profiles, Hough lines, PCA on ink pixels, PCA per letter, gradient-orientation histogram, `minAreaRect`, template matching, Tesseract `--psm 0`, a neural angle regressor): each label has only 200-400 ink pixels, which is too little to estimate a line direction. Gradient-orientation scored 0-2 of 12, `minAreaRect` 1 of 12. The rim was proven not to be the cause.
- **OCR confidence as the angle selector:** EasyOCR is very sure of noise such as `8 8`, so confidence is not evidence of a real label.
- **Brute-force rotation search:** a 15-degree grid over 360 degrees plus fine refine and voting reached 12/12 on set A, but that was the grid density, not a better filter. Log transform and FFT high-pass were 0/12 always.
- **Other recognisers** (PARSeq, PaddleOCR, TrOCR, ABINet, SATRN, SVTR, CRNN, YOLO, Qwen2.5-VL, HunyuanOCR, DeepSeek-OCR) were looked at or partly tried; none beat the chain above on set B, and training a custom model was ruled out by your constraint.
- **Whole-plate OCR with classical engines** (EasyOCR, Tesseract, Hausdorff matching): failed on the fat 1-bit stamps because those engines are trained on thin printed fonts.
- **Four-way upright after morphology** failed, and one run accidentally recomputed method C so the image was not the one you had inspected.
- **Serial Gonzalez and Woods pipelines P0-P12:** negative result. Stacking more textbook operations did not help; the existing order with Gemma already scored 12/12.
- **Upscaling the crop to 896x896:** nearest-neighbour zoom made Gemma swap `TZP` into `ZTP`, so Gemma reads the native crop.
- **Stock plates (watermarked photos):** the old script capped at 16 discs per plate, filled the cap with specks, and read nonsense (`TAB 25`, `AM 500`). One run died on a 10-minute timeout and produced no CSV. Replaced by a full-plate finder.
- **Ten new full-dish photos (86 discs):**
  - Same chain with the morphological open: letters were destroyed on some discs (for example TE 30 and VA 30 became fragments), so morphology stays off for this set.
  - Chain with method C and no morph: 71 of 86 parsed. Chain with method C also off: 74 of 86 parsed.
  - Colour crops sent straight to Gemma: you rejected it because it was not the pipeline you built.
  - `parse_label` requires 2-4 letters, so one-letter reads are stored raw with a blank parsed cell.

Detailed method catalogue and process write-up: `vision/antibiotic_vision/BTP_FULL_REPORT.md` and `PROCESS_REPORT.md`.

---

## 5. Summary of failures and root causes

| Area | What failed | Root cause | Fix or status |
|------|-------------|------------|---------------|
| ESP32 frames | Corrupted JPEGs | One large serial write dropped bytes at 921600 baud | 460800 baud, chunked writes, slower clock |
| Cameras | Seconds of lag | MJPEG stream queued in the SSH tunnel | Pull-based frame fetch |
| QR | Not decoding, blurry | Focus and noise, wrong Python env | Filters, focus meter, contrib OpenCV |
| GUI | New code not shown | Old process or cached page | Restart plus hard refresh |
| Model | Gripper reversed | Finger geometry drawn wrong way | Redrawn |
| Calibration | Sim never matched | Manual sync approach | Removed; use IK mapping file |
| J1 | Limit wrongly shifted | My mistake | Reverted |
| Slider safety | TCP below table | No check on slider moves | FK z-floor guard |
| Level mode | Blocked at home | Geometry: wrist would be behind base | Message; use when extended |
| Disc detection | Lettered discs rejected | Flatness test counted ink | Percentile-based test |
| Dish detection | Blue circle wrong | Single Hough pass, fixed radius range | Scored candidates, sliders, manual override |
| Label reading | Classical OCR and angle estimators | Too few ink pixels, wrong font class | Geometry plus Gemma 3 4B |

---

## 6. Open items and risks

- **GUI never visually checked.** Please open it and check the Arm and Webcam tabs, the new r/h/theta sliders, the level toggle and the tuning card.
- **No position feedback.** Everything is commanded angles. The z-floor guard and level mode protect only the TCP; the elbow, wrist and link bodies can still hit the table.
- **Camera resolution.** The webcam delivers 640x480 only. Zone and disc detection quality at that size is untested beyond a few frames.
- **Detector tuning data.** Thresholds are tuned on three sample plates and one webcam frame. Collect more plates through the Webcam tab and tune.
- **Dish scale assumption.** Millimetre values assume a 90 mm dish that is fully in view and fills most of the frame, with the camera looking straight down.
- **Code reading is not integrated into the GUI.** The Gemma 3 pipeline runs from scripts in `vision/antibiotic_vision/` and needs Ollama with `gemma3:4b`. It is not in the Webcam tab yet.
- **Level at low heights** is geometrically impossible with the current tool length.
- **Mechanical work remaining:** the arm still needs two to three links lengthened and a wider base; the drawing of the modified arm is not done; the URDF and world files are not checked in.
- **ESP32 firmware** must be re-flashed for the brightness slider to work (the old firmware ignores `B`).

---

## 7. How to run and test

```bash
# GUI (from ARM_control/, use the environment that has opencv-contrib)
python -m robot_arm.interfaces.web_gui --host 127.0.0.1 --port 8000
# flags: --simulation-only, --no-camera, --no-pi-camera, --no-webcam,
#        --webcam-device /dev/video0, --webcam-rotate 90, --camera-port, --pi-width/--pi-height/--pi-fps
# tunnel from the laptop:  ssh -L 8000:localhost:8000 <user>@<pi>

# tests
.venv/bin/python -m pytest -q

# dish/disc/zone detector on a file
python3 vision/petri_detect.py vision/sample/s1.png
```

Only one program can use a camera at a time: stop the GUI before running your own camera scripts, and the other way round.
