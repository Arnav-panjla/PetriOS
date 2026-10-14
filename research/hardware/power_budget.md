# PetriOS Power Budget (rough estimate)

Assumes NEMA17 steppers (~1.5A/phase, 12V) driven via standard drivers (A4988/DRV8825/TMC2209),
a 15–25kg-cm hobby servo for the carousel, and small logic boards for control/vision.

| Component | Qty | Voltage | Current (typ) | Current (peak) | Notes |
|---|---|---|---|---|---|
| NEMA17 stepper (SCARA joints) | 5 | 12V | 1.0 A each | 1.5 A each | via stepper drivers, current-limited |
| Stepper driver logic overhead | 5 | 5V (logic) | ~0.02 A each | — | negligible, powered from controller |
| Servo (end-effector joint) | 1 | 5–6V | 0.5 A | 1.5 A | small hobby servo |
| High-torque servo (carousel) | 1 | 6–7.4V | 1.5 A | 3.0 A | stall current dominates |
| ESP32-CAM (QR scan) | 1 | 5V | 0.3 A | 0.5 A | wifi streaming |
| Arduino Nano (SCARA control) | 1 | 5V | 0.05 A | 0.1 A | logic only |
| Raspberry Pi 5 | 1 | 5V | 3.0 A | 5.0 A | official PSU is 5V/5A |
| RPi 12MP camera module | 1 | 3.3V (from Pi CSI) | 0.25 A | 0.3 A | powered by Pi, not separate rail |

## Rail totals (rough, typ / peak)

**12V rail (steppers):**
- Typ: 5 × 1.0A = 5.0 A → **60 W**
- Peak: 5 × 1.5A = 7.5 A → **90 W**

**5–7.4V servo rail (carousel + end-effector servo):**
- Typ: 0.5 + 1.5 = 2.0 A
- Peak: 1.5 + 3.0 = 4.5 A
- If run off a 6V rail: ~12–27 W

**5V logic/compute rail (Pi 5, ESP32-CAM, Nano):**
- Typ: 3.0 + 0.3 + 0.05 ≈ 3.35 A → **~17 W**
- Peak: 5.0 + 0.5 + 0.1 ≈ 5.6 A → **~28 W**

## System-level recommendation

- **12V supply:** ≥8A (96W) — covers 5 steppers with headroom.
- **6V servo supply:** ≥5A (30W), separate from logic rail to avoid brownouts on the Pi.
- **5V supply:** ≥6A (30W), dedicated for Pi 5 + ESP32-CAM + Nano (Pi 5 alone wants a clean 5V/5A source).
- **Total system draw (peak, all rails):** roughly 90W + 27W + 28W ≈ **~145 W**, typical closer to **~95 W**.

ponytail: these are rough hobby-motor/datasheet-typical numbers, not measured — replace with actual measured/stall currents once motors are on hand, especially for the carousel servo (stall current varies a lot by model).
