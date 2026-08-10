# NoIR Camera V2 

```bash
sudo apt update
sudo apt full-upgrade
sudo apt install -y python3-picamera2 --no-install-recommends #for_lite_system
sudo apt install -y python3-picamera2 #for_full_version
```



## Live stream (headless, view from your laptop)

`stream.py` serves an MJPEG stream over HTTP — run it on the Pi over SSH, watch it in a
browser on any machine on the same network. No GUI needed on the Pi.

```bash
python3 camera/stream.py --selftest     # sanity check, no camera required
python3 camera/stream.py                # prints the URL, ctrl-c to stop
hostname -I                             # the Pi's IP, if the printed one looks wrong
```

Then open `http://<pi-ip>:8000` on your laptop.

Defaults to 640x480 — enough to check framing and focus without choking a Pi 3B+ over
Wi-Fi. Bump it once it works: `--size 1280x720`. Port with `--port`.

## Live QR-code scanning while streaming

`qr_stream.py` is the same MJPEG-over-HTTP stream, but each frame is scanned for QR
codes: detected codes get a green box drawn on the video and their contents printed
once to the terminal (not spammed every frame while the same code stays in view).

One-time setup on the Pi:

```bash
sudo apt install -y libzbar0 python3-pil
pip3 install pyzbar
```

Then:

```bash
python3 camera/qr_stream.py --selftest   # sanity check, no camera required
python3 camera/qr_stream.py              # prints the URL, ctrl-c to stop
```

Same `--size`/`--port` flags as `stream.py`. This runs the JPEG encode in software
(needed to draw the overlay), so it's slower/heavier on the Pi 3B+ than the plain
hardware-encoded `stream.py` — expect a few FPS rather than smooth video.

## Python script
- with GUI
```bash
from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start_preview(Preview.QTGL)
picam2.start()
time.sleep(2)
picam2.capture_file("test.jpg")
```

- without GUI
```bash
from picamera2 import Picamera2, Preview
import time
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration()
picam2.configure(camera_config)
picam2.start_preview(Preview.DRM)
picam2.start()
time.sleep(2)
picam2.capture_file("test.jpg")
```
