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
