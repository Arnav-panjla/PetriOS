# NoIR Camera V2 

```bash
sudo apt update
sudo apt full-upgrade
sudo apt install -y python3-picamera2 --no-install-recommends #for_lite_system
sudo apt install -y python3-picamera2 #for_full_version
```



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
