#!/usr/bin/env python3
"""Live MJPEG stream with real-time QR code detection, viewable in a browser.

    python3 camera/qr_stream.py        # then open http://<pi-ip>:8000
    python3 camera/qr_stream.py --selftest

Detected codes get a green box drawn on the stream and their payload printed
to the terminal (once per new value, not every frame).

New deps on top of stream.py's picamera2 (Pillow is usually already present):
    sudo apt install -y libzbar0 python3-pil
    pip3 install pyzbar
"""

import argparse
import io
import os
import sys
import threading
from http.server import ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stream import PORT, RESOLUTION, StreamingHandler, StreamingOutput, local_ip


def draw_overlay(image, codes):
    from PIL import ImageDraw

    draw = ImageDraw.Draw(image)
    for code in codes:
        x, y, w, h = code.rect
        draw.rectangle([x, y, x + w, y + h], outline="lime", width=3)
        draw.text((x, max(0, y - 12)), code.data.decode("utf-8", "replace"), fill="lime")
    return image


def announce(codes, last_seen):
    """Print newly-seen QR payloads once; return the updated seen-set."""
    seen = {c.data for c in codes}
    for data in seen - last_seen:
        print(f"QR: {data.decode('utf-8', 'replace')}")
    return seen


def selftest():
    from PIL import Image
    from pyzbar.pyzbar import decode

    blank = Image.new("L", (64, 64), color=128)
    assert decode(blank) == [], "decode() should find nothing in a blank image"

    overlaid = draw_overlay(blank.convert("RGB"), [])
    assert overlaid.size == (64, 64)

    last_seen = announce([], set())
    assert last_seen == set()
    print("selftest ok")


def capture_loop(picam2, output, stop_event):
    from pyzbar.pyzbar import decode
    from PIL import Image

    last_seen = set()
    while not stop_event.is_set():
        frame = picam2.capture_array()
        image = Image.fromarray(frame).convert("RGB")
        codes = decode(image)
        last_seen = announce(codes, last_seen)
        draw_overlay(image, codes)

        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=80)
        output.write(buf.getvalue())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--size", default="{}x{}".format(*RESOLUTION), help="e.g. 1280x720")
    p.add_argument("--selftest", action="store_true", help="check without a camera")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    from picamera2 import Picamera2

    width, height = (int(n) for n in args.size.lower().split("x"))
    picam2 = Picamera2()
    picam2.configure(
        picam2.create_video_configuration(main={"size": (width, height), "format": "RGB888"})
    )
    picam2.start()

    StreamingHandler.output = StreamingOutput()
    stop_event = threading.Event()
    capture_thread = threading.Thread(
        target=capture_loop, args=(picam2, StreamingHandler.output, stop_event), daemon=True
    )
    capture_thread.start()

    try:
        server = ThreadingHTTPServer(("", args.port), StreamingHandler)
        server.daemon_threads = True
        print(f"serving on http://{local_ip()}:{args.port}  (ctrl-c to stop)")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        picam2.stop()


if __name__ == "__main__":
    main()
