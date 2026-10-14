#!/usr/bin/env python3
"""Live MJPEG stream from the Pi camera, viewable in a browser on the LAN.

    python3 camera/stream.py           # then open http://<pi-ip>:8000
    python3 camera/stream.py --selftest

Based on picamera2's mjpeg_server.py example. Stdlib + picamera2, nothing else.
"""

import argparse
import io
import socket
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ponytail: tuning knobs for a Pi 3B+ over Wi-Fi. Bump RESOLUTION once the stream
# works and you need detail on the plate; swap to JpegEncoder only if the hardware
# MJPEG encoder misbehaves (software JPEG costs a lot more CPU on a Pi 3).
RESOLUTION = (1640, 1232)
PORT = 8000

PAGE = b"""<!DOCTYPE html>
<html><head><title>PetriOS camera</title></head>
<body style="margin:0;background:#111">
<img src="stream.mjpg" style="width:100%;height:auto"/>
</body></html>"""


class StreamingOutput(io.BufferedIOBase):
    """Holds only the newest JPEG frame; readers wake up and skip the backlog."""

    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()

    def wait_frame(self, timeout=5):
        with self.condition:
            if not self.condition.wait(timeout):
                raise TimeoutError("no frame from camera")
            return self.frame


class StreamingHandler(BaseHTTPRequestHandler):
    output = None  # set in main()

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
            )
            self.end_headers()
            try:
                while True:
                    frame = self.output.wait_frame()
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # viewer closed the tab
            except TimeoutError as e:
                print(f"stream stopped: {e}", file=sys.stderr)
        else:
            self.send_error(404)


def local_ip():
    """Best-guess LAN address to print in the URL (UDP connect sends no traffic)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        except OSError:
            return "localhost"


def selftest():
    out = StreamingOutput()
    got = []
    t = threading.Thread(target=lambda: got.append(out.wait_frame()), daemon=True)
    t.start()
    threading.Event().wait(0.1)  # let the reader reach the wait
    out.write(b"\xff\xd8jpeg\xff\xd9")
    t.join(timeout=5)
    assert got == [b"\xff\xd8jpeg\xff\xd9"], f"frame handoff broken: {got}"
    print("selftest ok")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument("--size", default="{}x{}".format(*RESOLUTION), help="e.g. 1280x720")
    p.add_argument("--selftest", action="store_true", help="check without a camera")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    from picamera2 import Picamera2
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import FileOutput

    width, height = (int(n) for n in args.size.lower().split("x"))
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(main={"size": (width, height)}))

    StreamingHandler.output = StreamingOutput()
    picam2.start_recording(MJPEGEncoder(), FileOutput(StreamingHandler.output))
    try:
        server = ThreadingHTTPServer(("", args.port), StreamingHandler)
        server.daemon_threads = True
        print(f"serving on http://{local_ip()}:{args.port}  (ctrl-c to stop)")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop_recording()


if __name__ == "__main__":
    main()
