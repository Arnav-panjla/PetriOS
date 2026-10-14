#!/usr/bin/env python3

import cv2
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

from picamera2 import Picamera2


# ============================================================
# SETTINGS
# ============================================================

WIDTH = 1640
HEIGHT = 1232

PORT = 8000

# JPEG quality for streaming
JPEG_QUALITY = 80

# QR detection resolution
# The actual stream remains 1640x1232.
QR_WIDTH = 820
QR_HEIGHT = 616

# Check QR every N frames
QR_CHECK_EVERY = 3


# ============================================================
# GLOBAL VARIABLES
# ============================================================

latest_jpeg = None
frame_lock = threading.Lock()

running = True

last_qr = None


# ============================================================
# CAMERA SETUP
# ============================================================

picam2 = Picamera2()

camera_config = picam2.create_video_configuration(
    main={
        "size": (WIDTH, HEIGHT),
        "format": "BGR888"
    }
)

picam2.configure(camera_config)

# Start camera
picam2.start()

print("Camera started.")
print(f"Resolution: {WIDTH} x {HEIGHT}")
print(f"Stream: http://<RASPBERRY_PI_IP>:{PORT}")


# ============================================================
# QR DETECTOR
# ============================================================

qr_detector = cv2.QRCodeDetector()


# ============================================================
# CAMERA LOOP
# ============================================================

def camera_loop():

    global latest_jpeg
    global last_qr

    frame_count = 0

    while running:

        # Capture frame from camera
        frame = picam2.capture_array()

        frame_count += 1

        # ----------------------------------------------------
        # QR SCANNING
        # ----------------------------------------------------

        if frame_count % QR_CHECK_EVERY == 0:

            # Make smaller copy only for QR detection
            qr_frame = cv2.resize(
                frame,
                (QR_WIDTH, QR_HEIGHT)
            )

            try:

                data, points, _ = qr_detector.detectAndDecode(qr_frame)

                if data:

                    # Print only when a new QR is detected
                    if data != last_qr:

                        print("\n==============================")
                        print("QR CODE DETECTED:")
                        print(data)
                        print("==============================\n")

                        last_qr = data

            except Exception as e:
                print("QR detection error:", e)

        # ----------------------------------------------------
        # JPEG ENCODING FOR STREAM
        # ----------------------------------------------------

        success, jpeg = cv2.imencode(
            ".jpg",
            frame,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                JPEG_QUALITY
            ]
        )

        if success:

            with frame_lock:
                latest_jpeg = jpeg.tobytes()


# ============================================================
# HTTP STREAM SERVER
# ============================================================

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class StreamHandler(BaseHTTPRequestHandler):

    def do_GET(self):

        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()

            html = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Raspberry Pi Camera</title>
                <style>
                    body {
                        background: #111;
                        color: white;
                        text-align: center;
                        font-family: Arial;
                    }

                    img {
                        max-width: 95%;
                        height: auto;
                    }
                </style>
            </head>

            <body>

                <h2>Raspberry Pi Camera</h2>

                <img src="/stream.mjpg">

            </body>
            </html>
            """

            self.wfile.write(html.encode())

        elif self.path == "/stream.mjpg":

            self.send_response(200)

            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )

            self.send_header("Cache-Control", "no-cache")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")

            self.end_headers()

            try:

                while True:

                    with frame_lock:
                        jpeg = latest_jpeg

                    if jpeg is None:
                        time.sleep(0.01)
                        continue

                    self.wfile.write(
                        b"--frame\r\n"
                    )

                    self.send_header(
                        "Content-Type",
                        "image/jpeg"
                    )

                    self.send_header(
                        "Content-Length",
                        str(len(jpeg))
                    )

                    self.end_headers()

                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")

                    # Prevent excessive CPU/network usage
                    time.sleep(0.03)

            except (BrokenPipeError, ConnectionResetError):
                pass

        else:

            self.send_error(404)

    def log_message(self, format, *args):
        # Don't print every HTTP request
        pass


# ============================================================
# START EVERYTHING
# ============================================================

def main():

    global running

    # Start camera capture thread
    camera_thread = threading.Thread(
        target=camera_loop,
        daemon=True
    )

    camera_thread.start()

    # Start web server
    server = ThreadedHTTPServer(
        ("0.0.0.0", PORT),
        StreamHandler
    )

    print(f"HTTP server started on port {PORT}")
    print("Open the following in a browser:")
    print(f"http://<RASPBERRY_PI_IP>:{PORT}")
    print()
    print("Press Ctrl+C to stop.")

    try:

        server.serve_forever()

    except KeyboardInterrupt:

        print("\nStopping...")

    finally:

        running = False

        server.shutdown()
        server.server_close()

        picam2.stop()

        print("Camera stopped.")


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":
    main()
