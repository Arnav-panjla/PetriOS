"""ESP32-CAM over USB serial. Pairs with esp32_cam/esp32_cam_usb/esp32_cam_usb.ino.

Wire format per frame: AA 55 AA 55, uint32 little-endian length, JPEG bytes.
A background thread keeps the newest frame; readers wait for a new one.
"""

from __future__ import annotations

import glob
import logging
import struct
import threading
import time

logger = logging.getLogger(__name__)

MAGIC = b"\xaa\x55\xaa\x55"
MAX_FRAME = 300_000  # bytes; anything bigger is treated as line noise


def find_port() -> str | None:
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


class SerialCamera:
    """Reads JPEG frames from the ESP32-CAM and reconnects if the cable drops."""

    def __init__(self, port: str | None = None, baud: int = 460800) -> None:
        self.port = port
        self.baud = baud
        self.connected = False
        self.last_error: str | None = None
        self.bytes_rx = 0
        self.bad_frames = 0
        self.light = False
        self._link = None
        self._wlock = threading.Lock()
        self.tail = b""  # last bytes received, for diagnosis
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._times: list[float] = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    @property
    def fps(self) -> float:
        now = time.monotonic()
        recent = [t for t in self._times if now - t < 3.0]
        return len(recent) / 3.0 if recent else 0.0

    def latest(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def wait_frame(self, last_seq: int, timeout: float = 2.0) -> tuple[bytes | None, int]:
        """Block until a frame newer than last_seq exists. Returns (jpeg, seq)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq or self._stop, timeout)
            return self._jpeg, self._seq

    def set_light(self, on: bool) -> None:
        """Switch the ESP32-CAM flash LED. Raises OSError when not connected."""
        link = self._link
        if link is None or not self.connected:
            raise OSError("ESP32-CAM is not connected")
        with self._wlock:
            link.write(b"L" if on else b"l")
            link.flush()
        self.light = on

    def _publish(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()
        now = time.monotonic()
        self._times = [t for t in self._times if now - t < 3.0] + [now]

    def _run(self) -> None:
        import serial  # pyserial

        while not self._stop:
            port = self.port or find_port()
            if port is None:
                self.connected = False
                self.last_error = "no /dev/ttyUSB* or /dev/ttyACM* found"
                time.sleep(1.0)
                continue
            try:
                # Open with DTR/RTS low so opening the port does not reset the ESP32.
                link = serial.Serial()
                link.port, link.baudrate, link.timeout = port, self.baud, 1.0
                link.dtr = False
                link.rts = False
                link.open()
                self._link = link
                self.connected, self.last_error = True, None
                logger.info("camera connected on %s @ %d", port, self.baud)
                self._read_frames(link)
            except Exception as exc:  # serial.SerialException, OSError
                self.connected = False
                self.last_error = f"{type(exc).__name__}: {exc}"
                time.sleep(1.0)

    def _read_frames(self, link) -> None:
        buf = bytearray()
        while not self._stop:
            chunk = link.read(max(1, link.in_waiting))
            if not chunk:
                continue
            buf += chunk
            self.bytes_rx += len(chunk)
            self.tail = (self.tail + chunk)[-32:]
            while True:
                start = buf.find(MAGIC)
                if start < 0:
                    del buf[: max(0, len(buf) - 3)]  # keep a possible partial magic
                    break
                if len(buf) < start + 8:
                    del buf[:start]
                    break
                (length,) = struct.unpack_from("<I", buf, start + 4)
                if length == 0 or length > MAX_FRAME:
                    del buf[: start + 4]  # false magic, resync
                    continue
                end = start + 8 + length
                if len(buf) < end:
                    del buf[:start]
                    break
                jpeg = bytes(buf[start + 8 : end])
                del buf[:end]
                if jpeg[:2] == b"\xff\xd8" and jpeg[-2:] == b"\xff\xd9":
                    self._publish(jpeg)
                else:
                    self.bad_frames += 1


def probe(port: str | None, baud: int, seconds: float = 4.0) -> None:
    """Print what the port is sending. Stop the GUI first (one reader per port)."""
    import serial

    port = port or find_port()
    print(f"port {port} baud {baud}")
    link = serial.Serial()
    link.port, link.baudrate, link.timeout = port, baud, 0.5
    link.dtr = False
    link.rts = False
    link.open()
    data = b""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        data += link.read(4096)
    link.close()
    print(f"{len(data)} bytes, {data.count(MAGIC)} frame markers")
    print("first 64 bytes:", data[:64])
    if not data:
        print("nothing received: firmware not running, camera init failed (flash LED blinks), or wrong port")
    elif MAGIC not in data:
        print("data but no frame marker: wrong baud, or firmware not flashed (boot text only)")


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Probe the ESP32-CAM serial stream.")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=460800)
    args = ap.parse_args()
    probe(args.port, args.baud)
