"""16-LED WS2812 ring on SPI MOSI (GPIO10, pin 19). D0 = data in; leave D1 unconnected.

Each WS2812 bit is sent as one SPI byte at 6.4 MHz: 1 -> 0xF8, 0 -> 0xC0.
Needs SPI on (sudo raspi-config nonint do_spi 0) and `pip install spidev`.
Falls back to a silent no-op if SPI is unavailable, so the GUI still starts.
"""

from __future__ import annotations

import logging
import threading

logger = logging.getLogger(__name__)

WHITE = (255, 255, 255)  # the standard colour


def parse_color(value) -> tuple[int, int, int]:
    """'#rrggbb' or [r, g, b] -> (r, g, b). Raises ValueError."""
    if isinstance(value, str):
        h = value.lstrip("#")
        if len(h) != 6:
            raise ValueError("color must be #rrggbb")
        return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)
    r, g, b = value
    return tuple(max(0, min(255, int(c))) for c in (r, g, b))  # type: ignore[return-value]


class LedRing:
    def __init__(self, count: int = 16, bus: int = 0, device: int = 0) -> None:
        self.count = count
        self.on = False
        self.color = WHITE
        self.brightness = 0.5  # 0..1
        self.available = False
        self._lock = threading.Lock()
        try:
            import spidev

            self._spi = spidev.SpiDev()
            self._spi.open(bus, device)
            self._spi.max_speed_hz = 6_400_000
            self._spi.mode = 0
            self.available = True
        except Exception as exc:  # no spidev, SPI off, no permission
            self._spi = None
            logger.warning("LED ring disabled (%s)", exc)
        self._show()

    def set(self, on=None, color=None, brightness=None) -> dict:
        if on is not None:
            self.on = bool(on)
        if color is not None:
            self.color = parse_color(color)
        if brightness is not None:
            self.brightness = max(0.0, min(1.0, float(brightness)))
        self._show()
        return self.state()

    def white(self) -> dict:
        return self.set(on=True, color=WHITE)

    def state(self) -> dict:
        return {
            "available": self.available,
            "on": self.on,
            "color": list(self.color),
            "brightness": self.brightness,
        }

    def _show(self) -> None:
        if self._spi is None:
            return
        level = self.brightness if self.on else 0.0
        r, g, b = (int(c * level) for c in self.color)
        data = bytearray()
        for _ in range(self.count):
            for byte in (g, r, b):  # WS2812 wants GRB
                for bit in range(7, -1, -1):
                    data.append(0xF8 if (byte >> bit) & 1 else 0xC0)
        data += bytes(64)  # >50 us low = latch
        with self._lock:
            self._spi.writebytes2(data)

    def close(self) -> None:
        if self._spi is not None:
            self.set(on=False)
            self._spi.close()
            self._spi = None
