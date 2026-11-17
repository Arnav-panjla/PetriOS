import pytest

from robot_arm.vision.esp32_serial import SerialCamera


class FakeLink:
    def __init__(self):
        self.data = b""

    def write(self, b):
        self.data += b

    def flush(self):
        pass


def make():
    cam = SerialCamera()
    cam._link = FakeLink()
    cam.connected = True
    return cam


def test_brightness_bytes_and_clamp():
    cam = make()
    cam.set_brightness(100)
    cam.set_brightness(250)  # clamped
    cam.set_brightness(-5)
    assert cam._link.data == b"B\xff" + b"B\xff" + b"B\x00"
    assert cam.brightness == 0.0


def test_light_on_resends_brightness():
    cam = make()
    cam.brightness = 40
    cam.set_light(True)
    assert cam._link.data == b"B" + bytes([102]) + b"L"
    cam.set_light(False)
    assert cam._link.data.endswith(b"l")


def test_not_connected_raises():
    with pytest.raises(OSError):
        SerialCamera().set_brightness(10)
