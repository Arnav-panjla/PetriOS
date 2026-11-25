from pathlib import Path

import pytest

pytest.importorskip("cv2")
pytest.importorskip("numpy")

from robot_arm.vision.webcam import DETECT_PATH, WebCam, load_detector  # noqa: E402

SAMPLE = DETECT_PATH.parent / "sample" / "s1.png"


def test_analyze_sample_plate():
    import cv2

    res = load_detector().analyze(cv2.imread(str(SAMPLE)))
    assert res["dish"][2] > 0 and res["mm_per_px"] > 0
    assert len(res["disks"]) >= 1


def test_capture_on_sample_frame():
    import cv2

    cam = WebCam("/dev/nonexistent")
    with pytest.raises(ValueError):
        cam.capture()  # no frame yet
    cam._raw = cv2.imread(str(SAMPLE))
    r = cam.capture()
    assert r["disks"] and cam.result_jpeg()


def test_no_device_does_not_crash():
    cam = WebCam("/dev/nonexistent")
    assert not cam.connected and cam.fps == 0.0
    assert cam.latest() is None


def test_params_and_manual_dish():
    import cv2

    cam = WebCam("/dev/nonexistent")
    cam._raw = cv2.imread(str(SAMPLE))
    cam.capture()
    d = cam.result["dish"]
    cam.params = {"DISK_MIN_BRIGHTNESS": 250.0}  # nothing passes: no disks
    assert cam.redetect()["disks"] == []
    cam.params = {}
    cam.manual_dish = [d[0], d[1], d[2] * 0.9]
    r = cam.redetect()
    assert abs(r["dish"][2] - d[2] * 0.9) < 1e-6 and r["disks"]
    assert cam.spec()
