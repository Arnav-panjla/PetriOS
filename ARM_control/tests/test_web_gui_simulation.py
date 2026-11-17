"""Safety checks for the browser GUI's simulation-only mode."""

from robot_arm.config import RobotConfig
from robot_arm.control.arm_controller import ArmController
from robot_arm.hardware.driver import NoOpDriver
from robot_arm.interfaces import web_gui


def test_noop_driver_does_not_retain_or_emit_pwm() -> None:
    driver = NoOpDriver()
    arm = ArmController(RobotConfig.load(), driver, sleep=lambda _seconds: None)
    arm.startup_simulation()
    arm.soften_ms = 0
    arm.move_joint(0, 100.0)
    assert not hasattr(driver, "writes")


def test_simulation_only_main_never_constructs_pca9685(monkeypatch) -> None:
    constructed = []

    def forbidden(_config):
        constructed.append(True)
        raise AssertionError("PCA9685 must not be constructed")

    class Server:
        daemon_threads = False

        def __init__(self, _address, _handler):
            pass

        def serve_forever(self):
            pass

        def server_close(self):
            pass

    monkeypatch.setattr(web_gui, "PCA9685Driver", forbidden)
    monkeypatch.setattr(web_gui, "ThreadingHTTPServer", Server)
    assert web_gui.main(["--simulation-only", "--no-camera"]) == 0
    assert constructed == []
