"""Map servo degrees onto URDF joint radians. Defaults are identity, not measured."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from robot_arm.config import RobotConfig
from robot_arm.kinematics.transforms import deg, rad, wrap_pi
from robot_arm.utils.errors import ConfigError, JointLimitError


@dataclass
class ServoMap:
    index: int
    role: str
    urdf_joint: str
    direction: int
    zero_offset_deg: float

    def to_urdf(self, servo_deg: float) -> float:
        return wrap_pi(self.direction * rad(servo_deg - self.zero_offset_deg))

    def to_servo(self, urdf_radians: float) -> float:
        angle = self.zero_offset_deg + self.direction * deg(urdf_radians)
        if angle < -1e-6 or angle > 180.0 + 1e-6:
            raise JointLimitError(
                f"servo {self.index} solution {angle:.2f} deg is outside 0..180"
            )
        return min(180.0, max(0.0, angle))


@dataclass
class JointCalibration:
    servos: tuple[ServoMap, ...]
    path: Path | None = None

    def to_urdf(
        self,
        servo_deg: tuple[float, ...],
    ) -> tuple[float, float, float, float, float]:
        return tuple(
            servo.to_urdf(angle)
            for servo, angle in zip(self.servos[:5], servo_deg, strict=True)
        )  # type: ignore[return-value]

    def to_servo(
        self,
        urdf_rad: tuple[float, float, float, float, float],
        hold: tuple[float, ...],
    ) -> tuple[float, float, float, float, float, float]:
        solved = [
            servo.to_servo(angle)
            for servo, angle in zip(self.servos[:5], urdf_rad, strict=True)
        ]
        gripper = hold[5] if len(hold) > 5 else 0.0
        return (solved[0], solved[1], solved[2], solved[3], solved[4], gripper)


def load_joint_calibration(config_dir: Path | None = None) -> JointCalibration:
    root = config_dir if config_dir is not None else RobotConfig.default_dir()
    path = root / "calibration.yaml"
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    rows = loaded.get("servos") if isinstance(loaded, dict) else None
    if not isinstance(rows, list) or len(rows) != 6:
        raise ConfigError(f"{path} must list 6 servos")
    servos = []
    for row in rows:
        direction = int(row["direction"])
        if direction not in (1, -1):
            raise ConfigError(f"servo {row.get('index')} direction must be +1 or -1")
        servos.append(
            ServoMap(
                index=int(row["index"]),
                role=str(row["role"]),
                urdf_joint=str(row.get("urdf_joint", "")),
                direction=direction,
                zero_offset_deg=float(row["zero_offset_deg"]),
            )
        )
    servos.sort(key=lambda item: item.index)
    if [item.index for item in servos] != list(range(6)):
        raise ConfigError("calibration servos must be indexed 0..5")
    return JointCalibration(servos=tuple(servos), path=path)
