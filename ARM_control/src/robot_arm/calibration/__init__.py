"""Load and edit calibration.yaml without touching the URDF."""

from __future__ import annotations

from pathlib import Path

import yaml

from robot_arm.calibration.coordinate_calibration import (
    CoordinateCalibration,
    load_coordinate_calibration,
)
from robot_arm.calibration.joint_calibration import JointCalibration, load_joint_calibration
from robot_arm.utils.errors import ConfigError


class CalibrationStore:
    """In-memory calibration with an explicit save back to YAML."""

    def __init__(self, joints: JointCalibration, coordinates: CoordinateCalibration) -> None:
        self.joints = joints
        self.coordinates = coordinates

    @classmethod
    def load(cls, config_dir: Path | None = None) -> CalibrationStore:
        return cls(load_joint_calibration(config_dir), load_coordinate_calibration(config_dir))

    def show(self) -> str:
        lines = ["servo calibration (identity defaults are not measured):"]
        for servo in self.joints.servos:
            lines.append(
                f"  servo_{servo.index} {servo.role:12s} "
                f"direction {servo.direction:+d}  "
                f"zero_offset {servo.zero_offset_deg:g} deg  "
                f"urdf {servo.urdf_joint or '-'}"
            )
        frame = self.coordinates
        lines.append(
            "coordinate: "
            f"theta_zero {frame.theta_zero_offset_deg:g} deg, "
            f"theta_direction {frame.theta_direction:+d}, "
            f"r_offset {frame.r_offset_mm:g} mm, "
            f"z_offset {frame.z_offset_mm:g} mm, "
            f"tool_pitch_offset {frame.tool_pitch_offset_deg:g} deg"
        )
        correction = tuple(item * 1000.0 for item in frame.position_correction_m)
        lines.append(f"position_correction_mm: {correction}")
        lines.append(
            f"ik check: {frame.position_tolerance_mm:g} mm, "
            f"{frame.pitch_tolerance_deg:g} deg"
        )
        return "\n".join(lines)

    def set_value(self, key: str, raw: str) -> None:
        value = float(raw)
        if key.startswith("servo_"):
            servo_name, _, field = key.partition(".")
            index = int(servo_name.split("_", 1)[1])
            servo = self.joints.servos[index]
            if field in {"zero_offset", "zero_offset_deg"}:
                servo.zero_offset_deg = value
            elif field == "direction":
                if value not in (1.0, -1.0):
                    raise ConfigError("direction must be +1 or -1")
                servo.direction = int(value)
            else:
                raise ConfigError(f"unknown servo field {field}")
        elif key.startswith("coordinate."):
            name = key.split(".", 1)[1]
            aliases = {
                "theta_zero_offset": "theta_zero_offset_deg",
                "tool_pitch_offset": "tool_pitch_offset_deg",
                "r_offset": "r_offset_mm",
                "z_offset": "z_offset_mm",
            }
            self.coordinates.set_value(aliases.get(name, name), value)
        else:
            raise ConfigError(f"unknown calibration key {key}")
        self.save()

    def save(self) -> None:
        path = self.joints.path or self.coordinates.path
        if path is None:
            raise ConfigError("calibration has no file to save")
        document = {
            "units": {"angle": "deg", "length": "mm"},
            "servos": [
                {
                    "index": servo.index,
                    "role": servo.role,
                    "urdf_joint": servo.urdf_joint,
                    "direction": servo.direction,
                    "zero_offset_deg": servo.zero_offset_deg,
                }
                for servo in self.joints.servos
            ],
            "coordinate": {
                "theta_zero_offset_deg": self.coordinates.theta_zero_offset_deg,
                "theta_direction": self.coordinates.theta_direction,
                "r_offset_mm": self.coordinates.r_offset_mm,
                "z_offset_mm": self.coordinates.z_offset_mm,
                "tool_pitch_offset_deg": self.coordinates.tool_pitch_offset_deg,
                "position_correction_mm": [
                    item * 1000.0 for item in self.coordinates.position_correction_m
                ],
            },
            "ik_check": {
                "position_mm": self.coordinates.position_tolerance_mm,
                "pitch_deg": self.coordinates.pitch_tolerance_deg,
            },
        }
        path.write_text(
            "# Servo and frame corrections. These are not URDF dimensions.\n"
            "# direction +1 means increasing servo degrees increases the URDF angle.\n"
            "# zero_offset_deg is the servo angle that corresponds to URDF q = 0.\n"
            "# The values below start as an identity map. They are not measured.\n"
            + yaml.safe_dump(document, sort_keys=False),
            encoding="utf-8",
        )
