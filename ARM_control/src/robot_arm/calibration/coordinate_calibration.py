"""Cylindrical-frame corrections. Zero means no measured correction."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from robot_arm.config import RobotConfig
from robot_arm.kinematics.transforms import Vec3
from robot_arm.utils.errors import ConfigError


@dataclass
class CoordinateCalibration:
    theta_zero_offset_deg: float = 0.0
    theta_direction: int = 1
    r_offset_mm: float = 0.0
    z_offset_mm: float = 0.0
    tool_pitch_offset_deg: float = 0.0
    position_correction_m: Vec3 = (0.0, 0.0, 0.0)
    position_tolerance_mm: float = 2.0
    pitch_tolerance_deg: float = 2.0
    path: Path | None = None

    def set_value(self, name: str, value: float) -> None:
        if name == "theta_direction":
            if value not in (1.0, -1.0):
                raise ConfigError("theta_direction must be +1 or -1")
            self.theta_direction = int(value)
            return
        if name not in {
            "theta_zero_offset_deg",
            "r_offset_mm",
            "z_offset_mm",
            "tool_pitch_offset_deg",
            "position_tolerance_mm",
            "pitch_tolerance_deg",
        }:
            raise ConfigError(f"unknown coordinate calibration {name}")
        setattr(self, name, value)


def load_coordinate_calibration(config_dir: Path | None = None) -> CoordinateCalibration:
    root = config_dir if config_dir is not None else RobotConfig.default_dir()
    path = root / "calibration.yaml"
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    block = loaded.get("coordinate") if isinstance(loaded, dict) else None
    check = loaded.get("ik_check") if isinstance(loaded, dict) else None
    if not isinstance(block, dict) or not isinstance(check, dict):
        raise ConfigError(f"{path} needs coordinate and ik_check sections")
    correction = block.get("position_correction_mm", [0, 0, 0])
    if not isinstance(correction, list) or len(correction) != 3:
        raise ConfigError("position_correction_mm must have 3 values")
    direction = int(block["theta_direction"])
    if direction not in (1, -1):
        raise ConfigError("theta_direction must be +1 or -1")
    return CoordinateCalibration(
        theta_zero_offset_deg=float(block["theta_zero_offset_deg"]),
        theta_direction=direction,
        r_offset_mm=float(block["r_offset_mm"]),
        z_offset_mm=float(block["z_offset_mm"]),
        tool_pitch_offset_deg=float(block["tool_pitch_offset_deg"]),
        position_correction_m=(
            float(correction[0]) / 1000.0,
            float(correction[1]) / 1000.0,
            float(correction[2]) / 1000.0,
        ),
        position_tolerance_mm=float(check["position_mm"]),
        pitch_tolerance_deg=float(check["pitch_deg"]),
        path=path,
    )
