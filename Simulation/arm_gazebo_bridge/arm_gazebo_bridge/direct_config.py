"""Shared configuration for the Gazebo-only Petri transfer."""

from __future__ import annotations

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from .mapping import ServoMapping

DIRECT_ZERO_DEG = (90.0, 75.0, 58.0, 66.5, 116.25)
DIRECT_SIGN = (1.0, 1.0, -1.0, 1.0, -1.0)
DIRECT_TCP_M = (0.010023, -0.125002, 0.0015)


def repository_root() -> Path:
    share = Path(get_package_share_directory("amr_simulation"))
    return share.parents[3]


def load_ik():
    path = repository_root() / "ARM_control/ik/arm_ik_v2.py"
    spec = importlib.util.spec_from_file_location("petrios_direct_ik", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load IK from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_home() -> list[float]:
    path = repository_root() / "ARM_control/configs/home.txt"
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator and key.strip() == "servo_deg":
            angles = [float(item) for item in value.split()]
            if len(angles) == 6:
                return angles
    raise ValueError(f"missing six servo_deg values in {path}")


def direct_mapping(ik=None) -> ServoMapping:
    # Pivot X coordinates come from robot_arm_gazebo_gui_sync.urdf.xacro.
    return ServoMapping(
        zero_deg=tuple(float(value) for value in ik.URDF_ZERO_SERVO) if ik else DIRECT_ZERO_DEG,
        sign=tuple(float(value) for value in ik.SERVO_SIGN) if ik else DIRECT_SIGN,
        gripper_closed_deg=167.5,
        gripper_open_deg=10.0,
        gripper_max_width_m=0.035,
        finger_pivot_separation_m=0.025679 - (-0.005981),
    )
