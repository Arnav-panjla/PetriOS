"""Load the arm chain used for kinematics. This does not solve IK."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from robot_arm.config import RobotConfig
from robot_arm.utils.errors import ConfigError


@dataclass(frozen=True)
class JointGeometry:
    """One URDF joint. xyz is the translation before the rotation about axis."""

    name: str
    parent: str
    child: str
    xyz_m: tuple[float, float, float]
    rpy_rad: tuple[float, float, float]
    axis: tuple[float, float, float]


def kinematics_path() -> Path:
    return RobotConfig.default_dir() / "kinematics.yaml"


def load_positioning_joints(path: Path | None = None) -> tuple[JointGeometry, ...]:
    """Return joint_1 .. joint_5 in chain order.

    Each transform is translate(xyz_m) then rotate about axis by q radians.
    q = 0 is the CAD pose in the simulation URDF.
    """
    file_path = path if path is not None else kinematics_path()
    if not file_path.is_file():
        raise ConfigError(f"missing kinematics file: {file_path}")
    with file_path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    rows = loaded.get("positioning_joints") if isinstance(loaded, dict) else None
    if not isinstance(rows, list) or len(rows) != 5:
        raise ConfigError(f"{file_path} must list 5 positioning_joints")
    joints: list[JointGeometry] = []
    for row in rows:
        if not isinstance(row, dict):
            raise ConfigError(f"{file_path} joint entries must be mappings")
        joints.append(
            JointGeometry(
                name=str(row["name"]),
                parent=str(row["parent"]),
                child=str(row["child"]),
                xyz_m=_vec(row["xyz_m"], file_path),
                rpy_rad=_vec(row["rpy_rad"], file_path),
                axis=_vec(row["axis"], file_path),
            )
        )
    return tuple(joints)


def _vec(value: object, path: Path) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ConfigError(f"{path} expected a 3-vector, got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))
