"""Read nominal arm geometry from the URDF. Calibration is applied later."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import yaml

from robot_arm.config import RobotConfig
from robot_arm.kinematics.transforms import Vec3
from robot_arm.utils.errors import ConfigError


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    parent: str
    child: str
    xyz_m: Vec3
    rpy_rad: Vec3
    axis: Vec3
    kind: str


@dataclass(frozen=True)
class ArmGeometry:
    """Five positioning joints plus the tool point, all from the URDF."""

    urdf_path: Path
    joints: tuple[UrdfJoint, ...]
    tool_in_link_5_m: Vec3
    planar: bool
    planar_note: str

    @property
    def shoulder_xyz(self) -> Vec3:
        return self.joints[1].xyz_m

    @property
    def upper_arm_m(self) -> float:
        return abs(self.joints[2].xyz_m[2])

    @property
    def forearm_m(self) -> float:
        return abs(self.joints[3].xyz_m[1])

    @property
    def wrist_offset_m(self) -> Vec3:
        return self.joints[4].xyz_m


def load_geometry(config_dir: Path | None = None) -> ArmGeometry:
    root = config_dir if config_dir is not None else RobotConfig.default_dir()
    spec_path = root / "geometry.yaml"
    if not spec_path.is_file():
        raise ConfigError(f"missing geometry file: {spec_path}")
    with spec_path.open("r", encoding="utf-8") as handle:
        spec = yaml.safe_load(handle)
    if not isinstance(spec, dict):
        raise ConfigError(f"{spec_path} must be a mapping")
    urdf_path = (root / str(spec["urdf"])).resolve()
    parsed = _parse_urdf(urdf_path)
    names = [str(name) for name in spec["positioning_chain"]]
    if len(names) != 5:
        raise ConfigError("positioning_chain must name 5 joints")
    joints = []
    for name in names:
        if name not in parsed:
            raise ConfigError(f"{urdf_path} has no joint {name}")
        joints.append(parsed[name])
    declared = spec.get("extracted_joints", {})
    for joint in joints:
        row = declared.get(joint.name) if isinstance(declared, dict) else None
        if row is not None:
            _require_close(joint.xyz_m, _vec(row["xyz_m"]), f"{joint.name} xyz")
            _require_close(joint.axis, _vec(row["axis"]), f"{joint.name} axis")
    tool = parsed.get("tool0_fixed")
    if tool is None:
        raise ConfigError(f"{urdf_path} has no fixed joint tool0_fixed")
    planar, note = _planar_report(tuple(joints))
    return ArmGeometry(
        urdf_path=urdf_path,
        joints=tuple(joints),
        tool_in_link_5_m=tool.xyz_m,
        planar=planar,
        planar_note=note,
    )


def _parse_urdf(path: Path) -> dict[str, UrdfJoint]:
    if not path.is_file():
        raise ConfigError(f"missing URDF: {path}")
    tree = ET.parse(path)
    found: dict[str, UrdfJoint] = {}
    for node in tree.getroot().findall("joint"):
        name = node.attrib["name"]
        origin = node.find("origin")
        axis = node.find("axis")
        parent = node.find("parent")
        child = node.find("child")
        if parent is None or child is None or origin is None:
            raise ConfigError(f"joint {name} is missing parent, child, or origin")
        found[name] = UrdfJoint(
            name=name,
            parent=parent.attrib["link"],
            child=child.attrib["link"],
            xyz_m=_floats(origin.attrib.get("xyz", "0 0 0")),
            rpy_rad=_floats(origin.attrib.get("rpy", "0 0 0")),
            axis=_floats(axis.attrib["xyz"]) if axis is not None else (0.0, 0.0, 1.0),
            kind=node.attrib.get("type", ""),
        )
    return found


def _planar_report(joints: tuple[UrdfJoint, ...]) -> tuple[bool, str]:
    """The analytic IK expects joint 1 about Z and joints 2-4 about X."""
    problems: list[str] = []
    expected_axes = (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    for joint, axis in zip(joints, expected_axes, strict=True):
        if not _near(joint.axis, axis):
            problems.append(f"{joint.name} axis is {joint.axis}, expected {axis}")
        if not _near(joint.rpy_rad, (0.0, 0.0, 0.0)):
            problems.append(f"{joint.name} rpy is {joint.rpy_rad}")
    upper = joints[2].xyz_m
    forearm = joints[3].xyz_m
    if abs(upper[0]) > 1e-9 or abs(upper[1]) > 1e-9:
        problems.append("upper arm offset is not along local Z")
    if abs(forearm[0]) > 1e-9 or abs(forearm[2]) > 1e-9:
        problems.append("forearm offset is not along local Y")
    if problems:
        return False, "; ".join(problems)
    return True, "joints 2, 3 and 4 are parallel about local X"


def _floats(text: str) -> Vec3:
    parts = [float(item) for item in text.split()]
    if len(parts) != 3:
        raise ConfigError(f"expected 3 floats, got {text!r}")
    return (parts[0], parts[1], parts[2])


def _vec(value: object) -> Vec3:
    if not isinstance(value, list) or len(value) != 3:
        raise ConfigError(f"expected a 3-vector, got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


def _near(left: Vec3, right: Vec3, tol: float = 1e-9) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(left, right, strict=True))


def _require_close(left: Vec3, right: Vec3, label: str) -> None:
    if not _near(left, right, tol=1e-6):
        raise ConfigError(
            f"{label} in geometry.yaml {right} does not match the URDF {left}"
        )
