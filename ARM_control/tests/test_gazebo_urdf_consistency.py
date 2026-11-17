"""Guard the verified GUI/IK and Gazebo transform equivalence."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


REPO = Path(__file__).resolve().parents[2]
KINEMATICS = REPO / "ARM_control" / "configs" / "kinematics.yaml"
XACRO = (
    REPO
    / "Simulation"
    / "robot_arm_description"
    / "urdf"
    / "robot_arm_gazebo_gui_sync.urdf.xacro"
)


def _floats(value: str) -> list[float]:
    return [float(part) for part in value.split()]


def test_positioning_joint_origins_and_axes_match_exactly() -> None:
    configured = yaml.safe_load(KINEMATICS.read_text(encoding="utf-8"))["positioning_joints"]
    root = ET.parse(XACRO).getroot()
    urdf_joints = {element.attrib["name"]: element for element in root.findall("joint")}
    for expected in configured:
        joint = urdf_joints[expected["name"]]
        assert _floats(joint.find("origin").attrib["xyz"]) == pytest.approx(expected["xyz_m"], abs=1e-12)
        assert _floats(joint.find("axis").attrib["xyz"]) == pytest.approx(expected["axis"], abs=1e-12)


def test_tcp_link_uses_existing_ik_offset_defaults() -> None:
    root = ET.parse(XACRO).getroot()
    args = {
        element.attrib["name"]: float(element.attrib["default"])
        for element in root
        if element.tag.endswith("arg") and element.attrib["name"].startswith("tcp_")
    }
    assert [args["tcp_x"], args["tcp_y"], args["tcp_z"]] == pytest.approx(
        [0.010023, -0.125002, 0.0015], abs=1e-12
    )
