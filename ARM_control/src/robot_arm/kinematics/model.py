"""Kinematic chain from the arm simulation model.

Link offsets and joint axes are in configs/kinematics.yaml and models/robot.urdf.
Forward and inverse solvers are not implemented here.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol


class KinematicModel(Protocol):
    """Future forward and inverse kinematics. Not implemented."""

    def forward(self, joint_deg: Sequence[float]) -> object:
        """Map joint angles to a tool pose once a measured model exists."""

    def inverse(self, pose: object) -> Sequence[float]:
        """Map a tool pose to joint angles once a measured model exists."""


class UncalibratedKinematics:
    """Geometry is known. The solver is not.

    Read configs/kinematics.yaml for the joint offsets and axes.
    """

    def forward(self, joint_deg: Sequence[float]) -> object:
        raise NotImplementedError(
            "Forward kinematics is not implemented. "
            "Joint offsets and axes are in configs/kinematics.yaml."
        )

    def inverse(self, pose: object) -> Sequence[float]:
        raise NotImplementedError(
            "Inverse kinematics is not implemented. "
            "Use the chain in configs/kinematics.yaml to write the equations."
        )
