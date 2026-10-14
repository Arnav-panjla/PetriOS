"""Kinematic extension point. Geometry has not been measured."""

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
    """Placeholder. models/robot.urdf is not a measured kinematic model."""

    def forward(self, joint_deg: Sequence[float]) -> object:
        raise NotImplementedError(
            "Forward kinematics needs measured link lengths. "
            "models/robot.urdf is only a joint-limit placeholder."
        )

    def inverse(self, pose: object) -> Sequence[float]:
        raise NotImplementedError(
            "Inverse kinematics is not implemented. "
            "Use joint-space moves until a measured model exists."
        )
