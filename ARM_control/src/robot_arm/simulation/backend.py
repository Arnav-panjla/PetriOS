"""Simulation backend placeholder. Gazebo and ros2_control are not wired up."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol


class ArmBackend(Protocol):
    """Hardware or simulator that accepts the same joint commands."""

    def command_joints(self, degrees: Sequence[float]) -> None:
        """Send one synchronized joint command."""


class UnimplementedSimulation:
    """Stand-in for a future Gazebo backend."""

    def command_joints(self, degrees: Sequence[float]) -> None:
        raise NotImplementedError(
            "Simulation is not implemented. Use MockDriver in tests "
            "or PCA9685Driver on the Pi."
        )
