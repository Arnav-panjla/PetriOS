"""Simulation placeholders. Imported by future backends, not by the CLI."""

from robot_arm.simulation.backend import ArmBackend, UnimplementedSimulation

__all__ = ["ArmBackend", "UnimplementedSimulation"]
