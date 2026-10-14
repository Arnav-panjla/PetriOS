"""Duration of a synchronized joint-space move."""

from __future__ import annotations

from collections.abc import Sequence

from robot_arm.hardware.servo import ServoCalibration
from robot_arm.motion.interpolators import minimum_duration
from robot_arm.motion.trajectory import Trajectory


class TrajectoryPlanner:
    """Pick one duration for every joint, then build a quintic trajectory.

    soften_ms, when set, replaces the velocity and acceleration limits:
    duration is the largest |delta| * soften_ms / 1000. Zero means a single
    jump to the targets.
    """

    def plan(
        self,
        starts: Sequence[float],
        targets: Sequence[float],
        joints: Sequence[ServoCalibration],
        *,
        soften_ms: float | None = None,
    ) -> Trajectory:
        if not len(starts) == len(targets) == len(joints):
            raise ValueError("starts, targets, and joints must be the same length")
        deltas = [
            target - start for start, target in zip(starts, targets, strict=True)
        ]
        if soften_ms is not None:
            if soften_ms <= 0:
                duration = 0.0
            else:
                duration = max(abs(delta) * soften_ms / 1000.0 for delta in deltas)
        else:
            duration = 0.0
            for delta, joint in zip(deltas, joints, strict=True):
                duration = max(
                    duration,
                    minimum_duration(
                        delta,
                        joint.max_velocity_deg_s,
                        joint.max_acceleration_deg_s2,
                    ),
                )
        return Trajectory(
            starts=tuple(starts),
            targets=tuple(targets),
            duration_s=duration,
        )
