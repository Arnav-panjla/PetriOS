"""One shared clock for every joint in a move."""

from __future__ import annotations

import logging
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from robot_arm.motion.interpolators import QuinticInterpolator

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Trajectory:
    """Joint-space quintic from starts to targets over one duration."""

    starts: tuple[float, ...]
    targets: tuple[float, ...]
    duration_s: float
    interpolator: QuinticInterpolator = field(default_factory=QuinticInterpolator)

    def __post_init__(self) -> None:
        if len(self.starts) != len(self.targets):
            raise ValueError("starts and targets must have the same length")
        if self.duration_s < 0:
            raise ValueError("duration must be >= 0")

    def sample(self, tau: float) -> list[float]:
        blend = self.interpolator.blend
        return [
            blend(start, target, tau)
            for start, target in zip(self.starts, self.targets, strict=True)
        ]


def run_trajectory(
    trajectory: Trajectory,
    apply: Callable[[Sequence[float]], None],
    *,
    period_s: float,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Sample every period using absolute deadlines from one monotonic t0.

    A late frame takes its angle from the clock instead of replaying skipped
    steps. The last call of apply is always the exact target angles.
    KeyboardInterrupt leaves the last successful apply in place.
    """
    if period_s <= 0:
        raise ValueError("period_s must be positive")
    if trajectory.duration_s <= 1e-9:
        apply(trajectory.targets)
        return

    t0 = clock()
    try:
        while True:
            elapsed = clock() - t0
            if elapsed >= trajectory.duration_s:
                apply(trajectory.targets)
                return
            apply(trajectory.sample(elapsed / trajectory.duration_s))
            elapsed = clock() - t0
            if elapsed >= trajectory.duration_s:
                apply(trajectory.targets)
                return
            # floor, then push the deadline strictly past now. A value like
            # 0.02/0.02 == 0.999999999 makes int() repeat the same slot and
            # the loop never sleeps.
            now = clock()
            step_index = math.floor((now - t0) / period_s) + 1
            deadline = t0 + step_index * period_s
            while deadline <= now:
                step_index += 1
                deadline = t0 + step_index * period_s
            end = t0 + trajectory.duration_s
            if deadline >= end:
                delay = end - now
                if delay > 0:
                    sleep(delay)
                apply(trajectory.targets)
                return
            sleep(deadline - now)
    except KeyboardInterrupt:
        logger.warning(
            "trajectory interrupted; last commanded pulses were sent, "
            "physical position is not known"
        )
        raise
