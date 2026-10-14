"""Quintic position blend. Velocity and acceleration are zero at both ends."""

from __future__ import annotations

import math

# max ds/dτ at τ = 0.5. s'(τ) = 30τ²(1-τ)².
PEAK_VELOCITY_FACTOR = 1.875

# max |d²s/dτ²| = 10*sqrt(3)/3 at τ = (3 ± sqrt(3))/6.
PEAK_ACCELERATION_FACTOR = 10.0 * math.sqrt(3.0) / 3.0


def clamp_tau(tau: float) -> float:
    return min(1.0, max(0.0, tau))


def quintic(tau: float) -> float:
    """s(τ) = 10τ³ - 15τ⁴ + 6τ⁵, with τ clamped to 0..1."""
    t = clamp_tau(tau)
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    return 10.0 * t3 - 15.0 * t4 + 6.0 * t5


def quintic_velocity(tau: float) -> float:
    """ds/dτ. Divide by the move duration for joint velocity."""
    t = clamp_tau(tau)
    t2 = t * t
    return 30.0 * t2 - 60.0 * t2 * t + 30.0 * t2 * t2


def quintic_acceleration(tau: float) -> float:
    """d²s/dτ². Divide by duration squared for joint acceleration."""
    t = clamp_tau(tau)
    return 60.0 * t - 180.0 * t * t + 120.0 * t * t * t


class QuinticInterpolator:
    """Scalar quintic blend shared by every joint in one move."""

    def position(self, tau: float) -> float:
        return quintic(tau)

    def blend(self, start: float, end: float, tau: float) -> float:
        return start + (end - start) * self.position(tau)


def minimum_duration(
    delta_deg: float,
    max_velocity_deg_s: float,
    max_acceleration_deg_s2: float,
) -> float:
    """Shortest quintic duration that respects this joint's speed limits."""
    distance = abs(delta_deg)
    if distance < 1e-9:
        return 0.0
    by_velocity = PEAK_VELOCITY_FACTOR * distance / max_velocity_deg_s
    by_acceleration = math.sqrt(
        PEAK_ACCELERATION_FACTOR * distance / max_acceleration_deg_s2
    )
    return max(by_velocity, by_acceleration)
