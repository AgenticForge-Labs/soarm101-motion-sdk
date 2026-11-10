"""Small deterministic joint-stream target follower for the GUI."""

from __future__ import annotations

from math import isfinite
from typing import Mapping


class JointTargetFollower:
    """Shape a replaceable five-joint target into bounded fixed-period samples."""

    def __init__(
        self,
        start: Mapping[str, float],
        *,
        frequency_hz: float,
        max_speed: float,
        max_acceleration: float,
        max_step: float,
    ) -> None:
        limits = (frequency_hz, max_speed, max_acceleration, max_step)
        if not all(isfinite(float(value)) and float(value) > 0 for value in limits):
            raise ValueError("follower limits and frequency must be positive")
        self.position = {name: float(value) for name, value in start.items()}
        if not all(isfinite(value) for value in self.position.values()):
            raise ValueError("follower starting positions must be finite")
        self.velocity = {name: 0.0 for name in self.position}
        self.dt = 1.0 / frequency_hz
        # Leave floating-point headroom so speed * dt never becomes an SDK
        # command-step violation by a final ulp.
        self.max_speed = min(float(max_speed), float(max_step) * float(frequency_hz)) * (1.0 - 1e-12)
        self.max_acceleration = float(max_acceleration)
        self.max_step = float(max_step)

    def step(self, target: Mapping[str, float]) -> dict[str, float]:
        if set(target) != set(self.position):
            raise ValueError("target must contain exactly the initialized joints")
        if not all(isfinite(float(value)) for value in target.values()):
            raise ValueError("follower targets must be finite")
        result: dict[str, float] = {}
        for name, current in self.position.items():
            error = float(target[name]) - current
            snap_velocity = error / self.dt
            if (
                abs(error) <= self.max_step
                and abs(snap_velocity) <= self.max_speed
                and abs(snap_velocity) <= self.max_acceleration * self.dt
                and abs(snap_velocity - self.velocity[name])
                <= self.max_acceleration * self.dt
            ):
                result[name] = float(target[name])
                self.velocity[name] = snap_velocity
                continue
            if abs(error) < 1e-7:
                desired_velocity = 0.0
            else:
                # The square-root braking curve begins deceleration before a target
                # crossing; it also handles a newly replaced target behind current
                # velocity without carrying the old command as backlog.
                direction = 1.0 if error > 0 else -1.0
                # Bound the next speed by the discrete stopping distance: the
                # next sample travels v*dt before another acceleration-limited
                # braking sample can be applied.
                accel_step = self.max_acceleration * self.dt
                braking_speed = max(
                    0.0,
                    (accel_step * accel_step + 2.0 * self.max_acceleration * abs(error)) ** 0.5
                    - accel_step,
                )
                desired_velocity = direction * min(self.max_speed, braking_speed)
            delta_v = max(-self.max_acceleration * self.dt,
                          min(self.max_acceleration * self.dt,
                              desired_velocity - self.velocity[name]))
            velocity = max(-self.max_speed, min(self.max_speed, self.velocity[name] + delta_v))
            delta = max(-self.max_step, min(self.max_step, velocity * self.dt))
            result[name] = current + delta
            self.velocity[name] = velocity
        self.position = result
        return dict(result)

    def settled(self, target: Mapping[str, float], tolerance: float = 0.001) -> bool:
        return all(abs(float(target[name]) - self.position[name]) <= tolerance
                   and abs(self.velocity[name]) <= tolerance / self.dt
                   for name in self.position)
