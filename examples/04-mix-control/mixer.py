"""Pure X-frame motor mixer for Module 04."""

from dataclasses import dataclass

import numpy as np

from controller import clamp


@dataclass(frozen=True)
class Mixer:
    """Allocate collective thrust and body torques to four bounded rotors."""

    positions_m: tuple[tuple[float, float], ...]
    yaw_signs: tuple[int, ...]
    torque_to_thrust_m: float
    max_thrust_per_motor_n: float

    def __post_init__(self) -> None:
        """Precompute the least-squares torque allocation matrix."""
        allocation = np.array([
            [y for _, y in self.positions_m],
            [-x for x, _ in self.positions_m],
            [sign * self.torque_to_thrust_m for sign in self.yaw_signs],
        ])
        object.__setattr__(self, "_inverse", np.linalg.pinv(allocation))

    def mix(self, collective_thrust_n: float, torque_nm: tuple[float, float, float]) -> tuple[float, ...]:
        """Return four motor thrusts, scaling torque corrections before clipping."""
        collective_per_motor = clamp(collective_thrust_n / 4.0, 0.0, self.max_thrust_per_motor_n)
        deltas = tuple(float(value) for value in self._inverse @ np.array(torque_nm))
        scale = 1.0
        for delta in deltas:
            if delta > 0.0:
                scale = min(scale, (self.max_thrust_per_motor_n - collective_per_motor) / delta)
            elif delta < 0.0:
                scale = min(scale, collective_per_motor / -delta)
        return tuple(clamp(collective_per_motor + scale * delta, 0.0, self.max_thrust_per_motor_n) for delta in deltas)
