"""Backend-neutral helpers shared by cumulative Module 6 topic loops."""

from dataclasses import dataclass
from math import sqrt
import time

import numpy as np
import pybullet as p

from .pybullet_recording import render_fixed_camera_frame


@dataclass
class ReducedState:
    """Minimal translational state used by the reduced-order topic examples."""

    position_m: np.ndarray
    velocity_mps: np.ndarray


def integrate_reduced_state(state: ReducedState, force_world_n: np.ndarray, mass_kg: float, time_step_s: float) -> np.ndarray:
    """Advance a reduced-order point mass and return its world acceleration."""
    acceleration_mps2 = force_world_n / mass_kg
    state.velocity_mps += acceleration_mps2 * time_step_s
    state.position_m += state.velocity_mps * time_step_s
    return acceleration_mps2


def clamp(value: float, low: float, high: float) -> float:
    """Limit one scalar command to an inclusive range."""
    return max(low, min(high, value))


def thrust_from_pwm(pwm_us: float, max_thrust_per_motor_n: float) -> float:
    """Convert one collective PWM command into a bounded motor thrust request."""
    normalized = clamp((pwm_us - 1000.0) / 1000.0, 0.0, 1.0)
    return max_thrust_per_motor_n * normalized**2


def rpm_from_thrust(thrust_n: float, max_thrust_per_motor_n: float, thrust_coefficient_n_per_rpm2: float) -> float:
    """Convert one bounded motor-thrust request into a target RPM."""
    bounded_thrust = clamp(thrust_n, 0.0, max_thrust_per_motor_n)
    return sqrt(bounded_thrust / thrust_coefficient_n_per_rpm2)


def advance_motor_rpm(actual_rpm: float, target_rpm: float, time_step_s: float, time_constant_s: float) -> float:
    """Advance a first-order motor response by one fixed physics timestep."""
    alpha = min(1.0, time_step_s / time_constant_s)
    return actual_rpm + alpha * (target_rpm - actual_rpm)


def capture_scene_frame(step: int, physics_hz: int, frames: list[np.ndarray] | None, fps: int = 12) -> None:
    """Capture a fixed-camera GIF frame at a reduced rate while physics runs."""
    if frames is not None and step % max(1, round(physics_hz / fps)) == 0:
        frames.append(render_fixed_camera_frame())


def wait_for_exit() -> None:
    """Keep the connected PyBullet GUI open until Q or Esc is pressed."""
    while p.isConnected():
        keys = p.getKeyboardEvents()
        if any(key in (ord("q"), ord("Q"), 27) and state & p.KEY_WAS_TRIGGERED for key, state in keys.items()):
            return
        time.sleep(1 / 60)
