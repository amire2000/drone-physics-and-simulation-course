"""PyBullet plant runner driven directly by Betaflight motor outputs."""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
import time
from typing import Callable

import pybullet as p

from bridge.client import BetaflightBridge
from bridge.control import OuterPositionPid
from bridge.sensors import PyBulletObservationAdapter
from common.simulation import Vehicle, apply_motor_forces

TargetProvider = Callable[[float], tuple[bool, tuple[float, float, float], float] | None]


# Betaflight's QUADX output slots are REAR_R, FRONT_R, REAR_L, FRONT_L.
# The shared PyBullet URDF joints are FRONT_L, FRONT_R, REAR_L, REAR_R.
BETAFLIGHT_TO_URDF_ROTOR = (3, 1, 2, 0)


@dataclass(frozen=True)
class PositionScenario:
    """Home-relative takeoff, precision-hover, and disarm schedule."""

    takeoff_target_m: tuple[float, float, float] = (0.0, 0.0, 3.0)
    hover_target_m: tuple[float, float, float] = (0.0, 0.0, 3.0)
    yaw_target_rad: float = 0.0
    arm_delay_s: float = 1.0
    takeoff_hold_s: float = 5.0
    hover_hold_s: float = 5.0
    disarm_delay_s: float = 1.0


def run_position_scenario(drone: int, vehicle: Vehicle, bridge: BetaflightBridge, controller: OuterPositionPid, scenario: PositionScenario, *, physics_hz: int, rc_hz: int, motor_timeout_s: float, real_time: bool) -> None:
    """Run a fixed takeoff and takeoff-point hover through the shared loop."""
    total_s = scenario.arm_delay_s + scenario.takeoff_hold_s + scenario.hover_hold_s + scenario.disarm_delay_s
    run_bridge_flight(
        drone,
        vehicle,
        bridge,
        controller,
        lambda time_s: (*_phase(scenario, time_s), scenario.yaw_target_rad),
        physics_hz=physics_hz,
        rc_hz=rc_hz,
        motor_timeout_s=motor_timeout_s,
        real_time=real_time,
        max_steps=round(total_s * physics_hz),
    )


def remap_betaflight_motors(normalized: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Place QUADX output slots on the matching PyBullet URDF rotors."""
    urdf_order = [0.0] * len(normalized)
    for betaflight_index, urdf_index in enumerate(BETAFLIGHT_TO_URDF_ROTOR):
        urdf_order[urdf_index] = normalized[betaflight_index]
    return tuple(urdf_order)


# ! Module 08 Betaflight SITL: this cumulative loop replaces local attitude/rate/mixing with SITL motor outputs.
def run_bridge_flight(drone: int, vehicle: Vehicle, bridge: BetaflightBridge, controller: OuterPositionPid, target_provider: TargetProvider, *, physics_hz: int, rc_hz: int, motor_timeout_s: float, real_time: bool, max_steps: int | None = None) -> None:
    """Run PyBullet from live or scheduled targets while SITL owns inner control."""
    dt = 1.0 / physics_hz
    rc_period_s = 1.0 / rc_hz
    next_rc_send_s = 0.0
    home_position, home_orientation = p.getBasePositionAndOrientation(drone)
    home_yaw = p.getEulerFromQuaternion(home_orientation)[2]
    # ! Module 08 home-relative odometry: keep `(0, 0, 0)` at the takeoff base pose.
    adapter = PyBulletObservationAdapter(tuple(home_position))
    controller.reset()
    motor_rpms = [0.0] * 4
    max_rpm = sqrt(vehicle.max_thrust_per_motor_n / vehicle.thrust_coefficient)
    rc_command = controller.update(adapter.read(drone, 0.0, dt), (0.0, 0.0, 0.0), home_yaw, dt, armed=False)

    step = 0
    while p.isConnected() and (max_steps is None or step < max_steps):
        simulation_time_s = step * dt
        target = target_provider(simulation_time_s)
        if target is None:
            break
        armed, target_offset, yaw_offset_rad = target
        # ! Module 08 home-relative odometry: target offsets and feedback share ENU metres.
        target_position = target_offset
        observation = adapter.read(drone, simulation_time_s, dt)
        rc_command = controller.update(observation, target_position, home_yaw + yaw_offset_rad, dt, armed=armed)
        if simulation_time_s + 1e-12 >= next_rc_send_s:
            bridge.send_pilot_command(simulation_time_s, rc_command)
            next_rc_send_s += rc_period_s
        normalized = bridge.latest_motor_command(time.monotonic(), motor_timeout_s)
        # ! Module 08 Betaflight SITL: motor outputs replace the local PID/ADRC mixer.
        urdf_motor_outputs = remap_betaflight_motors(normalized)
        target_rpms = [command * max_rpm for command in urdf_motor_outputs]
        alpha = min(1.0, dt / vehicle.motor_time_constant_s)
        motor_rpms[:] = [actual + alpha * (target - actual) for actual, target in zip(motor_rpms, target_rpms)]
        motor_thrusts = tuple(vehicle.thrust_coefficient * rpm**2 for rpm in motor_rpms)
        apply_motor_forces(drone, vehicle, motor_thrusts, motor_rpms)
        p.stepSimulation()
        bridge.send_observation(adapter.read(drone, simulation_time_s + dt, dt))
        if real_time:
            time.sleep(dt)
        step += 1

    final_time_s = step * dt
    bridge.send_pilot_command(final_time_s, controller.update(adapter.read(drone, final_time_s, dt), (0.0, 0.0, 0.0), home_yaw, dt, armed=False))


def _phase(scenario: PositionScenario, time_s: float) -> tuple[bool, tuple[float, float, float]]:
    """Return whether to arm and which home-relative target applies at this time."""
    if time_s < scenario.arm_delay_s:
        return False, (0.0, 0.0, 0.0)
    if time_s < scenario.arm_delay_s + scenario.takeoff_hold_s:
        return True, scenario.takeoff_target_m
    if time_s < scenario.arm_delay_s + scenario.takeoff_hold_s + scenario.hover_hold_s:
        return True, scenario.hover_target_m
    return False, scenario.hover_target_m
