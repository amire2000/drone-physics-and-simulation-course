"""Pure binary encoders and decoders for the pinned Betaflight SITL protocol."""

from __future__ import annotations

import struct
from math import isfinite, sqrt

from bridge.types import FlightObservation, MotorCommand, PilotCommand

FDM_FORMAT = "<18d"
MOTOR_FORMAT = "<4f"
RC_FORMAT = "<d16H"
FDM_PACKET_BYTES = struct.calcsize(FDM_FORMAT)
MOTOR_PACKET_BYTES = struct.calcsize(MOTOR_FORMAT)
RC_PACKET_BYTES = struct.calcsize(RC_FORMAT)


def clamp(value: float, low: float, high: float) -> float:
    """Limit one normalized bridge value to its inclusive range."""
    return max(low, min(high, value))


def pack_fdm(observation: FlightObservation) -> bytes:
    """Encode sensors with reserved-zero velocity/position slots for patched SITL."""
    orientation_wxyz = _pybullet_to_sitl_quaternion(observation.orientation_xyzw)
    # The Gazebo bridge receives gyro in its IMU-sensor frame, but the SITL
    # acceleration driver negates every received axis before the estimator.
    body_rates_frd = _body_rates_to_sitl_imu(observation.body_rates_rad_s)
    specific_force_frd = _specific_force_to_sitl_packet(observation.specific_force_body_mps2)
    # ! Module 08 sensor FDM: local odometry never crosses the SITL boundary.
    # Retain six reserved doubles for the fixed 144-byte receiver layout.
    # The course SITL_EXTERNAL_BAROMETER build reads pressure directly.
    return struct.pack(
        FDM_FORMAT,
        observation.timestamp_s,
        *body_rates_frd,
        *specific_force_frd,
        *orientation_wxyz,
        *(0.0,) * 6,
        observation.pressure_pa,
    )


def pack_rc(timestamp_s: float, command: PilotCommand) -> bytes:
    """Encode normalized pilot intent as the UDP 9004 AETR/AUX packet."""
    # AETR uses 1500 us as neutral; throttle uses 1000 through 2000 us.
    def centered(value: float) -> int:
        return round(1500.0 + 500.0 * clamp(value, -1.0, 1.0))

    channels = [
        centered(command.roll),
        centered(command.pitch),
        round(1000.0 + 1000.0 * clamp(command.throttle, 0.0, 1.0)),
        centered(command.yaw),
        2000 if command.armed else 1000,
        2000 if command.angle_mode else 1000,
    ]
    channels.extend([1000] * (16 - len(channels)))
    return struct.pack(RC_FORMAT, timestamp_s, *channels)


def unpack_motor(packet: bytes, received_at_s: float) -> MotorCommand:
    """Validate and decode four normalized UDP 9002 motor outputs."""
    if len(packet) != MOTOR_PACKET_BYTES:
        raise ValueError(f"Expected {MOTOR_PACKET_BYTES} motor bytes, received {len(packet)}")
    normalized = tuple(float(value) for value in struct.unpack(MOTOR_FORMAT, packet))
    if any(not isfinite(value) or not 0.0 <= value <= 1.0 for value in normalized):
        raise ValueError(f"Motor output is outside [0.0, 1.0]: {normalized}")
    return MotorCommand(normalized, received_at_s)


def _body_rates_to_sitl_imu(vector_flu: tuple[float, float, float]) -> tuple[float, float, float]:
    """Encode FLU body rates in the Gazebo IMU frame consumed by SITL."""
    x, y, z = vector_flu
    return x, -y, -z


def _specific_force_to_sitl_packet(vector_nwu: tuple[float, float, float]) -> tuple[float, float, float]:
    """Negate NWU/FLU specific force because the SITL acceleration driver negates it again."""
    x, y, z = vector_nwu
    return -x, -y, -z


def _pybullet_to_sitl_quaternion(quaternion_xyzw: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """Convert PyBullet XYZW attitude to the pinned SITL_GAZEBO packet convention."""
    x, y, z, w = quaternion_xyzw
    # SITL undoes Rx(pi) conjugation, then applies ENU-to-NWU Rz(pi/2).
    # Invert that path: q_packet = Rx-conjugate(Rz(-pi/2) * q_pybullet).
    half_sqrt = sqrt(0.5)
    rotated_w = half_sqrt * (w + z)
    rotated_x = half_sqrt * (x + y)
    rotated_y = half_sqrt * (y - x)
    rotated_z = half_sqrt * (z - w)
    return rotated_w, rotated_x, -rotated_y, -rotated_z
