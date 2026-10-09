"""Pack and validate the Betaflight 2026.6.2 SITL UDP packet layouts."""

from __future__ import annotations

import struct


FDM_FORMAT = "<18d"
MOTOR_FORMAT = "<4f"
RC_FORMAT = "<d16H"
FDM_PACKET_BYTES = 144
MOTOR_PACKET_BYTES = 16
RC_PACKET_BYTES = 40


def pack_fdm_packet(
    timestamp_s: float,
    angular_velocity_body_rad_s: tuple[float, float, float],
    specific_force_body_mps2: tuple[float, float, float],
    orientation_wxyz: tuple[float, float, float, float],
    pressure_pa: float,
) -> bytes:
    """Pack sensors and six reserved zeros for the course's patched SITL."""
    return struct.pack(
        FDM_FORMAT,
        timestamp_s,
        *angular_velocity_body_rad_s,
        *specific_force_body_mps2,
        *orientation_wxyz,
        *(0.0,) * 6,
        pressure_pa,
    )


def unpack_motor_packet(packet: bytes) -> tuple[float, float, float, float]:
    """Validate and decode four normalized Betaflight motor outputs from UDP 9002."""
    if len(packet) != MOTOR_PACKET_BYTES:
        raise ValueError(f"Expected {MOTOR_PACKET_BYTES} motor bytes, received {len(packet)}")
    motors = struct.unpack(MOTOR_FORMAT, packet)
    if any(not 0.0 <= motor <= 1.0 for motor in motors):
        raise ValueError(f"Motor output is outside the normal [0.0, 1.0] range: {motors}")
    return motors


def pack_rc_packet(timestamp_s: float, channels_us: tuple[int, ...]) -> bytes:
    """Pack sixteen AETR/AUX virtual-RC channels for UDP port 9004."""
    if len(channels_us) != 16:
        raise ValueError(f"Expected 16 RC channels, received {len(channels_us)}")
    if any(not 750 <= channel <= 2250 for channel in channels_us):
        raise ValueError("RC channels must use plausible microsecond values from 750 through 2250")
    return struct.pack(RC_FORMAT, timestamp_s, *channels_us)


def main() -> None:
    """Run deterministic layout checks before connecting a future bridge to SITL."""
    fdm = pack_fdm_packet(
        1.25,
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 9.81),
        (1.0, 0.0, 0.0, 0.0),
        101_325.0,
    )
    rc = pack_rc_packet(1.25, (1500, 1500, 1000, 1500) + (1000,) * 12)
    motors = unpack_motor_packet(struct.pack(MOTOR_FORMAT, 0.0, 0.25, 0.5, 1.0))
    assert len(fdm) == FDM_PACKET_BYTES
    assert struct.unpack(FDM_FORMAT, fdm)[11:17] == (0.0,) * 6
    assert len(rc) == RC_PACKET_BYTES
    assert motors == (0.0, 0.25, 0.5, 1.0)
    print(f"FDM packet: {len(fdm)} bytes ({FDM_FORMAT})")
    print(f"RC packet: {len(rc)} bytes ({RC_FORMAT})")
    print(f"Motor packet: {MOTOR_PACKET_BYTES} bytes ({MOTOR_FORMAT})")
    print("Protocol self-check passed")


if __name__ == "__main__":
    main()
