"""Verify origin hover and yaw recovery against a fresh, course-configured SITL."""

from pathlib import Path
import socket
import struct
import subprocess
import tempfile

import pybullet as p

from position_bridge_demo import (
    BetaflightBridge, MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S,
    MOTOR_TIMEOUT_S, MOTOR_YAW_SIGNS, PHYSICS_HZ, RC_HZ, build_outer_pid,
    load_vehicle,
)
from bridge.simulation import run_bridge_flight

ROOT = Path(__file__).resolve().parents[2]


def require_free_ports() -> None:
    """Refuse to compete with an existing simulator or bridge on fixed ports."""
    for kind, port in ((socket.SOCK_DGRAM, 9002), (socket.SOCK_DGRAM, 9003),
                       (socket.SOCK_DGRAM, 9004), (socket.SOCK_STREAM, 5761)):
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.bind(("127.0.0.1", port))


def check_hover() -> None:
    """Hold local XYZ at (0, 0, 3) for 30 s and recover an injected yaw rate."""
    client = p.connect(p.DIRECT)
    try:
        drone, vehicle = load_vehicle(ROOT / "examples/common/assets/full_drone.urdf",
                                      MAX_THRUST_PER_MOTOR_N, MOTOR_TIME_CONSTANT_S, MOTOR_YAW_SIGNS)
        samples = []
        sitl_altitude_samples = []

        def target(time_s: float):
            """Record hover motion and inject +0.5 rad/s around world +Z at 15 s."""
            # ! Module 08 yaw recovery: disturb the plant after a settled takeoff.
            # Positive world +Z yaw is CCW; SITL must apply opposing rotor torque.
            if round(time_s * PHYSICS_HZ) == 15 * PHYSICS_HZ:
                linear, _ = p.getBaseVelocity(drone)
                p.resetBaseVelocity(drone, linearVelocity=linear, angularVelocity=(0.0, 0.0, 0.5))
            if round(time_s * PHYSICS_HZ) == 14 * PHYSICS_HZ:
                sitl_altitude_samples.append(read_sitl_altitude_m())
            position, quaternion = p.getBasePositionAndOrientation(drone)
            if time_s >= 12.0:
                samples.append((position, p.getEulerFromQuaternion(quaternion)))
            if round(time_s * PHYSICS_HZ) % PHYSICS_HZ == 0:
                yaw = p.getEulerFromQuaternion(quaternion)[2]
                yaw_rate = p.getBaseVelocity(drone)[1][2]
                print(f"t={time_s:.0f}s base XYZ={tuple(round(v, 3) for v in position)} "
                      f"yaw={yaw:.3f} rad rate={yaw_rate:.3f} rad/s", flush=True)
            # Allow BOOTGRACE to expire with arm low before requesting takeoff.
            return time_s >= 5.0, (0.0, 0.0, 3.0), 0.0

        with BetaflightBridge() as bridge:
            run_bridge_flight(drone, vehicle, bridge, build_outer_pid(vehicle.mass_kg), target,
                              physics_hz=PHYSICS_HZ, rc_hz=RC_HZ, motor_timeout_s=MOTOR_TIMEOUT_S,
                              real_time=True, max_steps=30 * PHYSICS_HZ)
        # World spawn Z=0.05 m; home-relative target Z=3 m means world Z=3.05 m.
        altitude_error = max(abs(position[2] - 3.05) for position, _ in samples)
        horizontal_error = max((position[0]**2 + position[1]**2)**0.5 for position, _ in samples)
        tilt = max(abs(angle) for _, attitude in samples for angle in attitude[:2])
        final_yaw = samples[-1][1][2]
        print(f"hover: max altitude error={altitude_error:.3f} m, XY error={horizontal_error:.3f} m, "
              f"tilt={tilt:.3f} rad, final yaw={final_yaw:.3f} rad")
        assert altitude_error < 0.3, "Altitude failed to hold after takeoff"
        assert horizontal_error < 0.3, "Drone left the takeoff point"
        assert tilt < 0.2, "Attitude became unstable"
        assert abs(final_yaw) < 0.1, "Yaw did not recover the disturbance"
        print(f"SITL pressure-derived estimated altitude: {sitl_altitude_samples[0]:.3f} m")
        assert 1.0 < sitl_altitude_samples[0] < 5.0, "SITL did not receive the climb's barometer measurement"
    finally:
        p.disconnect(client)


def read_sitl_altitude_m() -> float:
    """Read MSP altitude to check pressure reception while FDM state slots are zero."""
    with socket.create_connection(("127.0.0.1", 5761), timeout=2.0) as connection:
        connection.settimeout(2.0)
        connection.sendall(b"$M<\x00\x6d\x6d")  # MSP_ALTITUDE = 109.
        response = bytearray()
        while len(response) < 5 or len(response) < response[3] + 6:
            chunk = connection.recv(64)
            if not chunk:
                raise RuntimeError("SITL closed the MSP altitude connection")
            response.extend(chunk)
        if response[:3] != b"$M>" or response[4] != 109 or response[3] < 4:
            raise RuntimeError(f"Unexpected MSP altitude response: {response.hex()}")
        checksum = 0
        for value in response[3:6 + response[3]]:
            checksum ^= value
        if checksum:
            raise RuntimeError("MSP altitude checksum mismatch")
        return struct.unpack_from("<i", response, 5)[0] / 100.0


def main() -> None:
    """Provision an isolated EEPROM, run the live regression, and close SITL."""
    require_free_ports()
    binary = ROOT / ".sitl/betaflight-source/obj/main/betaflight_SITL.elf"
    config = ROOT / "examples/08-betaflight-sitl/config/course_angle_mode.config"
    with tempfile.TemporaryDirectory(prefix="course-sitl-hover-") as work_dir:
        with tempfile.TemporaryFile(mode="w+") as log:
            subprocess.run([str(binary), "--config", str(config)], cwd=work_dir,
                           stdout=log, stderr=subprocess.STDOUT, timeout=30, check=True)
            log.seek(0)
            log.truncate()
            process = subprocess.Popen([str(binary), "--ip", "127.0.0.1"], cwd=work_dir,
                                       stdout=log, stderr=subprocess.STDOUT)
            try:
                check_hover()
                print("Live SITL disturbed-hover self-check passed")
            except BaseException:
                log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                process.terminate()
                process.wait(timeout=10)


if __name__ == "__main__":
    main()
