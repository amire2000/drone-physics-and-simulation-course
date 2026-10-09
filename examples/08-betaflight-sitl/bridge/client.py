"""Non-blocking UDP ownership for the Betaflight SITL data plane."""

from __future__ import annotations

import socket
from bridge.protocol import pack_fdm, pack_rc, unpack_motor
from bridge.types import FlightObservation, MotorCommand, PilotCommand


class BetaflightBridge:
    """Send FDM/RC packets and retain only the freshest valid motor packet."""

    def __init__(self, host: str = "127.0.0.1", *, motor_port: int = 9002, fdm_port: int = 9003, rc_port: int = 9004) -> None:
        """Open bridge sockets without starting or configuring the SITL process."""
        self._destination = (host, fdm_port)
        self._rc_destination = (host, rc_port)
        self._send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._motor_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._motor_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._motor_socket.bind((host, motor_port))
        self._motor_socket.setblocking(False)
        self._latest: MotorCommand | None = None

    def send_observation(self, observation: FlightObservation) -> None:
        """Send one current PyBullet observation to SITL on UDP 9003."""
        self._send_socket.sendto(pack_fdm(observation), self._destination)

    def send_pilot_command(self, timestamp_s: float, command: PilotCommand) -> None:
        """Send one 50 Hz virtual RC command to SITL on UDP 9004."""
        self._send_socket.sendto(pack_rc(timestamp_s, command), self._rc_destination)

    def latest_motor_command(self, now_s: float, timeout_s: float) -> tuple[float, float, float, float]:
        """Drain motor packets and return zeros when the newest output is stale."""
        while True:
            try:
                packet, _ = self._motor_socket.recvfrom(256)
            except BlockingIOError:
                break
            try:
                self._latest = unpack_motor(packet, now_s)
            except ValueError:
                # ! Module 08 Betaflight SITL: invalid motor packets never interrupt physics.
                continue
        if self._latest is None or now_s - self._latest.received_at_s > timeout_s:
            return (0.0, 0.0, 0.0, 0.0)
        return self._latest.normalized

    def close(self) -> None:
        """Release only the bridge-owned UDP sockets."""
        self._send_socket.close()
        self._motor_socket.close()

    def __enter__(self) -> "BetaflightBridge":
        """Allow the bridge to be used as a scoped resource."""
        return self

    def __exit__(self, *_: object) -> None:
        """Close sockets when the scoped bridge exits."""
        self.close()
