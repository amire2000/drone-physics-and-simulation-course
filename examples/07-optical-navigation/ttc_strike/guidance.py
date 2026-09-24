"""Flight phases and high-level pitch/thrust guidance."""

from dataclasses import dataclass
from enum import Enum

from common.pid import PID

from .config import StrikeConfig
from .sensing import BarometerReading
from .trajectory import DiagonalTrajectory, TrajectoryCommand
from .ttc import TtcObservation


class FlightPhase(str, Enum):
    TAKEOFF = "takeoff"
    TRACK = "track"
    COMMIT = "commit"
    ABORT = "abort"


@dataclass(frozen=True)
class GuidanceInput:
    now_s: float
    barometer: BarometerReading
    observation: TtcObservation | None
    last_observation: TtcObservation | None
    target_visible: bool
    commit_ready: bool


@dataclass(frozen=True)
class GuidanceCommand:
    """High-level command consumed by ``StrikeSimulation``.

    | Field | Simulation use | Effect |
    | --- | --- | --- |
    | ``thrust_n`` | Divided across four motors, converted to PWM, then passed to ``step_drone``. | Supplies collective lift. |
    | ``pitch_target_rad`` | Passed to ``attitude_torque``. | The attitude PID tilts the drone along the path. |
    | ``phase`` | Checked for abort and shown in the camera/summary. | Selects normal, commit, or safe-stop behavior. |
    | ``trajectory`` | Sent to annotations and telemetry only. | Makes desired ``vx``, ``vz``, and altitude observable. |
    | ``reset_ttc`` | Resets the bbox tracker after takeoff. | Removes vertical-ascent image scale. |
    | ``commit_expired`` | Ends a missed terminal approach. | Prevents indefinite command holding. |
    """

    phase: FlightPhase
    thrust_n: float
    pitch_target_rad: float
    trajectory: TrajectoryCommand | None
    reset_ttc: bool = False
    commit_expired: bool = False


class StrikeGuidance:
    """Translate sensor observations into pitch and collective-thrust commands."""

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.phase = FlightPhase.TAKEOFF
        self.trajectory = DiagonalTrajectory(config)
        self.altitude_pid = PID(*config.altitude_pid_gains, integral_limit=config.altitude_integral_limit)
        self.forward_pid = PID(*config.forward_pid_gains)
        self.vertical_velocity_pid = PID(*config.vertical_velocity_pid_gains)
        self.last_command = GuidanceCommand(self.phase, config.hover_thrust_n, 0.0, None)
        self.commit_deadline_s: float | None = None

    def update(self, data: GuidanceInput) -> GuidanceCommand:
        """Advance the flight phase and return one high-level control command.

        Flow: climb with altitude PID until the drone is settled at takeoff
        height; track when a red bbox supplies TTC; hold the final valid command
        if a large target disappears; otherwise abort and level the drone. The
        returned collective thrust and pitch target are later converted to motor
        PWM and torque by the PyBullet adapter.
        """
        if self.phase == FlightPhase.TAKEOFF:
            # Takeoff is intentionally level: altitude PID supplies only the
            # collective force needed to reach and settle at the start height.
            thrust = self.config.hover_thrust_n + self.altitude_pid.update(self.config.takeoff_altitude_m - data.barometer.altitude_m, data.barometer.vertical_velocity_mps)
            command = GuidanceCommand(self.phase, thrust, 0.0, None)
            ready = data.barometer.altitude_m > self.config.takeoff_altitude_m - self.config.takeoff_altitude_tolerance_m
            stable = abs(data.barometer.vertical_velocity_mps) < self.config.takeoff_velocity_tolerance_mps
            if ready and stable and data.last_observation:
                # Discard takeoff image scale; TTC must start from forward
                # closure, not from the cube growing during vertical ascent.
                self.phase = FlightPhase.TRACK
                command = GuidanceCommand(self.phase, thrust, 0.0, None, reset_ttc=True)
            self.last_command = command
            return command

        if self.phase == FlightPhase.TRACK and not data.target_visible:
            if data.commit_ready and data.last_observation:
                # The target was already large, so a transient image loss near
                # contact must not make the drone cancel its terminal approach.
                self.phase = FlightPhase.COMMIT
                self.commit_deadline_s = data.now_s + data.last_observation.ttc_s + self.config.commit_timeout_margin_s
            else:
                # Before commit, target loss is unsafe: neutralize pitch and
                # use the altitude loop to hold the current vertical motion.
                self.phase = FlightPhase.ABORT

        if self.phase == FlightPhase.COMMIT:
            # Reuse the last valid pitch/thrust pair; no fresh vision command
            # is trusted after the target has left the image.
            return GuidanceCommand(
                self.phase,
                self.last_command.thrust_n,
                self.last_command.pitch_target_rad,
                self.last_command.trajectory,
                commit_expired=data.now_s > (self.commit_deadline_s or data.now_s),
            )

        if self.phase == FlightPhase.ABORT:
            # Hold the current altitude by commanding zero altitude error and
            # remove the forward pitch request.
            thrust = self.config.hover_thrust_n + self.altitude_pid.update(0.0, data.barometer.vertical_velocity_mps)
            return GuidanceCommand(self.phase, thrust, 0.0, None)

        # A valid TTC range updates the diagonal path. Before the first valid
        # growth estimate, use the planned initial range to begin moving.
        trajectory = self.trajectory.command(data.observation.range_m if data.observation else self.config.initial_range_m)
        if data.observation:
            # Forward-speed error adjusts the nominal path pitch, then clamps
            # it so this first POC never asks for more than the allowed tilt.
            correction = self.forward_pid.update(trajectory.forward_velocity_mps - data.observation.forward_velocity_mps, 0.0)
            pitch = max(0.0, min(self.config.max_pitch_rad, self.config.max_pitch_rad + correction))
        else:
            pitch = self.config.max_pitch_rad
        # Blend altitude error into desired vertical velocity, then let the
        # vertical-velocity PID turn that target into collective thrust.
        corrected_vz = max(self.config.terminal_vz_mps, min(3.0, trajectory.vertical_velocity_mps + self.config.vertical_position_correction * (trajectory.altitude_target_m - data.barometer.altitude_m)))
        thrust = self.config.hover_thrust_n + self.vertical_velocity_pid.update(corrected_vz - data.barometer.vertical_velocity_mps, 0.0)
        command = GuidanceCommand(self.phase, thrust, pitch, trajectory)
        self.last_command = command
        return command
