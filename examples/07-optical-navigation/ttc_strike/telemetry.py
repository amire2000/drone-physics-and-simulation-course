"""Telemetry storage and live/final trajectory plots."""

from dataclasses import dataclass, field
from math import degrees
from pathlib import Path

from .config import StrikeConfig
from .guidance import GuidanceCommand


@dataclass
class FlightLog:
    time_s: list[float] = field(default_factory=list)
    x_m: list[float] = field(default_factory=list)
    z_m: list[float] = field(default_factory=list)
    vx_mps: list[float] = field(default_factory=list)
    vz_mps: list[float] = field(default_factory=list)
    command_vx_mps: list[float] = field(default_factory=list)
    command_vz_mps: list[float] = field(default_factory=list)
    command_altitude_m: list[float] = field(default_factory=list)
    command_thrust_n: list[float] = field(default_factory=list)
    command_pitch_deg: list[float] = field(default_factory=list)

    def append(self, now_s: float, position: tuple[float, float, float], velocity: tuple[float, float, float], command: GuidanceCommand) -> None:
        self.time_s.append(now_s)
        self.x_m.append(position[0])
        self.z_m.append(position[2])
        self.vx_mps.append(velocity[0])
        self.vz_mps.append(velocity[2])
        trajectory = command.trajectory
        self.command_vx_mps.append(trajectory.forward_velocity_mps if trajectory else float("nan"))
        self.command_vz_mps.append(trajectory.vertical_velocity_mps if trajectory else float("nan"))
        self.command_altitude_m.append(trajectory.altitude_target_m if trajectory else float("nan"))
        self.command_thrust_n.append(command.thrust_n)
        self.command_pitch_deg.append(degrees(command.pitch_target_rad))


@dataclass
class TelemetryPlot:
    figure: object
    velocity_axis: object
    path_axis: object
    trajectory_axis: object
    trajectory_altitude_axis: object
    guidance_axis: object
    pitch_axis: object
    lines: tuple[object, ...]


def make_plot(config: StrikeConfig) -> TelemetryPlot:
    import matplotlib.pyplot as plt

    figure, (velocity_axis, path_axis, trajectory_axis, guidance_axis) = plt.subplots(4, 1, figsize=(10, 11))
    vx_line, = velocity_axis.plot([], [], label="vx forward", color="#2563eb")
    vz_line, = velocity_axis.plot([], [], label="vz vertical", color="#dc2626")
    velocity_axis.set(xlabel="time (s)", ylabel="velocity (m/s)", title="Measured world-frame velocity")
    velocity_axis.grid(alpha=0.25)
    velocity_axis.legend()

    path_line, = path_axis.plot([], [], color="#16a34a", label="drone path")
    path_axis.scatter((config.target_face_x_m,), (config.target_center[2],), color="#dc2626", label="cube face")
    path_axis.set(xlabel="world x (m)", ylabel="world z / altitude (m)", title="Measured diagonal path (x-z)")
    path_axis.grid(alpha=0.25)
    path_axis.legend()

    command_vx_line, = trajectory_axis.plot([], [], label="command vx", color="#2563eb")
    command_vz_line, = trajectory_axis.plot([], [], label="command vz", color="#dc2626")
    trajectory_altitude_axis = trajectory_axis.twinx()
    command_altitude_line, = trajectory_altitude_axis.plot([], [], "--", label="altitude target", color="#16a34a")
    trajectory_axis.set(xlabel="time (s)", ylabel="velocity command (m/s)", title="TrajectoryCommand")
    trajectory_altitude_axis.set_ylabel("altitude target (m)")
    trajectory_axis.grid(alpha=0.25)
    trajectory_axis.legend((command_vx_line, command_vz_line, command_altitude_line), ("command vx", "command vz", "altitude target"), loc="upper left")

    thrust_line, = guidance_axis.plot([], [], label="collective thrust", color="#7c3aed")
    pitch_axis = guidance_axis.twinx()
    pitch_line, = pitch_axis.plot([], [], label="pitch target", color="#f97316")
    guidance_axis.set(xlabel="time (s)", ylabel="GuidanceCommand thrust (N)", title="GuidanceCommand")
    pitch_axis.set_ylabel("pitch target (deg)")
    guidance_axis.grid(alpha=0.25)
    guidance_axis.legend((thrust_line, pitch_line), ("collective thrust", "pitch target"), loc="upper left")
    figure.tight_layout()
    return TelemetryPlot(figure, velocity_axis, path_axis, trajectory_axis, trajectory_altitude_axis, guidance_axis, pitch_axis, (vx_line, vz_line, path_line, command_vx_line, command_vz_line, command_altitude_line, thrust_line, pitch_line))


def refresh_plot(plot: TelemetryPlot, log: FlightLog) -> None:
    vx_line, vz_line, path_line, command_vx_line, command_vz_line, command_altitude_line, thrust_line, pitch_line = plot.lines
    vx_line.set_data(log.time_s, log.vx_mps)
    vz_line.set_data(log.time_s, log.vz_mps)
    path_line.set_data(log.x_m, log.z_m)
    command_vx_line.set_data(log.time_s, log.command_vx_mps)
    command_vz_line.set_data(log.time_s, log.command_vz_mps)
    command_altitude_line.set_data(log.time_s, log.command_altitude_m)
    thrust_line.set_data(log.time_s, log.command_thrust_n)
    pitch_line.set_data(log.time_s, log.command_pitch_deg)
    for axis in (plot.velocity_axis, plot.path_axis, plot.trajectory_axis, plot.trajectory_altitude_axis, plot.guidance_axis, plot.pitch_axis):
        axis.relim()
        axis.autoscale_view()
    plot.figure.canvas.draw_idle()
    plot.figure.canvas.flush_events()


def move_plot_window(plot: TelemetryPlot, position_px: tuple[int, int]) -> None:
    """Place the Matplotlib window on Tk or Qt without coupling to one backend."""
    window = plot.figure.canvas.manager.window
    x, y = position_px
    if hasattr(window, "wm_geometry"):  # TkAgg
        window.wm_geometry(f"+{x}+{y}")
    elif hasattr(window, "move"):  # QtAgg
        window.move(x, y)


def save_plot(log: FlightLog, config: StrikeConfig, output: Path) -> None:
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    plot = make_plot(config)
    refresh_plot(plot, log)
    plot.figure.savefig(output, dpi=140)
    plt.close(plot.figure)
