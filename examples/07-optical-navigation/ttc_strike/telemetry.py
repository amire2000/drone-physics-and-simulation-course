"""Telemetry storage and live/final trajectory plots."""

from dataclasses import dataclass, field
from pathlib import Path

from .config import StrikeConfig


@dataclass
class FlightLog:
    time_s: list[float] = field(default_factory=list)
    x_m: list[float] = field(default_factory=list)
    z_m: list[float] = field(default_factory=list)
    vx_mps: list[float] = field(default_factory=list)
    vz_mps: list[float] = field(default_factory=list)

    def append(self, now_s: float, position: tuple[float, float, float], velocity: tuple[float, float, float]) -> None:
        self.time_s.append(now_s)
        self.x_m.append(position[0])
        self.z_m.append(position[2])
        self.vx_mps.append(velocity[0])
        self.vz_mps.append(velocity[2])


def make_plot(config: StrikeConfig):
    import matplotlib.pyplot as plt

    figure, (velocity_axis, path_axis) = plt.subplots(2, 1, figsize=(9, 7))
    vx_line, = velocity_axis.plot([], [], label="vx forward", color="#2563eb")
    vz_line, = velocity_axis.plot([], [], label="vz vertical", color="#dc2626")
    velocity_axis.set(xlabel="time (s)", ylabel="velocity (m/s)", title="World-frame velocity")
    velocity_axis.grid(alpha=0.25)
    velocity_axis.legend()
    path_line, = path_axis.plot([], [], color="#16a34a", label="drone path")
    path_axis.scatter((config.target_face_x_m,), (config.target_center[2],), color="#dc2626", label="cube face")
    path_axis.set(xlabel="world x (m)", ylabel="world z / altitude (m)", title="Diagonal trajectory (x-z)")
    path_axis.grid(alpha=0.25)
    path_axis.legend()
    figure.tight_layout()
    return figure, velocity_axis, path_axis, (vx_line, vz_line, path_line)


def refresh_plot(plot, log: FlightLog) -> None:
    figure, velocity_axis, path_axis, (vx_line, vz_line, path_line) = plot
    vx_line.set_data(log.time_s, log.vx_mps)
    vz_line.set_data(log.time_s, log.vz_mps)
    path_line.set_data(log.x_m, log.z_m)
    for axis in (velocity_axis, path_axis):
        axis.relim()
        axis.autoscale_view()
    figure.canvas.draw_idle()
    figure.canvas.flush_events()


def save_plot(log: FlightLog, config: StrikeConfig, output: Path) -> None:
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    plot = make_plot(config)
    refresh_plot(plot, log)
    plot[0].savefig(output, dpi=140)
    plt.close(plot[0])
