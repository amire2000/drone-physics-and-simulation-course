"""Compare 4S and 6S takeoff with the same 1750 KV motors and PWM command."""

import argparse
from dataclasses import dataclass, field
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data

EXAMPLES_ROOT = Path(__file__).resolve().parents[1]
if str(EXAMPLES_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_ROOT))

from common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from common.drone_physics import PWM_MAX, PWM_MIN

GRAVITY_Z = DEFAULT_PHYSICS_SETTINGS.gravity_z_mps2
MAX_THRUST_PER_MOTOR = DEFAULT_DRONE_MODEL.max_thrust_per_motor_n
MOTOR_TIME_CONSTANT = DEFAULT_DRONE_MODEL.motor_time_constant_s
TIME_STEP = DEFAULT_PHYSICS_SETTINGS.time_step_s
URDF_PATH = DEFAULT_DRONE_MODEL.urdf_path

NOMINAL_CELL_VOLTAGE = 3.7
LOAD_RPM_FRACTION = 0.8
BASELINE_KV = 1750
BASELINE_CELLS = 6
DEFAULT_PWM = 1800.0
DEFAULT_SECONDS = 4.0


@dataclass(frozen=True)
class MotorConfiguration:
    """Motor, battery, propeller, and display properties for one drone."""

    name: str
    cells: int
    kv: int
    propeller: str
    color: tuple[float, float, float]


@dataclass
class Telemetry:
    """Time-series measurements collected from one simulated drone."""

    configuration: MotorConfiguration
    time_s: list[float] = field(default_factory=list)
    altitude_m: list[float] = field(default_factory=list)
    vertical_velocity_mps: list[float] = field(default_factory=list)
    rpm: list[float] = field(default_factory=list)
    total_thrust_n: list[float] = field(default_factory=list)
    max_tilt_rad: float = 0.0


CONFIGURATIONS = (
    MotorConfiguration("6S 1750 KV", 6, 1750, "5x4.3x3", (0.15, 0.65, 1.0)),
    MotorConfiguration("4S 1750 KV", 4, 1750, "5x4.3x3", (1.0, 0.55, 0.15)),
)


def nominal_voltage(configuration: MotorConfiguration) -> float:
    """Return nominal pack voltage from the configuration's cell count."""
    return configuration.cells * NOMINAL_CELL_VOLTAGE


def loaded_max_rpm(configuration: MotorConfiguration) -> float:
    """Return the teaching-model RPM after applying a fixed propeller load factor."""
    return configuration.kv * nominal_voltage(configuration) * LOAD_RPM_FRACTION


BASELINE_LOADED_RPM = loaded_max_rpm(CONFIGURATIONS[0])
THRUST_COEFFICIENT = MAX_THRUST_PER_MOTOR / BASELINE_LOADED_RPM**2


def target_rpm(configuration: MotorConfiguration, pwm: float) -> float:
    """Map PWM linearly to a configuration's loaded maximum RPM."""
    throttle = max(0.0, min(1.0, (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)))
    return throttle * loaded_max_rpm(configuration)


def update_rpm(actual_rpm: float, requested_rpm: float) -> float:
    """Apply the shared first-order motor-response delay to one rotor speed."""
    alpha = min(1.0, TIME_STEP / MOTOR_TIME_CONSTANT)
    return actual_rpm + alpha * (requested_rpm - actual_rpm)


def rotor_thrust(rpm: float) -> float:
    """Convert one rotor's RPM into thrust using the calibrated teaching model."""
    return THRUST_COEFFICIENT * rpm**2


def create_world() -> None:
    """Reset a gravity world containing only a plane and the comparison drones."""
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, GRAVITY_Z)
    p.setTimeStep(TIME_STEP)
    p.loadURDF("plane.urdf")


def spawn_drone(configuration: MotorConfiguration, y_position: float) -> int:
    """Load and color one course drone at a separate side-by-side position."""
    drone = p.loadURDF(str(URDF_PATH), (0, y_position, 0.05), flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(drone, -1, linearDamping=0, angularDamping=0)
    p.changeVisualShape(drone, -1, rgbaColor=(*configuration.color, 1.0))
    p.addUserDebugText(
        f"{configuration.name}\n{configuration.propeller}",
        (0.3, y_position, 0.25),
        textColorRGB=configuration.color,
        textSize=1.4,
    )
    return drone


def apply_collective_thrust(drone: int, rpm: float) -> float:
    """Apply equal upward force at all four rotor links and return total thrust."""
    thrust = rotor_thrust(rpm)
    for link_index in range(4):
        p.applyExternalForce(drone, link_index, (0, 0, thrust), (0, 0, 0), p.LINK_FRAME)
    return 4 * thrust


def record(telemetry: Telemetry, now_s: float, drone: int, rpm: float, total_thrust_n: float) -> None:
    """Append one state sample and track the largest roll/pitch angle."""
    position, orientation = p.getBasePositionAndOrientation(drone)
    velocity, _ = p.getBaseVelocity(drone)
    roll, pitch, _ = p.getEulerFromQuaternion(orientation)
    telemetry.time_s.append(now_s)
    telemetry.altitude_m.append(position[2])
    telemetry.vertical_velocity_mps.append(velocity[2])
    telemetry.rpm.append(rpm)
    telemetry.total_thrust_n.append(total_thrust_n)
    telemetry.max_tilt_rad = max(telemetry.max_tilt_rad, abs(roll), abs(pitch))


def save_plot(telemetry: list[Telemetry], path: Path) -> None:
    """Save altitude, vertical-velocity, thrust, and RPM comparison plots."""
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    rpm_axis = axes[2].twinx()
    for item in telemetry:
        color = item.configuration.color
        axes[0].plot(item.time_s, item.altitude_m, color=color, label=item.configuration.name)
        axes[1].plot(item.time_s, item.vertical_velocity_mps, color=color, label=item.configuration.name)
        axes[2].plot(item.time_s, item.total_thrust_n, color=color, label=f"{item.configuration.name} thrust")
        rpm_axis.plot(item.time_s, item.rpm, color=color, linestyle="--", label=f"{item.configuration.name} RPM")
    axes[0].set_ylabel("altitude (m)")
    axes[1].set_ylabel("vertical velocity (m/s)")
    axes[2].set_ylabel("total thrust (N)")
    rpm_axis.set_ylabel("motor RPM")
    axes[2].set_xlabel("time (s)")
    for axis in axes[:2]:
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[2].grid(True, alpha=0.3)
    thrust_lines, thrust_labels = axes[2].get_legend_handles_labels()
    rpm_lines, rpm_labels = rpm_axis.get_legend_handles_labels()
    axes[2].legend(thrust_lines + rpm_lines, thrust_labels + rpm_labels, loc="upper left")
    figure.suptitle("Same PWM: 6S versus 4S with 1750 KV motors")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def run(gui: bool, pwm: float, seconds: float) -> list[Telemetry]:
    """Run both configurations under the same fixed collective PWM command."""
    create_world()
    if gui:
        p.resetDebugVisualizerCamera(65, 45, -25, (0, 0, 45))
    drones = [spawn_drone(configuration, y_position) for configuration, y_position in zip(CONFIGURATIONS, (-1.0, 1.0))]
    telemetry = [Telemetry(configuration) for configuration in CONFIGURATIONS]
    actual_rpms = [0.0, 0.0]
    requested_rpms = [target_rpm(configuration, pwm) for configuration in CONFIGURATIONS]

    for step in range(round(seconds / TIME_STEP)):
        now_s = step * TIME_STEP
        total_thrusts = []
        for index, drone in enumerate(drones):
            actual_rpms[index] = update_rpm(actual_rpms[index], requested_rpms[index])
            total_thrusts.append(apply_collective_thrust(drone, actual_rpms[index]))
        p.stepSimulation()
        for item, drone, rpm, thrust in zip(telemetry, drones, actual_rpms, total_thrusts):
            record(item, now_s, drone, rpm, thrust)
        if gui:
            time.sleep(TIME_STEP)
    return telemetry


def validate(telemetry: list[Telemetry]) -> None:
    """Verify that the 6S configuration produces the expected stronger takeoff."""
    six_s, four_s = telemetry
    assert six_s.altitude_m[-1] > four_s.altitude_m[-1] + 1.0
    assert max(six_s.vertical_velocity_mps) > max(four_s.vertical_velocity_mps)
    assert max(six_s.rpm) > max(four_s.rpm)
    assert max(six_s.total_thrust_n) > max(four_s.total_thrust_n)
    assert max(item.max_tilt_rad for item in telemetry) < 0.05


def main() -> None:
    """Parse command-line settings, run the experiment, print results, and plot."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the PyBullet GUI")
    parser.add_argument("--pwm", type=float, default=DEFAULT_PWM, help="Fixed collective PWM command in microseconds")
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS, help="Simulation duration")
    parser.add_argument("--plot", type=Path, default=Path("outputs/configuration_takeoff_comparison.png"))
    parser.add_argument("--no-plot", action="store_true", help="Do not save the comparison PNG")
    parser.add_argument("--self-check", action="store_true", help="Run a headless verification")
    args = parser.parse_args()
    if not PWM_MIN <= args.pwm <= PWM_MAX:
        parser.error(f"--pwm must be between {PWM_MIN:.0f} and {PWM_MAX:.0f}")
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        telemetry = run(not args.headless and not args.self_check, args.pwm, args.seconds)
        validate(telemetry)
        for item in telemetry:
            print(
                f"{item.configuration.name}: final altitude {item.altitude_m[-1]:.2f} m, "
                f"peak vz {max(item.vertical_velocity_mps):.2f} m/s, "
                f"peak total thrust {max(item.total_thrust_n):.2f} N"
            )
        if not args.no_plot and not args.self_check:
            save_plot(telemetry, args.plot)
            print(f"Saved plot: {args.plot}")
        if args.self_check:
            print("Configuration takeoff comparison self-check passed")
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
