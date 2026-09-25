"""PyBullet world lifecycle and visual helpers shared by course examples."""

import pybullet as p
import pybullet_data

from .drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, PhysicsSettings, PhysicsStep
from .pybullet_sensors import body_to_world_vector


def create_world(model: DroneModel = DEFAULT_DRONE_MODEL, settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS) -> int:
    """Reset the PyBullet world, load ground and the course drone, and return its id."""
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, settings.gravity_z_mps2)
    p.setTimeStep(settings.time_step_s)
    p.loadURDF("plane.urdf")
    drone = p.loadURDF(str(model.urdf_path), (0, 0, 0.05), flags=p.URDF_USE_INERTIA_FROM_FILE)
    p.changeDynamics(drone, -1, linearDamping=0, angularDamping=0)
    return drone


def reset_drone(drone: int, position_m: tuple[float, float, float], orientation_quaternion: tuple[float, float, float, float] = (0, 0, 0, 1)) -> None:
    """Reset position, attitude, and velocity for a new deterministic scenario."""
    p.resetBasePositionAndOrientation(drone, position_m, orientation_quaternion)
    p.resetBaseVelocity(drone, (0, 0, 0), (0, 0, 0))


def draw_force_vectors(drone: int, step: PhysicsStep, line_ids: list[int]) -> None:
    """Draw green body-up arrows proportional to each applied rotor thrust."""
    if not p.isConnected():
        return
    try:
        direction = body_to_world_vector(drone, (0.0, 0.0, 1.0))
        for index, thrust in enumerate(step.motor_thrusts_n):
            position = p.getLinkState(drone, index)[0]
            endpoint = tuple(start + axis * thrust * 0.15 for start, axis in zip(position, direction))
            line_ids[index] = p.addUserDebugLine(position, endpoint, (0.1, 0.8, 0.2), lineWidth=3, replaceItemUniqueId=line_ids[index])
    except p.error:
        if p.isConnected():
            raise


def state_text(step: PhysicsStep, stabilized: bool) -> str:
    """Format the latest actuator and rigid-body state for a PyBullet overlay."""
    state = step.state
    roll_pitch_yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
    return (
        f"PWM command: {step.collective_pwm_us:.0f} us\n"
        f"Rotor RPM: {', '.join(f'{value:.0f}' for value in step.motor_rpms)}\n"
        f"Motor thrusts: {', '.join(f'{value:.2f}' for value in step.motor_thrusts_n)} N\n"
        f"Total thrust: {step.total_thrust_n:.2f} N\n"
        f"Altitude: {state.position_m[2]:.2f} m, vertical velocity: {state.linear_velocity_mps[2]:.2f} m/s\n"
        f"Roll/pitch/yaw: {', '.join(f'{value:.2f}' for value in roll_pitch_yaw)} rad\n"
        f"Body rate: {', '.join(f'{value:.2f}' for value in state.angular_velocity_body_rad_s)} rad/s\n"
        f"Attitude hold: {'on' if stabilized else 'off'}"
    )
