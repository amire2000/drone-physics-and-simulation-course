# Drone Physics & Simulation

A beginner-friendly MkDocs course for building a quadcopter physics engine in
PyBullet, from external forces to autonomous precision hover.

## About

Learn drone dynamics by building a real-time quadcopter simulator from
scratch with Python, PyBullet, and PID control.

## Course target

Build a mathematically grounded, real-time drone simulation from rigid-body
physics through PID-tuned hover, visual navigation, and learned control. The
track models frame inertia, propeller thrust and torque, aerodynamic drag,
motor mixing, PID stabilization, and battery voltage sag.

## Syllabus

0. **PyBullet setup and GUI** — install the environment and run the first
   plane-and-cube sandbox.
1. **Mass and external forces** — state vectors, gravity, free fall, and
   hover equilibrium.
2. **Physical asset blueprint** — world/body frames, six-DOF rotation,
   URDF design, and inertia tensors.
3. **Propeller dynamics and aerodynamics** — rotor thrust, reaction torque,
   lever-arm torque, PWM mapping, and air drag.
4. **Motor mixer and PID control** — X-frame mixing, roll/pitch/yaw prediction,
   stabilization, wind-relative drag, and PWM command mapping.
5. **Battery profiles and voltage sag** — numerical integration, timestep
   stability, LiPo discharge, and reduced thrust.
6. **Autonomous takeoff and precision hover** — complete engine validation,
   altitude tracking, live PID tuning, and integrated flight verification.
7. **Forward camera and monocular optical navigation** — render a
   body-mounted RGB camera and estimate relative image motion with optical flow.
8. **Betaflight SITL control bridge** *(planned)* — route simulated sensors and
   motor commands through a pinned Betaflight SITL integration.
9. **Learned vertical hover with an MLP** *(planned)* — imitate the altitude
   PID with a small NumPy network while retaining deterministic low-level
   stabilization.

## Roadmap

PyBullet setup → rigid-body basics → realistic URDF → motor/aerodynamic forces
→ PID flight control → battery limits → autonomous hover → optical navigation
→ SITL integration → learned control.

The site expands this path in [docs/roadmap.md](docs/roadmap.md) and the full
module outline in [docs/syllabus.md](docs/syllabus.md).

## Install and run

This project uses [uv](https://docs.astral.sh/uv/).

```bash
uv sync
uv run mkdocs serve
```

Open <http://127.0.0.1:8000/>. To validate the site:

```bash
uv run mkdocs build --strict
```

## Runnable examples

Run every example from the repository root with `uv run python`. GUI examples
open PyBullet; most also provide `--headless` or `--self-check` for a terminal
check.

### Module 0: PyBullet setup

| Example | What it demonstrates | Run |
| --- | --- | --- |
| `sandbox.py` | A plane, a cube, and the basic PyBullet GUI loop. | `uv run python examples/00-pybullet-setup/sandbox.py` |

### Module 1: Mass and forces

| Example | What it demonstrates | Run |
| --- | --- | --- |
| `free_fall.py` | Gravity acting on a falling cube. | `uv run python examples/01-mass-and-forces/free_fall.py` |
| `hover_force.py` | Forces below, equal to, and above weight. | `uv run python examples/01-mass-and-forces/hover_force.py` |
| `velocity_tracking.py` | Estimate gravitational acceleration from vertical velocity. | `uv run python examples/01-mass-and-forces/velocity_tracking.py` |

### Module 2: URDF engine and frames

| Example | What it demonstrates | Run |
| --- | --- | --- |
| `frames_state.py` | World and body frames on a yawed drone. | `uv run python examples/02-urdf-engine/frames_state.py` |
| `inspect_urdf.py` | Links, joints, mass, and inertia in the course URDF. | `uv run python examples/02-urdf-engine/inspect_urdf.py` |
| `inspect_racing_quad.py` | The adapted real racing-quad URDF. | `uv run python examples/02-urdf-engine/inspect_racing_quad.py` |
| `off_axis_torque.py` | Why a force away from the center of mass rotates a body. | `uv run python examples/02-urdf-engine/off_axis_torque.py` |
| `urdf_inertia.py` | Read and verify the drone mass and inertia tensor. | `uv run python examples/02-urdf-engine/urdf_inertia.py` |

### Module 6: Autonomous hover

| Example | What it demonstrates | Run |
| --- | --- | --- |
| `manual_takeoff.py` | Manual collective thrust with shared motor and attitude control. | `uv run python examples/06-autonomous-hover/manual_takeoff.py` |
| `auto_takeoff_and_hover.py` | Automatic 3 m takeoff, 180° yaw turn, hover, and landing. | `uv run python examples/06-autonomous-hover/auto_takeoff_and_hover.py` |
| `pid_tuning_hover.py` | Live altitude-PID tuning with optional barometer noise. | `uv run python examples/06-autonomous-hover/pid_tuning_hover.py` |

### Module 7: Optical navigation

| Example | What it demonstrates | Run |
| --- | --- | --- |
| `forward_camera.py` | A body-mounted RGB camera viewing a red cube. | `uv run python examples/07-optical-navigation/forward_camera.py` |
| `red_target_detector.py` | HSV red detection and a camera-frame bounding box. | `uv run python examples/07-optical-navigation/red_target_detector.py` |
| `ttc_diagonal_strike.py` | Bbox TTC guidance, diagonal flight, impact recording, and telemetry plots. | `uv run python examples/07-optical-navigation/ttc_diagonal_strike.py` |

For the TTC example, use `--no-video` or `--no-plot` to skip outputs, and
`--headless` to verify the complete strike from the terminal.

## Layout

- `docs/` — course site content
- `docs/modules/` — one folder per course module
- `examples/` — runnable examples grouped by module
- `mkdocs.yml` — navigation and theme configuration
- `.vscode/tasks.json` — build and serve tasks
