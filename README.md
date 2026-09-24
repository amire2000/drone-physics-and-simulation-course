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
1. **Mass and external forces** — gravity, free fall, and
   hover equilibrium.
2. **Physical asset blueprint** — quadcopter URDF design and inertia tensors.
3. **Propeller dynamics and aerodynamics** — PWM-to-thrust, yaw torque, and
   air drag.
4. **Motor mixer and PID control** — X-frame mixing, stabilization, and PWM
   command mapping.
5. **Battery profiles and voltage sag** — LiPo discharge, internal resistance,
   and reduced thrust.
6. **Autonomous takeoff and precision hover** — altitude tracking, live PID
   tuning, noisy altitude measurements, and integrated flight verification.
7. **Forward camera and monocular optical navigation** *(planned)* — render a
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

## Layout

- `docs/` — course site content
- `docs/modules/` — one folder per course module
- `examples/` — runnable examples grouped by module
- `mkdocs.yml` — navigation and theme configuration
- `.vscode/tasks.json` — build and serve tasks
