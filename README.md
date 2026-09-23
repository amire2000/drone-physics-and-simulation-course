# Drone Physics & Simulation

A beginner-friendly MkDocs course for building a quadcopter physics engine in
PyBullet, from external forces to autonomous precision hover.

## About

Learn drone dynamics by building a real-time quadcopter simulator from
scratch with Python, PyBullet, and PID control.

## Course target

Build a mathematically grounded, real-time drone simulation and connect a
high-level decision model to low-level motor control. The capstone is a drone
that takes off and holds a stable 5 m hover while modeling frame inertia,
propeller thrust and torque, aerodynamic drag, PID stabilization, and battery
voltage sag.

## Syllabus

1. **PyBullet setup and GUI** — install the environment and run the first
   plane-and-cube sandbox.
2. **Mass and external forces** — gravity, free fall, and
   hover equilibrium.
3. **Physical asset blueprint** — quadcopter URDF design and inertia tensors.
4. **Propeller dynamics and aerodynamics** — PWM-to-thrust, yaw torque, and
   air drag.
5. **Motor mixer and PID control** — X-frame mixing, stabilization, and PWM
   command mapping.
6. **Battery profiles and voltage sag** — LiPo discharge, internal resistance,
   and reduced thrust.
7. **Autonomous takeoff and hover** — altitude tracking and integrated
   verification at 5 m.

## Roadmap

PyBullet setup → rigid-body basics → realistic URDF → motor/aerodynamic forces
→ PID flight control → battery limits → autonomous takeoff and precision hover.

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
