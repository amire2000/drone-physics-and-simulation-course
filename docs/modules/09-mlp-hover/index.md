# Module 9: Learned vertical hover with an MLP

This planned module starts with one small NumPy neural network, then connects
that idea to a drone's vertical-acceleration decision.

## Module achievement

By the end of the planned module, a small NumPy $2 \rightarrow 8 \rightarrow 1$
network will imitate a visible PD vertical-acceleration rule for a fixed 3 m
hover. The existing attitude PID, motor mixer, and PyBullet physics remain
responsible for low-level stabilization.

The first hover is a teaching baseline, not proof of robust flight. Later work
can add varied starts, disturbances, and richer sensor inputs.

---

## Lessons

1. [Build an MLP from scratch with NumPy](01-numpy-mlp-basics/index.md) — learn
   why a hidden layer helps, then implement, train, and validate a small MLP.
2. [A tiny MLP learns one drone decision](01-mlp-introduction/index.md) — map
   the same idea to altitude error, vertical velocity, and flight physics.
3. [From an MLP to vertical acceleration](02-vertical-acceleration-practice/index.md) —
   follow the safe path from generated teacher data to an offline-validated
   PyBullet hover.
4. [Read the PyBullet MLP hover code](03-pybullet-mlp-code/index.md) — trace
   acceleration through thrust, PWM, attitude hold, and the shared physics
   engine.

---

Prerequisite: [Module 8: Betaflight SITL bridge](../08-betaflight-sitl/index.md).
The implementation roadmap is recorded in `design/mlp_vertical_hover_module_plan.md`.
