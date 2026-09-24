# Module 9: Learned vertical hover with an MLP

## By the end, you will be able to

- Train a small NumPy MLP to imitate the existing altitude PID.
- Use the learned policy to command vertical acceleration for a fixed 3 m hover.
- Distinguish a successful fixed scenario from evidence that a policy
  generalizes.

---

## Module achievement

This planned module introduces learned control without Betaflight. A small
`2 → 8 → 1` MLP maps altitude error and vertical velocity to a desired vertical
acceleration. The existing attitude PID, motor mixer, actuator model, and
PyBullet simulation remain in charge of low-level stabilization.

The first achievement is a 3 m hover from one clean, level initial condition.
That is intentionally a teaching baseline, not proof of robust flight. Later
work can add randomized starts, 3-D goals, disturbances, and visual features.

---

Prerequisite: [Module 8: Betaflight SITL bridge](../08-betaflight-sitl/index.md).
See the implementation plan at `design/mlp_vertical_hover_module_plan.md`.
