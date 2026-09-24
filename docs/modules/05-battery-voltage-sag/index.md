# Module 5: Battery profiles and voltage sag

## By the end, you will be able to

- Track LiPo state of charge.
- Model voltage sag from internal resistance.
- Reduce available thrust as the battery depletes.
- Explain how timestep choice and integration method affect simulation error.

## Lessons

1. Simulate a LiPo cell from 4.2 V to its 3.7 V nominal floor.
2. Use Ohm's law to calculate voltage sag under current draw.
3. Scale maximum thrust with available voltage.

---

## Numerical integration and timestep

The equations are continuous, but a simulator updates a state at discrete
times. For a fixed step `Δt`, the simplest Euler update is:

```text
v_next = v + a · Δt
p_next = p + v · Δt
```

Semi-implicit Euler updates velocity first and then uses the new velocity for
position. It is often more stable for falling or thrust-driven motion. Higher
order methods such as RK4 can reduce integration error, but cost more force
evaluations per step.

```mermaid
flowchart LR
    command[Motor commands] --> forces[Sum gravity, thrust, drag, wind]
    forces --> acceleration[Linear and angular acceleration]
    acceleration --> integrate[Integrate for Δt]
    integrate --> state[New position, velocity, attitude, rate]
    state --> forces
```

### Timestep experiment

Use the same constant-force experiment with `Δt = 0.1`, `0.01`, and `0.001 s`.
Plot position and velocity on one graph. Look for numerical drift, overshoot,
or instability as the timestep becomes large. Then compare Euler and
semi-implicit Euler before changing the motor or battery model.

The real PyBullet examples use a fixed `1/240 s` physics step. The controller
can run less often, but every force and torque must be applied consistently to
the physics timestep.

---

Prerequisite: [Module 4: Motor mixer and PID](../04-motor-mixer-pid/index.md). Next: [Module 6: Autonomous hover](../06-autonomous-hover/index.md).
