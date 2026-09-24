# Module 4: Motor mixer and PID control

## By the end, you will be able to

- Convert flight commands into X-frame motor outputs.
- Stabilize attitude with a rate PID controller.
- Map high-level targets to 1000–2000 PWM signals.
- Predict how motor-pair changes produce roll, pitch, and yaw.
- Explain drag using velocity relative to the wind.

## Lessons

1. Build an X-frame motor mixer.
2. Implement a closed-loop rate PID controller.
3. Inject normalized joystick-like commands into PWM motor outputs.

---

## X-frame mixing

The mixer converts one collective command and three body-torque commands into
four rotor thrusts. In abstract form:

```text
[T1, T2, T3, T4] = mixer([collective, τroll, τpitch, τyaw])
```

The exact signs depend on motor numbering, but the invariants are constant:
equal motor values produce collective thrust, opposite arm differences produce
roll/pitch torque, and CW/CCW differences produce yaw torque. The shared mixer
scales corrections together when a motor reaches its limit so one clipped motor
does not create an unintended rotation.

### Mixer experiment

Use the manual takeoff force display and change one motor pair at a time. Before
running, predict the sign of the attitude change. Record the four thrusts and
compare the observed roll, pitch, or yaw direction with the prediction.

---

## Relative air velocity and drag

Drag depends on air moving across the vehicle, not only on the vehicle's world
velocity. Define:

```text
v_air = v_drone - v_wind
```

For a simple direction-aligned model, the drag magnitude is

```text
F_D = 1/2 · ρ · C_D · A · |v_air|²
```

and the force points opposite `v_air`. Air density `ρ`, drag coefficient `C_D`,
and projected area `A` are teaching parameters rather than a CFD model.

### Crosswind experiment

Run the same forward or falling trajectory twice: once with zero wind and once
with a `5 m/s` lateral wind. Compare terminal velocity and lateral displacement.
Explain why adding an arbitrary constant force would give the wrong result when
the drone changes direction.

---

Prerequisite: [Module 3: Propeller aerodynamics](../03-propeller-aerodynamics/index.md). Next: [Module 5: Battery voltage sag](../05-battery-voltage-sag/index.md).
