# Module 6: Autonomous takeoff and precision hover

## By the end, you will be able to

- Convert altitude error into high-level control inputs.
- Map a decision model's output to motor PWM.
- Hold a stable 5 m hover under simulated physical limits.

## Lessons

1. Build closed-loop altitude tracking from spatial displacement.
2. Map live MLP inference through the mixer to safe motor commands.
3. Verify hover while balancing drag, inertia, and battery depletion.

---

## Manual takeoff preview

This is an early capstone preview: a complete 650 g drone body on a ground
plane, with four virtual motors and a single collective PWM slider. It is manual
control, not an autonomous controller yet.

```bash
uv run python examples/06-autonomous-hover/manual_takeoff.py
```

In the PyBullet GUI, raise **Collective PWM (us)** slowly from `1000` toward
`1500`. The live display shows the commanded PWM, actual rotor RPM, individual
motor thrusts, total thrust, attitude, body angular rate, and the drone's
`6.38 N` weight.

| PWM range | Expected behavior |
| --- | --- |
| Below `1500 µs` | Total thrust is below weight; the drone stays on the ground or descends. |
| Near `1500 µs` | Total thrust is about equal to weight; the drone hovers. |
| Above `1500 µs` | Total thrust exceeds weight; the drone takes off. |

### Forces in this preview

- **Gravity** pulls the drone down with `W = mg`.
- **Rotor speed** follows PWM through a short motor-response delay, so an abrupt
  command does not create an impossible instant force jump.
- **Four rotor thrusts** use `F = Kf × RPM²` at their own fixed rotor links.
- **CW/CCW reaction torques** use `τ = Km × RPM²` in opposite pairs and cancel
  when all motors run equally.
- **Aerodynamic drag** opposes body-frame velocity and increases with total
  rotor RPM.

### Attitude hold and force view

The GUI starts with **Attitude hold** enabled. Its IMU-style state read supplies
roll, pitch, yaw, and body angular rate to three PID controllers. The X-frame
mixer keeps all motor forces equal while the drone is level, then makes only
small motor-force differences when it must correct a tilt. If a requested
correction would exceed a motor limit, the mixer scales all correction terms
together instead of clipping one motor and creating an unintended spin. Green
lines above each rotor show the current, delayed thrust force.

Attitude hold prevents uncontrolled rotation; it does **not** create lift. If
you reduce collective PWM below hover, the drone still descends because total
thrust is below weight.

The 1500 µs hover point is a teaching calibration, not propeller test data.
The shared constants in `drone_control.py` are deliberately easy to tune.

For a repeatable terminal run:

```bash
uv run python examples/06-autonomous-hover/manual_takeoff.py --headless --pwm 1550
```

Run the motor-lag and equal-force check with:

```bash
uv run python examples/06-autonomous-hover/manual_takeoff.py --self-check
```

---

## Automatic takeoff, yaw, and landing

The automatic example uses the shared `pid.py` controller for altitude, roll,
pitch, and yaw. It climbs to `3 m`, hovers for two seconds, turns `180°`, and
then lands. Its vertical gains are deliberately conservative: the headless
check limits the peak altitude to `3.25 m`.

```bash
uv run python examples/06-autonomous-hover/auto_takeoff_and_hover.py
```

Use `--headless` for the repeatable check:

```bash
uv run python examples/06-autonomous-hover/auto_takeoff_and_hover.py --headless
```

Later modules will replace collective throttle with individual motor commands,
add battery and wind effects, and close the loop with altitude control.

---

Prerequisite: [Module 5: Battery voltage sag](../05-battery-voltage-sag/index.md).
