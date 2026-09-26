# Module 9: Learned vertical hover with a NumPy MLP

## Goal

Add a learning module after the current syllabus that introduces an MLP without
using Betaflight. The first example should be as small and understandable as
possible: a fixed 3 m hover in the existing PyBullet drone simulation.

## Control boundary

The learned controller is a **high-level policy**. It receives altitude error
and vertical velocity and returns desired vertical acceleration:

```text
(target altitude - altitude, vertical velocity)
                    ↓
               NumPy MLP
                    ↓
        desired vertical acceleration
                    ↓
existing attitude PID → mixer → motors → PyBullet
```

The existing roll, pitch, and yaw PID controllers keep the drone level. The
MLP does not command individual motor PWM values.

## Teacher data and units

The first trainer uses generated states rather than PyBullet trajectories. It
labels altitude error $e$ and vertical velocity $v_z$ with an explicit
acceleration teacher:

$$
a_{teacher}=\operatorname{clip}(1.2e-1.2v_z,-4.0,4.0)\ \mathrm{m/s^2}.
$$

The MLP learns this acceleration in $\mathrm{m/s^2}$, then the hover example
converts it to total thrust with $T=m(g+a_{desired})$. This is deliberately
different from the existing Module 6 altitude PID, whose output is a force
correction in newtons and is added directly to $mg$.

## Implemented lesson and examples

- Lesson 3 explains PID imitation, the `2 → 8 → 1` network, mean-squared-error
  training, inference, force conversion, and the boundary between learned and
  deterministic control. It includes control-flow diagrams, hands-on work, and
  review quizzes.
- `vertical_acceleration_mlp.py` generates a deterministic state grid, trains
  the NumPy model, validates it, and writes `outputs/vertical_acceleration_mlp.npz`.
- `mlp_pybullet_hover.py` loads those weights and runs a GUI or headless hover.
  `--controller teacher` uses the transparent PD teacher through exactly the
  same physics path for comparison.
- Reuse the Module 6 flight physics, mixer, motor model, and attitude hold;
  do not duplicate them and do not add Betaflight integration.

## First-flight acceptance check

Start the drone level, motionless, and 5 cm above the ground. It must reach
the fixed 3 m target, remain within ±15 cm for three seconds, and never exceed
3.5 m in a headless run.

## Important limitation

This first success case is deliberately idealized: no wind, sensor noise,
randomized initial conditions, or horizontal motion. A policy that succeeds
there may have learned only that scenario. Future work should progress through
randomized recovery, goal-conditioned 3-D motion, domain randomization, and
camera-derived features. Raw RGB images require an image encoder; a plain MLP
is appropriate only after compact features such as optical flow are available.

## Deferred work

Do not add wind, sensor noise, randomized starts, horizontal flight, camera
features, raw RGB learning, or Betaflight integration to this first MLP lesson.
