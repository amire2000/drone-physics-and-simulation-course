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

## Planned lesson and examples

- Create `docs/modules/09-mlp-hover/index.md` when implementation begins.
  Explain PID imitation, the `2 → 8 → 1` network, mean-squared-error training,
  model inference, and the boundary between learned and deterministic control.
  Include a control-flow drawing, hands-on exercise, and review quiz.
- Add `examples/09-mlp-hover/train_vertical_hover_mlp.py`. Generate a
  deterministic grid of altitude errors and vertical velocities, label it using
  the existing altitude PID's desired vertical acceleration, train the NumPy
  network, and write `vertical_hover_mlp.npz`.
- Add `examples/09-mlp-hover/mlp_hover.py`. Load the saved weights and run the
  hover in the GUI or headless mode. Commit the deterministic `.npz` artifact
  so the inference demo can run immediately.
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

Do not implement the Module 9 lesson, runnable examples, model artifact,
glossary additions, or ADR as part of this planning-only change.
