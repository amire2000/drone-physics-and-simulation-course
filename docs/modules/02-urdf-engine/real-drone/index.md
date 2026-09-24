# Real racing-quad URDF case study

## By the end, you will be able to

- Identify the physical choices in a racing-quad URDF.
- Locate rotor links that later receive propeller forces.
- Compare a teaching URDF with a simulator-oriented source model.

---

## From reference model to course model

This runnable `racing_quad.urdf` is a small adaptation of the public
[Racer URDF from gym-pybullet-drones](https://github.com/learnsyslab/gym-pybullet-drones/blob/main/gym_pybullet_drones/assets/racer.urdf).
It keeps the reference model's `0.83 kg` base mass, diagonal inertia, cylinder
collision shape, and four rotor locations. It replaces the external mesh and
the simulator-specific `&lt;properties&gt;` block with simple course geometry.

```mermaid
flowchart LR
    A[Reference Racer URDF] --> B[Keep physical layout]
    B --> C[Replace external mesh]
    C --> D[Remove Gym-only properties]
    D --> E[Runnable course racing_quad.urdf]
```

---

## The racing-quad structure

```xml
--8<-- "examples/02-urdf-engine/assets/racing_quad.urdf"
```

The `base_link` carries all `0.83 kg` of mass. Each `rotor_N` link has zero
mass and a fixed joint. That makes the link a reliable attachment point for a
motor force without inventing extra mass. The four joint origins form an X-like
layout at `±0.085 m` in X and `±0.0675 m` in Y.

The visual arms help a learner see the layout. The smaller collision cylinder
is intentional: collision geometry should be cheap and stable, not necessarily
as detailed as the visual model.

---

## Example: inspect a realistic layout

```bash
uv run python examples/02-urdf-engine/inspect_racing_quad.py
```

```python
--8<-- "examples/02-urdf-engine/inspect_racing_quad.py"
```

The terminal output lists the base dynamics followed by all four rotor-link
positions. Those positions are where Module 3 and Module 6 can apply thrust.

---

## Hands-on: change a real design parameter

1. Change the `rotor_0_joint` X origin from `0.085` to `0.12` m. Run the
   inspection script and confirm only that rotor position moved.
2. Change all four X and Y rotor offsets by the same scale factor. Explain why
   a longer arm gives the same thrust more roll or pitch leverage.
3. Change base mass from `0.83` to `1.00 kg`. Calculate the new hover force
   before running a future thrust experiment: \(T = mg\).

---

## Review quiz

<form class="quiz" data-answer="b" data-explanation="The base link owns the real mass and inertia; massless rotor links provide named force locations.">
  <fieldset><legend>1. Why are the rotor links massless?</legend><label><input type="radio" name="racer-q1" value="a"> To make them invisible</label><br><label><input type="radio" name="racer-q1" value="b"> To provide force locations without adding invented mass</label><br><label><input type="radio" name="racer-q1" value="c"> To remove gravity from the base</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="c" data-explanation="An off-center rotor force has a longer lever arm, so it creates more roll or pitch torque for the same thrust.">
  <fieldset><legend>2. What happens when a rotor moves farther from the center?</legend><label><input type="radio" name="racer-q2" value="a"> Its thrust becomes gravity</label><br><label><input type="radio" name="racer-q2" value="b"> The collision shape disappears</label><br><label><input type="radio" name="racer-q2" value="c"> Its thrust has more torque leverage</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="a" data-explanation="Simple collision geometry improves stable contact simulation; visuals can be detailed separately.">
  <fieldset><legend>3. Why use a cylinder for collision instead of the full visual shape?</legend><label><input type="radio" name="racer-q3" value="a"> It is a simpler stable contact shape</label><br><label><input type="radio" name="racer-q3" value="b"> It changes the drone's yaw convention</label><br><label><input type="radio" name="racer-q3" value="c"> It makes inertia unnecessary</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

---

Back to the [Module 2 overview](../index.md). Previous: [URDF anatomy](../urdf-anatomy/index.md). Next: [Module 3: Propeller aerodynamics](../../03-propeller-aerodynamics/index.md).
