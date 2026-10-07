# Topic 4: Roll and pitch torque

## By the end, you will be able to

- calculate torque from an off-center rotor force;
- predict roll or pitch direction from unequal thrust;
- use the URDF inertia to estimate angular acceleration;
- compare PyBullet and reduced-order attitude response;
- identify where the new torque method enters the cumulative loop;
- observe bounded roll/pitch oscillation from an alternating thrust imbalance.

An equal four-rotor thrust command produces lift. If one side produces more
thrust than the other, the force no longer acts through the center of mass. The
lever arm turns that force difference into body torque, changing angular rate
and then attitude.

```mermaid
flowchart LR
    position[Rotor position r] --> cross[Compute r cross F]
    imbalance[Unequal rotor thrust] --> cross
    cross --> torque[Body roll or pitch torque]
    torque --> inertia[Divide by Ixx or Iyy]
    inertia --> rate[Angular rate]
    rate --> attitude[Roll or pitch angle]
```

---

## The lever-arm equation

For rotor \(i\), torque about the center of mass is:

\[
\mathbf{\tau}_i = \mathbf{r}_i \times \mathbf{F}_i
\]

For an upward rotor force \(\mathbf{F}_i=(0,0,T_i)\), where
\(\mathbf{r}_i=(x_i,y_i,z_i)\):

\[
\tau_x = y_i T_i, \qquad \tau_y = -x_i T_i
\]

Summing all four rotors gives:

\[
\tau_x = \sum_i y_iT_i, \qquad \tau_y = -\sum_i x_iT_i
\]

| Symbol | Meaning | SI unit |
| --- | --- | --- |
| \(\mathbf{r}_i\) | Rotor position relative to the center of mass | m |
| \(\mathbf{F}_i\) | Rotor thrust force | N |
| \(T_i\) | Rotor thrust magnitude | N |
| \(\tau_x\) | Roll torque about body x | N·m |
| \(\tau_y\) | Pitch torque about body y | N·m |
| \(I_{xx}, I_{yy}\) | Roll and pitch moments of inertia | kg·m² |

The rotational equation is:

\[
\alpha_x=\frac{\tau_x}{I_{xx}}, \qquad \alpha_y=\frac{\tau_y}{I_{yy}}
\]

For the reference drone, \(y=0.12\,m\), \(I_{xx}=0.00348\,kg\,m^2\),
and a \(0.02\,N\) thrust difference on each side:

\[
\tau_x = 4(0.12)(0.02)=0.0096\,N\,m
\]

\[
\alpha_x = \frac{0.0096}{0.00348}\approx2.76\,rad/s^2
\]

The inertia controls the slope of the angular-rate curve: the same torque
creates less angular acceleration on a vehicle with larger inertia.

To keep this lesson near hover, the default collective command is `1290 us`,
which is close to the reference drone's calculated hover command. The default
roll imbalance is a conservative `0.0005 N` per motor, and the roll and pitch
imbalance is driven by:

[
Delta T(t)=Acosleft(\frac{2\pi t}{P}\right)
]

The sign changes every (P/2) seconds. This makes the drone receive torque in
both directions instead of accelerating continuously in one direction.

---

## Sign convention and thrust pattern

The vehicle uses body \(+x\) forward, body \(+y\) left, and body \(+z\) up.

- Positive roll delta increases thrust on rotors with positive \(y\).
- Positive pitch delta increases thrust on rotors with negative \(x\).
- The resulting torque is applied in the body frame.
- Rotor reaction torque is deferred to Topic 5.

Topic 3 collective thrust remains active. In this topic it is applied at the
center of mass so equal lift cannot create an accidental arm torque. Topic 4
then computes the deliberate torque from a virtual unequal-thrust pattern,
isolating the rotational response.

---

## Run the simulation

```bash
uv run python examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py --self-check
```

Run the reduced-order version:

```bash
uv run python examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py --backend reduced --self-check
```

Try pitch instead of roll:

```bash
uv run python examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py --backend reduced --roll-delta 0 --pitch-delta 0.0005 --seconds 1
```

When the PyBullet backend runs interactively, an external Tk control window
provides live run controls:

- `Start`: begin or resume the physics loop.
- `Pause`: stop physics ticks without changing the current state.
- `Restart`: reset the drone, battery, motors, timer, telemetry, and GIF
  frames, then press `Start` again.
- `Quit`: stop the topic and close the PyBullet session.

Interactive PyBullet ticks are paced in real time, so the alternating roll
direction is visible in the window. Headless and reduced-order runs remain
uncapped for fast experiments.

Interactive PyBullet runs are not limited by `--seconds`; they continue until
the Tk control window is closed. The `--seconds` value still limits headless,
reduced-order, self-check, and output-generation runs.

As the drone rolls, its body-up thrust vector tilts and creates horizontal
acceleration. Reversing the roll torque reverses the acceleration tendency, but
it does not erase the velocity already accumulated. Therefore the open-loop
drone is expected to trace a curved drifting trajectory, not automatically
return to its original position. Position control comes later.

Use a bounded headless run when generating a graph or testing a longer case:

```bash
uv run python examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py --seconds 10
```

The graph records torque, angular rate, and attitude:

![Topic 4 roll and pitch torque response](images/topic-04-roll-pitch-torque.png)

The recorded data is available at
[topic-04-roll-pitch-torque.csv](images/topic-04-roll-pitch-torque.csv).

Try a slower, larger oscillation:

```bash
uv run python examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py --seconds 4 --roll-delta 0.001 --oscillation-period 1.2
```

---

## The new force method

The topic-owned method calculates signed rotor thrust values, computes
\(\mathbf{r}\times\mathbf{F}\), and applies the resulting body torque:

??? example "Open the Topic 4 torque method"

    ```python
    # ! TOPIC 4 NEW FORCE: ROLL AND PITCH TORQUE
    def apply_roll_pitch_torque(drone, profile, collective_thrust_n, roll_delta_n, pitch_delta_n):
        rotor_thrusts_n = tuple(
            max(0.0, collective_thrust_n + roll_delta_n * sign(y) - pitch_delta_n * sign(x))
            for x, y, _ in profile.model.rotor_positions_m
        )
        torque_body_nm = torque_from_rotor_thrusts(profile, rotor_thrusts_n)
        p.applyExternalTorque(drone, -1, tuple(torque_body_nm), p.LINK_FRAME)
        return rotor_thrusts_n, torque_body_nm
    ```

Complete implementation: `examples/06-autonomous-hover/04-roll-pitch-torque/roll_pitch_torque.py`.

The cumulative PyBullet loop is:

```python
motor_rpms, battery_state = advance_motor_state(profile, battery, motor_rpms, args.pwm)
apply_gravity(drone, profile)
motor_thrusts = apply_rotor_thrust(drone, profile, motor_rpms)
# ! TOPIC 4 NEW FORCE CALL: alternating torque follows previous forces before integration.
_, torque_body_nm = apply_roll_pitch_torque(
        drone, profile, np.mean(motor_thrusts), args.roll_delta, args.pitch_delta
)
p.stepSimulation()
```

The reduced-order loop uses the same torque equation and integrates:

\[
\omega_{k+1}=\omega_k+\frac{\tau}{I}\Delta t, \qquad
\theta_{k+1}=\theta_k+\omega_{k+1}\Delta t
\]

---

## Exercise

1. Run the default roll case and identify where the roll rate changes sign.
2. Set `--roll-delta 0 --pitch-delta 0.0005` and compare pitch with roll.
3. Change the inertia in the drone profile and predict the rate slope change.
4. Set both deltas to zero and confirm the torque and angular-rate changes
   disappear while near-hover collective thrust remains.

---

## Checkpoint quiz

<form class="quiz" data-answer="b" data-explanation="For an upward force, tau_x = yT, so the y lever arm creates roll torque.">
<fieldset><legend>1. For an upward rotor force at positive body y, which term contributes to roll torque?</legend><label><input type="radio" name="topic4-q1" value="a"> xT</label><br><label><input type="radio" name="topic4-q1" value="b"> yT</label><br><label><input type="radio" name="topic4-q1" value="c"> zT</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="a" data-explanation="Angular acceleration is torque divided by moment of inertia, alpha = tau/I.">
<fieldset><legend>2. What happens to angular acceleration if inertia doubles while torque stays constant?</legend><label><input type="radio" name="topic4-q2" value="a"> It is reduced by half</label><br><label><input type="radio" name="topic4-q2" value="b"> It doubles</label><br><label><input type="radio" name="topic4-q2" value="c"> It becomes zero</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

<form class="quiz" data-answer="c" data-explanation="Rotor positions and thrust directions are body-frame quantities, so the resulting torque is applied in the body frame.">
<fieldset><legend>3. Which frame is used for the Topic 4 applied torque?</legend><label><input type="radio" name="topic4-q3" value="a"> Camera frame</label><br><label><input type="radio" name="topic4-q3" value="b"> World frame</label><br><label><input type="radio" name="topic4-q3" value="c"> Body frame</label></fieldset><button type="button" class="quiz-check">Check answer</button><p class="quiz-result" aria-live="polite"></p>
</form>

---

## Topic summary

Roll and pitch motion begins when thrust acts away from the center of mass.
The cross product \(\mathbf{r}\times\mathbf{F}\) converts the rotor lever arm
and thrust difference into body torque. The inertia then determines angular
acceleration.

The graph connects three results: alternating applied torque, angular-rate
response, and accumulated roll or pitch angle. With the near-hover baseline,
the expected result is a bounded rocking response: the torque changes sign,
the angular rate turns around, and the attitude moves in both directions.
The response is not a stabilized hover yet; damping and PID control come later.

The important baseline is that Topic 4 applies roll and pitch torque only.
Rotor reaction torque around body z is not part of this lesson; Topic 5 adds
that yaw effect. The cumulative model now contains gravity, contact, collective
thrust, motor lag, and attitude torque.

Previous: [Topic 3](../03-rotor-thrust/index.md). Next: [Topic 5](../05-reaction-yaw-torque/index.md).
