# Syllabus

## [0. PyBullet setup and GUI](modules/00-pybullet-setup/index.md)

- Install the project's PyBullet environment with `uv`.
- Run a plane-and-cube scene in the GUI or headless mode.
- Learn the essential GUI controls before adding drone physics.

## [1. Mass and external forces](modules/01-mass-and-forces/index.md)

- Initialize a PyBullet world with gravity `(0, 0, -9.81)` and a 240 Hz tick.
- Measure free fall and verify gravitational acceleration.
- Apply upward force and calculate the equilibrium needed to hover.

## [2. Physical asset blueprint (URDF)](modules/02-urdf-engine/index.md)

- Define a 650 g, 5-inch quadcopter frame in URDF.
- Derive `Ixx`, `Iyy`, and `Izz` from a central body and four motor masses.
- Spawn the asset and observe its response to off-axis torque.

## [3. Propeller dynamics and aerodynamics](modules/03-propeller-aerodynamics/index.md)

- Explain propeller airflow, SI units, diameter, pitch, and blade count.
- Map PWM through RPM to quadratic thrust using calibrated `kT` and `kQ`.
- Use ideal momentum theory to estimate induced velocity and rotor disk effects.
- Calculate four-rotor collective thrust, lever-arm torque, reaction torque, and power.

## [4. Motor mixer and PID control](modules/04-motor-mixer-pid/index.md)

- Build an X-frame motor mixer.
- Implement a rate PID controller for attitude stabilization.
- Convert high-level commands into safe 1000–2000 PWM motor signals.

## [5. Battery profiles and voltage sag](modules/05-battery-voltage-sag/index.md)

- Track LiPo state of charge from 4.2 V to the 3.7 V nominal floor per cell.
- Use internal resistance and Ohm's law to model voltage sag under load.
- Scale maximum thrust as the battery depletes.

## [6. Autonomous takeoff and precision hover](modules/06-autonomous-hover/index.md)

- Turn altitude error into continuous high-level decision inputs.
- Map model outputs through the mixer to motor PWM signals.
- Balance battery, drag, and inertia to hold a stable 5 m hover.

## [7. Forward camera and monocular optical navigation](modules/07-optical-navigation/index.md)

- Attach a forward-facing RGB camera to the drone body and render synchronized
  PyBullet frames.
- Track real optical flow between consecutive images to estimate image motion,
  yaw, and relative travel.
- Explain why monocular flow is measured in pixels per frame and cannot recover
  metric distance without an additional scale source.

## [8. Betaflight SITL control bridge](modules/08-betaflight-sitl/index.md)

- Build and pin a verified Betaflight SITL revision for reproducible lessons.
- Send simulated IMU/FDM state and virtual RC input over the current UDP bridge,
  then apply SITL's four motor outputs to the PyBullet drone.
- Fly a virtual-RC Angle-mode sequence: arm, hover, yaw, then disarm and land.

The bridge will follow the current [Betaflight SITL harness](https://github.com/betaflight/betaflight/blob/master/src/test/sitl/sitl_harness.py) packet roles while keeping the tested revision pinned in the module setup instructions.

## [9. Learned vertical hover with an MLP](modules/09-mlp-hover/index.md)

- Train a small NumPy MLP to imitate the altitude PID for a fixed 3 m hover.
- Keep the existing attitude PID, motor mixer, and PyBullet physics responsible
  for low-level stabilization; the MLP commands only vertical acceleration.
- Recognize that success from one clean initial condition can be memorization,
  not evidence that the policy generalizes.

The implementation plan is recorded in
`design/mlp_vertical_hover_module_plan.md`.
