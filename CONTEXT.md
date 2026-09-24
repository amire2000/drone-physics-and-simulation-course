# Drone Physics and Simulation

This learning project models quadcopter flight from rigid-body physics through
learned guidance.

## Learned Flight Control

**High-level policy**:
An MLP that maps a measured drone state and a flight objective to stabilized
flight commands. It does not command individual motors.
_Avoid_: Motor controller, direct-PWM policy

**Low-level controller**:
The PID and motor mixer that convert high-level flight commands into four motor
commands while stabilizing the drone.
_Avoid_: Policy, neural controller
