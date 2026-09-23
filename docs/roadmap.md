# Roadmap: from gravity to autonomous hover

## Final goal

Create a real-time PyBullet quadcopter simulation that takes off and holds a
stable hover at 5 m while accounting for inertia, propeller forces, air drag,
control feedback, and battery voltage sag.

| Milestone | Modules | Outcome |
| --- | --- | --- |
| Simulation setup | 0 | Install PyBullet and run the first GUI sandbox. |
| Rigid-body foundation | 1 | Verify gravity, velocity, and hover force. |
| Physical vehicle | 2 | Load and validate a realistic quadcopter URDF. |
| Flight forces | 3 | Turn motor commands into thrust, yaw torque, and drag. |
| Stable control | 4 | Mix commands and stabilize the vehicle with PID. |
| Real-world limits | 5 | Model battery discharge and reduced available thrust. |
| Autonomous flight | 6 | Achieve a controlled takeoff and precision hover. |

## Recommended sequence

1. Start with [Module 0](modules/00-pybullet-setup/index.md); later modules
   use its environment and commands.
2. Keep a small runnable experiment for every force or controller change.
3. Compare simulation measurements with the expected equations before adding
   the next layer.
4. Integrate all layers only in the capstone.

See the complete [syllabus](syllabus.md).
