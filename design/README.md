# Design notes

This catalog records the technical decisions that guide the course. Lesson
content belongs in `docs/`; runnable work belongs in `examples/`.

## Physics

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-23 | [Drone physics fundamentals lesson plan](physics/drone_physics_fundamentals_lesson_plan.md) | Course-wide physics sequence and capstone model. | Active reference |
| 2026-09-27 | [Reusable flight-forces model](physics/missing_flight_forces_plan.md) | Shared aerodynamic-force ownership, switches, and tuning order. | Partly implemented |

## Navigation

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-25 | [TTC bounding-box diagonal strike](navigation/ttc_bbox_diagonal_strike.md) | Module 7 visual time-to-contact proof of concept. | Implemented; tuning continues |
| 2026-09-29 | [Aggressive default TTC takeoff](navigation/aggressive-takeoff-plan.md) | Bounded climb-rate launch for faster tracking tests. | Implemented; 3.5 s target deferred |
| 2026-09-29 | [IMU-barometer vertical fusion](navigation/imu-barometer-vertical-fusion.md) | ICM-42688-P-inspired vertical estimator for the TTC strike. | Implemented |

## Integration

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-10-08 | [Betaflight SITL bridge](integration/betaflight_sitl_bridge.md) | UDP protocol, embedding boundary, and setup contract for Module 8. | Partly implemented |
| 2026-10-08 | [SITL home-relative odometry](integration/sitl_home_relative_odometry.md) | Takeoff-relative ENU odometry owned by the outer PyBullet PID. | Implemented |
| 2026-10-08 | [SITL yaw RC-sign convention](integration/sitl_yaw_rc_sign.md) | Explicit rotor-direction configuration and matching RC yaw feedback, checked by live disturbed hover. | Implemented |
| 2026-10-08 | [SITL machine reproduction](integration/sitl_machine_reproduction.md) | Installation, configuration restore, portable setup, and dated backups for verified hover. | Documented |
| 2026-10-08 | [FDM without velocity or position](integration/sitl_sensor_only_fdm.md) | Reserved-zero state slots and direct barometer pressure in a patched SITL receiver. | Implemented; live verified |
| 2026-10-08 | [Dual controller API](integration/betaflight_dual_controller_api.md) | Planned common backend API, distinguished from the verified standalone bridge. | Planned |

## Learning

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-10-08 | [ADRC position-command hierarchy](learning/adrc_position_command_hierarchy.md) | Estimator-ready, multi-rate LADRC cascade from home-relative XYZ/yaw commands to motor allocation. | Proposed |
| 2026-10-07 | [Module 04 motor mixer with PID and ADRC controllers](learning/module_04_motor_mixer_pid_plan.md) | Shared X-frame mixer and PyBullet loop with interchangeable cascaded PID and ADRC controllers. | Implemented |
| 2026-10-07 | [Autonomous hover capstone module plan](learning/autonomous_hover_capstone_module_plan.md) | Module 6 capstone plan for folder-owned cumulative examples that add each flight behavior to the shared PyBullet loop, including the Topic 0 mass budget and motor-lag subtopic. | Partly implemented |
| 2026-09-26 | [Learned vertical hover with a NumPy MLP](learning/mlp_vertical_hover_module_plan.md) | Module 9 policy-learning progression from generated data to PyBullet. | Partly implemented |

## Research

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-29 | [Propeller thrust-data sources](research/propeller-thrust-data-sources.md) | Trusted bench-data sources and the selected Module 3 teaching dataset. | Adopted |
| 2026-09-29 | [BMP388 barometer sensor model](research/bmp388-barometer-sensor-model.md) | Datasheet-grounded runtime altitude-noise model for the TTC example. | Implemented |
