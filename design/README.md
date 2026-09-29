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

## Integration

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-27 | [Betaflight SITL bridge](integration/betaflight_sitl_bridge.md) | UDP protocol and setup contract for Module 8. | Planned |
| 2026-09-27 | [Dual controller API](integration/betaflight_dual_controller_api.md) | Boundary for selecting the local or Betaflight low-level controller. | Planned |

## Learning

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-26 | [Learned vertical hover with a NumPy MLP](learning/mlp_vertical_hover_module_plan.md) | Module 9 policy-learning progression from generated data to PyBullet. | Partly implemented |

## Research

| Updated | Document | Description | Status |
| --- | --- | --- | --- |
| 2026-09-29 | [Propeller thrust-data sources](research/propeller-thrust-data-sources.md) | Trusted bench-data sources and the selected Module 3 teaching dataset. | Adopted |
| 2026-09-29 | [BMP388 barometer sensor model](research/bmp388-barometer-sensor-model.md) | Datasheet-grounded runtime altitude-noise model for the TTC example. | Implemented |
