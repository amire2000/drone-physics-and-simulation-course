# TTC strike usage guide

This Module 7 proof of concept takes off, detects a red cube, estimates time to contact (TTC) from bounding-box growth, synchronizes descent with the diagonal approach, and records the result. The implementation and class design are described in [`ttc_strike/README.md`](ttc_strike/README.md).

## Setup and self-check

```bash
uv sync
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py --self-check
```

## Run the default scenario

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py
```

For a repeatable run without the PyBullet GUI:

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py --headless
```

Press `q` or `Esc` in the simulation window to abort safely. The process also stops when the configured mission timeout, collision, or abort condition is reached.

## Choose an input YAML file

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py \
  --config examples/07-optical-navigation/ttc_strike/config/scenario.yaml
```

Ready-to-run configurations are in [`ttc_strike_inputs/`](ttc_strike_inputs/):

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py \
  --config examples/07-optical-navigation/ttc_strike_inputs/30m_diagonal_strike.yaml

uv run python examples/07-optical-navigation/ttc_diagonal_strike.py \
  --config examples/07-optical-navigation/ttc_strike_inputs/50m_diagonal_strike.yaml
```

Copy [`template.yaml`](ttc_strike_inputs/template.yaml) when creating a new experiment. Keep the YAML grouped as follows:

```yaml
simulation:
  scene:
  vehicle_model:
  sensor_model:
  display:
  recording:
runtime:
  physical_setup:
  mission:
  flight_limits:
  ttc:
  sensors:
  pid:
  vertical_control:
```

`simulation` describes the reproducible test bench (scene, camera, GUI, and recording). `runtime` contains parameters to calibrate on the physical vehicle (camera mounting, mission targets, TTC policy, filters, and controllers). Any omitted value uses the default in the dataclasses in `ttc_strike/config.py`.

`runtime.sensors.barometer` models a BMP388 altitude sensor independently of
the camera. `altitude_noise_sigma_m: 0.10` is the BMP388 full-bandwidth
datasheet noise converted from 1.2 Pa to altitude. Use `altitude_bias_m` for
the takeoff-reference error and enable `drift_sigma_m_per_sqrt_s` only when
you want a seeded slow field-drift experiment. The default drift is zero.
The default `velocity_old_weight: 0.95` smooths differentiated 40 Hz altitude
noise before the vertical controller uses it.

`altitude_old_weight: 0.80` smooths raw barometer altitude before the guidance
controller uses it. Every new run logs raw altitude, filtered altitude, and
filtered vertical speed. To graph an existing run without rerunning PyBullet:

```bash
uv run python examples/07-optical-navigation/plot_barometer_csv.py \
  /tmp/ttc-barometer-check/bmp388-40hz-graph
```

`simulation.vehicle_model.profile` selects the physical drone. `default` keeps
the course quadcopter; `seven_inch_trainer` selects the generic 1.5 kg,
seven-inch vehicle. The linked URDF supplies mass and inertia, while the
profile supplies motor, propeller, drag, and damping data. Use the dedicated
seven-inch scenario when starting a comparison:

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py \
  --config examples/07-optical-navigation/ttc_strike/config/seven_inch_trainer.yaml
```

## Command-line options

| Option | Purpose |
| --- | --- |
| `--config PATH` | Load a grouped YAML scenario. |
| `--headless` | Run without the PyBullet GUI. |
| `--self-check` | Validate imports and basic configuration, then exit. |
| `--max-seconds N` | Override the mission timeout. |
| `--run-name NAME` | Name the output run directory. |
| `--output-root PATH` | Choose the parent directory for run artifacts. |
| `--video PATH` / `--no-video` | Enable or disable camera video recording. |
| `--plot PATH` / `--no-plot` | Enable or disable the telemetry plot. |
| `--csv PATH` / `--no-csv` | Enable or disable telemetry CSV output. |

For example, this creates a compact tuning run without video:

```bash
uv run python examples/07-optical-navigation/ttc_diagonal_strike.py \
  --headless \
  --config examples/07-optical-navigation/ttc_strike_inputs/30m_diagonal_strike.yaml \
  --run-name 30m-pitch-test \
  --no-video
```

## Output and telemetry

Each run is stored below `outputs/ttc_runs/<run-name>/`:

- `settings.json` — the resolved simulation and runtime settings.
- `summary.json` — final phase, success/abort status, abort reason, impact speed, and scene details.
- `telemetry.csv` — time-series data for analysis and comparison.
- `telemetry.png` — velocity, trajectory, trajectory-command, guidance, and raw-versus-filtered bbox-growth plots.
- `environment.mp4` — optional camera recording when video is enabled.

The pitch columns in `telemetry.csv` are especially useful when tuning: `command_pitch_deg`, `measured_pitch_deg`, `pitch_error_deg`, and `pitch_torque`. Compare the command with the measured attitude to distinguish a slow attitude response from a bad trajectory command.

## Troubleshooting

- **Target is lost before commit:** lower `runtime.ttc.commit_box_height_fraction`. The 30 m scenario uses `0.04`; the 50 m scenario uses `0.02`.
- **The target is not visible:** check `runtime.physical_setup.camera_look_down_deg`, camera FOV, target position, and the far-plane distance.
- **Headless run does not contact the target:** inspect `summary.json` (`final_phase`, `abort_reason`, and impact fields) and then examine `telemetry.csv` for the last valid bounding box and command.

## Archived tuning report

The archived pitch-tuning comparison is [`reports/ttc_pitch_tuning/2026-09-25/report.md`](../../reports/ttc_pitch_tuning/2026-09-25/report.md). It places the baseline run on the left and the tuned run on the right, with the PID settings and plots used for the comparison.
