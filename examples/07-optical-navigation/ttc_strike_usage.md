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
  --config examples/07-optical-navigation/ttc_strike/scenario.yaml
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
  sensor_filters:
  pid:
  vertical_control:
```

`simulation` describes the reproducible test bench (scene, camera, GUI, and recording). `runtime` contains parameters to calibrate on the physical vehicle (camera mounting, mission targets, TTC policy, filters, and controllers). Any omitted value uses the default in the dataclasses in `ttc_strike/config.py`.

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
- `telemetry.png` — velocity, trajectory, trajectory-command, and guidance plots.
- `environment.mp4` — optional camera recording when video is enabled.

The pitch columns in `telemetry.csv` are especially useful when tuning: `command_pitch_deg`, `measured_pitch_deg`, `pitch_error_deg`, and `pitch_torque`. Compare the command with the measured attitude to distinguish a slow attitude response from a bad trajectory command.

## Troubleshooting

- **Target is lost before commit:** lower `runtime.ttc.commit_box_height_fraction`. The 30 m scenario uses `0.04`; the 50 m scenario uses `0.02`.
- **The target is not visible:** check `runtime.physical_setup.camera_look_down_deg`, camera FOV, target position, and the far-plane distance.
- **Headless run does not contact the target:** inspect `summary.json` (`final_phase`, `abort_reason`, and impact fields) and then examine `telemetry.csv` for the last valid bounding box and command.

## Archived tuning report

The archived pitch-tuning comparison is [`reports/ttc_pitch_tuning/2026-09-25/report.md`](../../reports/ttc_pitch_tuning/2026-09-25/report.md). It places the baseline run on the left and the tuned run on the right, with the PID settings and plots used for the comparison.
