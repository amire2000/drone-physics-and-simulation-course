# Reusable simulation-report workflow

Use this workflow for future archived comparisons of TTC strike or other
simulation runs.

## 1. Select and label runs

Choose two completed runs with the same scenario and one intentional change.
Keep the older run on the left and the newer run on the right. Rename the raw
run folders under `outputs/ttc_runs/` to descriptive labels while recording
the original timestamped folder names in the report.

Recommended labels are `baseline-<condition>` and `tuned-<change>`.

## 2. Create the archive

Create a dated folder under `reports/`:

```text
reports/<experiment>/<YYYY-MM-DD>/
├── report.md
├── baseline-<condition>/
│   ├── telemetry.csv
│   ├── telemetry.png
│   ├── settings.json
│   └── summary.json
└── tuned-<change>/
    ├── telemetry.csv
    ├── telemetry.png
    ├── settings.json
    └── summary.json
```

Copy the CSV, plot, settings, and summary into the archive. Do not copy large
videos unless the experiment specifically needs them. Update copied summary
paths if the raw run folders were renamed.

## 3. Write the report

Every report should contain:

- objective, date, source run IDs, and scenario configuration;
- a complete PID table with `kp`, `ki`, `kd`, and every integral limit;
- a table of outcome metrics extracted from `summary.json` and `telemetry.csv`;
- a short explanation of what changed and why;
- the baseline graph on the left and the newer graph on the right;
- links to the archived CSV, settings, summary, and plot files;
- limitations, unexpected behavior, and the next tuning action.

Use an HTML table for side-by-side images so the ordering is unambiguous:

```html
<table>
<tr><th>Baseline</th><th>New run</th></tr>
<tr>
<td><img src="baseline/telemetry.png" alt="Baseline telemetry"></td>
<td><img src="tuned/telemetry.png" alt="New telemetry"></td>
</tr>
</table>
```

## 4. Validate

Before committing:

1. Recalculate headline metrics from the archived CSV/JSON, not memory.
2. Confirm the newer run is on the right in every comparison.
3. Check every relative link and image path.
4. Run `git diff --check`.
5. Run `uv run mkdocs build --strict` when the report is linked from docs.
6. Commit the report and archive together.
