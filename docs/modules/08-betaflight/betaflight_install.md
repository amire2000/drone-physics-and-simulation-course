# Betaflight SITL installation and configuration restore

**Updated:** 2026-10-08  
**Tested platform:** Ubuntu 24.04, native Linux SITL, headless PyBullet.

This document installs the course's verified Betaflight controller and restores
the settings that prevent the yaw spin and uncontrolled climb. Run every
command from the **course repository root**. SITL and PyBullet run on the same
machine using `127.0.0.1`.

```mermaid
flowchart LR
    copy[Copy updated course] --> deps[Install dependencies]
    deps --> build[Build pinned firmware and course patch]
    build --> restore[Restore text parameters]
    restore --> verify[Run isolated flight check]
    verify --> start[Start normal SITL and PyBullet]
```

---

## 1. Copy the required files

Transfer the updated course repository, including these files and folders:

- `tools/`, including `tools/patches/betaflight-sitl-external-barometer.patch`.
- `examples/08-betaflight-sitl/`, including the configuration and dated backup.
- `examples/04-mix-control/` and `examples/common/assets/full_drone.urdf`.
- `pyproject.toml` and `uv.lock`.

The fixes currently include uncommitted workspace files. Copy those files too;
cloning only the existing Git commit does not reproduce the current setup.
Create `.venv/` and rebuild `.sitl/` on the destination machine.

---

## 2. Install system and Python dependencies

On Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install -y build-essential git python3 python3-venv
```

If `uv` is already installed, synchronize the project dependencies:

```bash
uv sync --locked
```

Otherwise, install it into a dedicated tool environment inside the course:

```bash
python3 -m venv .venv-uv
.venv-uv/bin/python -m pip install uv
export PATH="$PWD/.venv-uv/bin:$PATH"
uv sync --locked
```

Repeat the `export PATH` command in later terminals if using that dedicated
installation. The project requires Python 3.11 or newer. A desktop display is
needed for the GUI example; the self-check runs without a GUI.

---

## 3. Build the course firmware

```bash
tools/setup_betaflight_sitl.sh
git -C .sitl/betaflight-source rev-parse HEAD
```

The firmware is Betaflight `2026.6.2`. The revision must be:

```text
e0b7bb01b17b21351057e9ead2d1ab39dd44fa16
```

The setup script applies the checked-in course patch and builds with:

```text
make TARGET=SITL EXTRA_FLAGS="-DSITL_EXTERNAL_BAROMETER"
```

Executable:

```text
.sitl/betaflight-source/obj/main/betaflight_SITL.elf
```

The patch is required: the bridge sends zero placeholders in all FDM velocity
and position fields. Patched SITL reads the transmitted pressure directly and
skips virtual GPS updates. An unpatched Gazebo build derives barometer pressure
from the zero altitude field instead. Gyroscope, accelerometer, quaternion,
and pressure are still transmitted; quaternion-derived magnetometer behavior
is retained.

`git -C .sitl/betaflight-source status --short` should show the expected
modification to `src/platform/SIMULATOR/sitl.c`. Re-running setup recognizes
an already-applied patch. Conflicting firmware changes require inspection.

---

## 4. Restore the verified configuration

Stop existing SITL and bridge processes before provisioning. The default run
directory is `.sitl/course-angle-mode/`. Provisioning replaces its EEPROM,
so preserve any custom settings before restoring the course configuration.

Copy the dated parameter backup into the active configuration file:

```bash
cp examples/08-betaflight-sitl/config/backups/2026-10-08/course_angle_mode.config examples/08-betaflight-sitl/config/course_angle_mode.config
sha256sum examples/08-betaflight-sitl/config/course_angle_mode.config
```

The exact backup checksum is:

```text
596583213075b39838218a2c51b194e746e92aaca344f10b5deedbc54d250fff
```

The restored commands include:

```text
mixer QUADX
map AETR1234
set yaw_motors_reversed = ON
set p_yaw = 10
set i_yaw = 0
set f_yaw = 0
aux 0 0 0 1700 2100
aux 1 1 0 1700 2100
set failsafe_procedure = DROP
save
```

Both `aux` commands use zero-based AUX index `0`: **AUX1 selects both Arm
and Angle mode** in the verified configuration. The historical backup comment
mentions AUX2 incorrectly; follow the actual commands above.

Restore a fresh EEPROM without leaving SITL running:

```bash
tools/provision_betaflight_sitl.sh
```

This invokes the pinned executable's `--config` mode and saves:

```text
.sitl/course-angle-mode/eeprom.bin
.sitl/course-angle-mode/provision.log
```

The backup contains explicit overrides over the pinned firmware defaults,
not a full `dump all` or a binary EEPROM image. See the backup files at
`examples/08-betaflight-sitl/config/backups/2026-10-08/`:

- `README.md`: restore scope and historical flight evidence.
- `controller-and-plant.json`: outer PID gains, rotor model, and timing.
- `sensor-fdm-verification.md`: patched firmware identity and latest test results.

Use the matching course Python files too. The outer yaw gains are
`(0.15, 0, 0.02)` and the RC yaw correction has no additional sign negation.
The JSON snapshot is a record; the application does not load gains from it.

---

## 5. Verify installation and restored parameters

Keep normal SITL and the bridge stopped. The flight check starts its own
temporary SITL and EEPROM and requires UDP 9002/9003/9004 and TCP 5761 free.
Its EEPROM is separate from the normal run directory.

```bash
uv run python examples/08-betaflight-sitl/protocol_self_check.py
uv run python examples/08-betaflight-sitl/position_bridge_demo.py --self-check
uv run python examples/08-betaflight-sitl/hover_self_check.py
```

The last command runs a 30-second hover at `(0, 0, 3)` relative to the initial
base position and injects a yaw disturbance at 15 seconds. It also checks
SITL's altitude response from the transmitted pressure. The last successful
patched run reported:

```text
hover: max altitude error=0.016 m, XY error=0.017 m, tilt=0.002 rad, final yaw=0.011 rad
SITL pressure-derived estimated altitude: 3.080 m
Live SITL disturbed-hover self-check passed
```

Exact values depend on host timing. The pass limits are 0.3 m altitude error,
0.3 m XY error, 0.2 rad tilt, 0.1 rad final yaw, and 1–5 m SITL estimated
altitude at the 3 m hover target. This verifies origin hover, not wind response
or autonomous landing.

---

## 6. Start a normal flight

In terminal 1:

```bash
tools/run_betaflight_sitl.sh
```

This script reprovisions the normal EEPROM from the active configuration on
every start, then runs SITL. Wait for startup/boot grace to finish before arming.

In terminal 2, run an interactive takeoff to 3 m and hold:

```bash
uv run python examples/08-betaflight-sitl/slider_bridge_demo.py --auto-takeoff
```

Or choose the scheduled headless takeoff/hover/disarm example:

```bash
uv run python examples/08-betaflight-sitl/position_bridge_demo.py --headless
```

X/Y `(0, 0)` denotes the initial takeoff point. The outer position and velocity
PID reads PyBullet odometry locally; SITL receives virtual RC commands and
sensor data. Its `pos=(0,0)` diagnostic does not show that local odometry.

Set the GUI Arm slider to zero before closing the simulation. Stop SITL with
Ctrl+C in its terminal. The scheduled example ends by disarming rather than
performing a controlled landing.

---

## Troubleshooting

| Symptom | Action |
| --- | --- |
| Port in use during restore/check | Stop the existing SITL or bridge, then retry. |
| `THROTTLE ARM_SWITCH` | Arm OFF and throttle zero first; retry the low-throttle arm sequence. |
| Spin or uncontrolled climb | Restore yaw direction and gains, then restart through the course run script. |
| SITL altitude stays zero | Rebuild with the course barometer patch and restart the new executable. |
| Edits do not change behavior | Reprovision EEPROM; an old EEPROM retains old settings. |
| Firmware patch conflict | Inspect local changes in the firmware checkout; do not discard them blindly. |

For architecture details, see the [bridge overview](index.md). The
[machine-transfer guide](setup-on-another-machine.md) contains additional
portability and backup context.
