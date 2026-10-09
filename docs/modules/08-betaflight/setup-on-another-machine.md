# Set up the verified Betaflight SITL bridge on another machine

**Updated:** 2026-10-08. This guide reproduces the last successful origin-hover
test with Betaflight `2026.6.2` on Linux. Ubuntu 24.04 is the tested host;
native Windows and macOS builds have not been validated here.

```mermaid
flowchart LR
    files[Copy course and lockfile] --> deps[Install dependencies]
    deps --> build[Build pinned SITL]
    build --> test[Run isolated hover check]
    test --> provision[Provision course EEPROM]
    provision --> fly[Start SITL and PyBullet]
```

---

## 1. Transfer the complete course

Copy the course repository including the latest bridge Python files, shared
URDF, `tools/`, `pyproject.toml`, `uv.lock`, and the dated backup under
`examples/08-betaflight-sitl/config/backups/2026-10-08/`.

The fixes are currently workspace changes. A clone of the existing repository
commit alone does not contain them; transfer the updated files too, or use a
repository revision that includes them. Rebuild `.sitl/` and `.venv/` on the
destination rather than copying those machine-specific directories.

All commands below run from the **course repository root**. SITL and PyBullet
run on the same destination machine and communicate over `127.0.0.1`.

---

## 2. Install dependencies

On Ubuntu 24.04:

```bash
sudo apt-get update
sudo apt-get install -y build-essential git python3 python3-venv
```

Install `uv` if it is not already available. One option is an isolated Python
environment for the tool:

```bash
python3 -m venv /tmp/course-uv-bootstrap
/tmp/course-uv-bootstrap/bin/python -m pip install uv
export PATH="/tmp/course-uv-bootstrap/bin:$PATH"
uv sync --locked
```

The bootstrap environment is temporary; use your usual persistent `uv`
installation for later sessions. The project requires Python 3.11 or newer;
Ubuntu 24.04's Python 3.12 satisfies that requirement. Keep `uv.lock` from the
tested course copy. GUI flights also require a working desktop display;
the acceptance test uses headless PyBullet.

---

## 3. Build the exact firmware

```bash
tools/setup_betaflight_sitl.sh
git -C .sitl/betaflight-source rev-parse HEAD
git -C .sitl/betaflight-source status --short
```

Expected revision:

```text
e0b7bb01b17b21351057e9ead2d1ab39dd44fa16
```

The source status now shows one expected modification, `src/platform/SIMULATOR/sitl.c`,
from the checked-in course patch. The script uses
`make TARGET=SITL EXTRA_FLAGS="-DSITL_EXTERNAL_BAROMETER"` and
creates `.sitl/betaflight-source/obj/main/betaflight_SITL.elf`. Use this bare
target and the explicit course settings; its defaults differ from the named
Gazebo target, notably for yaw motor direction. The patch makes SITL consume
transmitted barometer pressure and skip virtual GPS updates. The encoder sends
zero velocity/position placeholders. The setup script applies this patch
idempotently and refuses conflicting source modifications; keep the patch
file when copying the course to another machine.

---

## 4. Restore the verified parameter overrides

The dated backup is a text replay of explicit settings over the pinned
firmware defaults. It is not a full firmware parameter dump or an EEPROM image.

```bash
cp examples/08-betaflight-sitl/config/backups/2026-10-08/course_angle_mode.config examples/08-betaflight-sitl/config/course_angle_mode.config
sha256sum examples/08-betaflight-sitl/config/course_angle_mode.config
```

Expected configuration SHA-256:

```text
596583213075b39838218a2c51b194e746e92aaca344f10b5deedbc54d250fff
```

The critical tested settings are:

| Setting | Verified value |
| --- | --- |
| Mixer | `QUADX` |
| RC channel mapping | `AETR1234` |
| Yaw motor direction | `yaw_motors_reversed = ON` |
| Inner yaw P / I / feedforward | `10 / 0 / 0` |
| Arm mode | AUX1, 1700–2100 µs |
| Angle mode | AUX1, 1700–2100 µs, tied to Arm |
| RC-loss procedure | `DROP` |
| Outer yaw P / I / D in Python | `0.15 / 0 / 0.02` |
| Physics / outer position / outer velocity / RC rates | `240 / 30 / 60 / 50 Hz` |
| Motor lag / motor-packet timeout | `0.05 / 0.1 s` |

The snapshot preserves the old AUX2 comment byte for byte, but the actual
`aux` commands select AUX1 for both Arm and Angle. This table describes the
actual tested wiring. Updating the comment does not require changing the
commands; rewiring Angle to AUX2 changes the tested configuration.

Python controller and physics values are recorded in the dated
`controller-and-plant.json` backup. That file is a record, not a runtime
configuration loader; use the matching course Python files and shared URDF.

---

## 5. Verify before starting an interactive flight

Stop the normal SITL and bridge first. The isolated test needs UDP
9002/9003/9004 and TCP 5761 free, and starts and closes its own SITL process.

```bash
uv run python examples/08-betaflight-sitl/protocol_self_check.py
uv run python examples/08-betaflight-sitl/position_bridge_demo.py --self-check
uv run python examples/08-betaflight-sitl/hover_self_check.py
```

The live check runs 30 simulated seconds, requests `(0, 0, 3)` relative to the
initial base pose, and injects a `+0.5 rad/s` yaw disturbance at 15 seconds.
Expect the final message:

```text
Live SITL disturbed-hover self-check passed
```

The recorded pre-patch successful run reported maximum altitude error `0.016 m`, XY
error `0.026 m`, maximum tilt `0.002 rad`, and final yaw `0.012 rad`. Exact
values can vary with host timing. The acceptance bounds are `0.3 m` altitude,
`0.3 m` horizontal position, `0.2 rad` tilt, and `0.1 rad` final yaw.
Measurements begin at 12 seconds, after the takeoff transient. This validates
the hover baseline; wind, large manoeuvres, and autonomous landing remain
outside this test.
The revised check also reads SITL's MSP altitude at 14 seconds, accepting
1–5 m at the 3 m hover target. This confirms a receiver-side altitude response
while the FDM position and velocity slots are zero; it is separate from the
outer controller's PyBullet pose validation.

---

## 6. Start normal SITL and the simulation

In terminal 1:

```bash
tools/run_betaflight_sitl.sh
```

This recreates `.sitl/course-angle-mode/eeprom.bin` from the active text
configuration and starts SITL. Any custom parameters previously stored only
in that EEPROM are replaced. Keep your own custom CLI backup before using
the course provisioner on a customized run directory.

Wait for the initial startup/boot grace to finish before arming. In terminal 2,
choose a headless scheduled flight:

```bash
uv run python examples/08-betaflight-sitl/position_bridge_demo.py --headless
```

Or run an interactive takeoff and hold:

```bash
uv run python examples/08-betaflight-sitl/slider_bridge_demo.py --auto-takeoff
```

The scheduled demo climbs to `(0, 0, 3)`, holds the takeoff X/Y point, then
disarms; disarming is not an autonomous landing manoeuvre. The GUI exposes
home-relative targets and stays active until you stop it. Set Arm to zero
before closing it. The outer controller first sends arm-low/throttle-zero,
then arm-high/throttle-zero, before commanding takeoff thrust.

---

## Understand feedback and common failures

PyBullet base position minus the initial base position is the outer PID's
odometry. All position targets use that same metre-based frame. SITL controls
attitude, rates, and motors from RC and IMU data. All packet velocity/position
fields are reserved zeros, and the patched receiver uses transmitted pressure
for barometer data. The SITL `pos=(0,0)` diagnostic is not your XY odometry.
Quaternion data still supplies the synthetic magnetometer; this change does
not remove the quaternion or create a raw-IMU-only attitude estimator.

| Symptom | Check |
| --- | --- |
| `THROTTLE ARM_SWITCH` | Send arm OFF and zero throttle, then retry the low-throttle arm sequence. |
| Spin followed by climbing | Reprovision the corrected text config; check `yaw_motors_reversed=ON`, inner yaw `10/0/0`, and no extra Python RC yaw negation. |
| Configuration edits have no effect | Restart through `tools/run_betaflight_sitl.sh`; merely restarting an old binary with an old EEPROM keeps old values. |
| Port already in use | Stop the existing simulator/bridge before provisioning or running the isolated test. |
| Hover check never takes off | Check firmware revision, boot/arming state, and the final SITL diagnostics printed by the check. |

See [bridge overview](index.md) for packet and motor mapping details. The
dated backup and the integration design catalog live in the transferred
repository under `examples/08-betaflight-sitl/config/backups/2026-10-08/` and
`design/integration/`.
