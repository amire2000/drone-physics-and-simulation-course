# Verified SITL parameter backup — 2026-10-08

This snapshot matches the last successful 30-second disturbed-hover run on
Ubuntu 24.04. The test built on Betaflight `2026.6.2`, source revision
`e0b7bb01b17b21351057e9ead2d1ab39dd44fa16`, with a clean firmware source tree.

This is the historical baseline **before removal of wire velocity/position**.
The current setup additionally applies the course external-barometer firmware
patch and sends six reserved zeros. The parameter commands remain the same;
the firmware patch is a separate build requirement. See
[the FDM revision](../../../../../design/integration/sitl_sensor_only_fdm.md).
The [sensor FDM verification record](sensor-fdm-verification.md) identifies
the patched build and its flight evidence.

## Contents

- [course_angle_mode.config](course_angle_mode.config): byte-for-byte snapshot
  of the CLI configuration used to provision the successful test's temporary EEPROM.
- [controller-and-plant.json](controller-and-plant.json): Python outer PID,
  vehicle, timing, source revision, checksums, and recorded flight results.

The CLI file is a **parameter override backup**, not a full `dump all` export.
All other parameters come from the pinned firmware's defaults. Replaying these
overrides onto a fresh EEPROM with that firmware recreates the tested setup.
The test's temporary EEPROM was removed on test exit; no binary EEPROM from
that run is claimed or included. The ordinary `.sitl/course-angle-mode/`
EEPROM may predate this correction and must be reprovisioned.

The exact tested mode lines are:

```text
aux 0 0 0 1700 2100
aux 1 1 0 1700 2100
```

The third value is zero-based AUX channel index. Both lines use **AUX1**:
Arm and Angle become active together. The old comment saying AUX2 selected
Angle was incorrect. The bridge also transmits an Angle flag on AUX2, but this
configuration listens to AUX1. Preserve this wiring when reproducing the test.

## Verification evidence

```text
hover: max altitude error=0.016 m, XY error=0.026 m, tilt=0.002 rad, final yaw=0.012 rad
Live SITL disturbed-hover self-check passed
```

The test injected `+0.5 rad/s` yaw at 15 seconds, kept target `(0, 0, 3)`, and
measured hover from 12 seconds onward. These rounded results are from the
successful run recorded in this session, not a promised result on every host.
The native binary checksum is for provenance; compiler and platform changes
may produce a different checksum. The clean pinned source and live acceptance
test are the portability checks.

## Restore

Follow [setup on another machine](../../../../../docs/betaflight/setup-on-another-machine.md).
Run commands from the course repository root. Keep the normal SITL and bridge
stopped when running the isolated self-check; it owns UDP 9002/9003/9004 and
TCP 5761 for the duration of the test.
