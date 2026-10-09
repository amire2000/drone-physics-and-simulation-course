# Sensor FDM revision — 2026-10-08

**Status:** Live hover and receiver pressure check passed.

The parameter commands in this directory remain unchanged. The current build
uses the same pinned revision `e0b7bb01b17b21351057e9ead2d1ab39dd44fa16` plus
`tools/patches/betaflight-sitl-external-barometer.patch`, built with
`make TARGET=SITL EXTRA_FLAGS="-DSITL_EXTERNAL_BAROMETER"`.

- Patch SHA-256: `8615ea9e187b1a973ec1422443fc833de82b0deb6279638b3b7af011f5783fa9`
- Native binary SHA-256 on this host: `ebd42100236aca444efafce736e6cae0ed7318c7057bbd4fc3704e4c34927b12`

FDM retains its 144-byte layout but uses six constant zeros for velocity and
position. The receiver reads transmitted pressure directly and skips virtual
GPS updates. Quaternion and synthetic magnetometer behavior remain unchanged.
The earlier `controller-and-plant.json` records the pre-patch binary checksum;
it is historical evidence, not the checksum of this build.

The first patched 30-second disturbed-hover test reported:

```text
hover: max altitude error=0.018 m, XY error=0.011 m, tilt=0.001 rad, final yaw=0.012 rad
Live SITL disturbed-hover self-check passed
```

Reproduce with `uv run python examples/08-betaflight-sitl/hover_self_check.py`
from the course repository root, with other SITL/bridge processes stopped.
The current check also reads MSP altitude at 14 seconds to prove barometer
reception while the wire position and velocity remain zero.

Final test with that receiver check enabled:

```text
hover: max altitude error=0.016 m, XY error=0.017 m, tilt=0.002 rad, final yaw=0.011 rad
SITL pressure-derived estimated altitude: 3.080 m
Live SITL disturbed-hover self-check passed
```
