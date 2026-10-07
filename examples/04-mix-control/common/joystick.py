"""Keyboard-backed virtual joystick for the Module 04 attitude-hold entry point."""

from dataclasses import dataclass

import pybullet as p

ESCAPE_KEY = 27

@dataclass(frozen=True)
class JoystickLimits:
    """Maximum desired roll, pitch, and yaw angles in radians."""

    roll_rad: float
    pitch_rad: float
    yaw_rad: float


def command_from_keys(events: dict[int, int], limits: JoystickLimits) -> tuple[float, float, float] | None:
    """Convert held keyboard keys into a bounded attitude setpoint."""
    if events.get(ESCAPE_KEY, 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
        return None

    def held(key: str) -> bool:
        return bool(events.get(ord(key), 0) & p.KEY_IS_DOWN)

    roll = (held("d") - held("a")) * limits.roll_rad
    pitch = (held("w") - held("s")) * limits.pitch_rad
    yaw = (held("e") - held("q")) * limits.yaw_rad
    if events.get(ord(" "), 0) & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
        return (0.0, 0.0, 0.0)
    return (float(roll), float(pitch), float(yaw))


class KeyboardJoystick:
    """Read a simple four-key attitude joystick from the PyBullet GUI."""

    def __init__(self, limits: JoystickLimits) -> None:
        """Create a keyboard adapter with fixed angle limits."""
        self.limits = limits

    def read(self) -> tuple[float, float, float] | None:
        """Return the current attitude setpoint, or ``None`` when exiting."""
        return command_from_keys(p.getKeyboardEvents(), self.limits)
