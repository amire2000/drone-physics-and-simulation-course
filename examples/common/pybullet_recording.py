"""Small deterministic camera and animated-GIF helpers for PyBullet lessons."""

from pathlib import Path

import numpy as np
import pybullet as p


def render_fixed_camera_frame(width_px: int = 640, height_px: int = 480) -> np.ndarray:
    """Render the connected PyBullet world from the course's fixed teaching camera."""
    view = p.computeViewMatrixFromYawPitchRoll(
        (0.0, 0.0, 1.0),
        3.0,
        45.0,
        -20.0,
        0.0,
        2,
    )
    projection = p.computeProjectionMatrixFOV(50.0, width_px / height_px, 0.01, 10.0)
    _, _, rgba, _, _ = p.getCameraImage(
        width_px,
        height_px,
        view,
        projection,
        renderer=p.ER_TINY_RENDERER,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape(height_px, width_px, 4)[:, :, :3]


def save_gif(frames: list[np.ndarray], path: Path, fps: int = 12) -> None:
    """Write RGB PyBullet frames as a looping animated GIF."""
    if not frames:
        raise ValueError("at least one frame is required to write a GIF")
    if fps <= 0:
        raise ValueError("GIF frames per second must be positive")

    from PIL import Image

    path.parent.mkdir(parents=True, exist_ok=True)
    images = [Image.fromarray(frame, mode="RGB") for frame in frames]
    images[0].save(
        path,
        format="GIF",
        save_all=True,
        append_images=images[1:],
        duration=round(1000 / fps),
        loop=0,
        optimize=False,
    )
