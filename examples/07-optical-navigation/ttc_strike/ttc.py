"""Pure bounding-box time-to-contact estimation."""

from dataclasses import dataclass
from math import radians, sqrt, tan

from .config import StrikeConfig


@dataclass(frozen=True)
class TtcObservation:
    box: tuple[int, int, int, int]
    range_m: float
    ttc_s: float
    forward_velocity_mps: float


class BboxTtcTracker:
    """Turn target-box growth into range, TTC, and visual closing speed."""

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.focal_pixels = config.camera_height_px / (2 * tan(radians(config.camera_fov_deg) / 2))
        self.reset()

    def reset(self) -> None:
        self.last_scale: float | None = None
        self.last_time_s: float | None = None
        self.filtered_growth = 0.0
        self.commit_ready = False
        self.last_observation: TtcObservation | None = None

    def update(self, box: tuple[int, int, int, int] | None, now_s: float) -> TtcObservation | None:
        if box is None:
            return None
        _, _, width_px, height_px = box
        scale = sqrt(width_px * height_px)
        self.commit_ready = self.commit_ready or height_px >= self.config.commit_box_height_px
        if self.last_time_s is None:
            self.last_scale, self.last_time_s = scale, now_s
            return None
        growth = (scale - self.last_scale) / (now_s - self.last_time_s)
        self.last_scale, self.last_time_s = scale, now_s
        old = self.config.ttc_growth_old_weight
        self.filtered_growth = old * self.filtered_growth + (1 - old) * growth
        if self.filtered_growth <= self.config.min_growth_px_per_s:
            return None
        range_m = self.focal_pixels * self.config.target_size_m / scale
        ttc_s = scale / self.filtered_growth
        self.last_observation = TtcObservation(box, range_m, ttc_s, range_m / ttc_s)
        return self.last_observation
