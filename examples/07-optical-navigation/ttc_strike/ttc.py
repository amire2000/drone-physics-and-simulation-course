"""Geometry-free bounding-box time-to-contact estimation."""

from dataclasses import dataclass
from math import sqrt

from .config import StrikeConfig


@dataclass(frozen=True)
class TtcObservation:
    box: tuple[int, int, int, int]
    scale_px: float
    scale_growth_px_s: float
    ttc_s: float


class BboxTtcTracker:
    """Estimate TTC from bbox growth without target size or camera calibration."""

    def __init__(self, config: StrikeConfig) -> None:
        self.config = config
        self.reset()

    def reset(self) -> None:
        self.last_scale_px: float | None = None
        self.last_time_s: float | None = None
        self.filtered_growth_px_s = 0.0
        self.commit_ready = False
        self.last_observation: TtcObservation | None = None

    def update(self, box: tuple[int, int, int, int] | None, now_s: float) -> TtcObservation | None:
        if box is None:
            return None
        _, _, width_px, height_px = box
        scale_px = sqrt(width_px * height_px)
        self.commit_ready = self.commit_ready or height_px >= self.config.commit_box_height_px
        if self.last_time_s is None:
            self.last_scale_px, self.last_time_s = scale_px, now_s
            return None
        dt_s = now_s - self.last_time_s
        if dt_s <= 0:
            return None
        growth_px_s = (scale_px - self.last_scale_px) / dt_s
        self.last_scale_px, self.last_time_s = scale_px, now_s
        old = self.config.ttc_growth_old_weight
        self.filtered_growth_px_s = old * self.filtered_growth_px_s + (1 - old) * growth_px_s
        if self.filtered_growth_px_s <= self.config.min_growth_px_per_s:
            return None
        observation = TtcObservation(box, scale_px, self.filtered_growth_px_s, scale_px / self.filtered_growth_px_s)
        self.last_observation = observation
        return observation
