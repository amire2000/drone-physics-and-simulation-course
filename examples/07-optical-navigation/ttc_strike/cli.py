"""Command-line adapter for the TTC diagonal-strike package."""

import argparse
from pathlib import Path

import cv2
import pybullet as p

from .config import StrikeConfig
from .guidance import FlightPhase, GuidanceInput, StrikeGuidance
from .sensing import BarometerReading
from .simulation import StrikeSimulation
from .trajectory import DiagonalTrajectory
from .ttc import BboxTtcTracker, TtcObservation


def self_check() -> None:
    config = StrikeConfig()
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation and observation.ttc_s > 0 and observation.forward_velocity_mps > 0
    tracker.update((0, 0, 30, int(config.commit_box_height_px)), 0.2)
    assert tracker.commit_ready, "A large bbox should arm terminal commit"
    trajectory = DiagonalTrajectory(config).command(0.0)
    assert abs(trajectory.forward_velocity_mps - config.terminal_vx_mps) < 0.01
    assert abs(trajectory.vertical_velocity_mps - config.terminal_vz_mps) < 0.01

    guidance = StrikeGuidance(config)
    reading = BarometerReading(config.takeoff_altitude_m, 0.0)
    guidance.update(GuidanceInput(0.0, reading, observation, observation, True, False))
    assert guidance.phase == FlightPhase.TRACK
    tracked = guidance.update(GuidanceInput(0.1, reading, observation, observation, True, True))
    committed = guidance.update(GuidanceInput(0.2, reading, None, observation, False, True))
    assert committed.phase == FlightPhase.COMMIT
    assert committed.thrust_n == tracked.thrust_n and committed.pitch_target_rad == tracked.pitch_target_rad
    print("TTC strike component self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=35.0)
    parser.add_argument("--video", type=Path, default=Path("outputs/ttc_diagonal_strike.mp4"))
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--plot", type=Path, default=Path("outputs/ttc_diagonal_strike.png"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            result = StrikeSimulation().run(not args.headless, args.max_seconds, None if args.no_video else args.video, None if args.no_plot else args.plot)
            if args.headless:
                assert result.success, f"Strike failed; impact speed was {result.impact_speed_mps:.1f} m/s"
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)
