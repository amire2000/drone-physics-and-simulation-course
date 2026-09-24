"""Command-line adapter for the TTC diagonal-strike package."""

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path

import cv2
import pybullet as p

from .config import SceneConfig, StrikeConfig
from .guidance import FlightPhase, GuidanceInput, StrikeGuidance
from .sensing import BarometerReading
from .simulation import StrikeSimulation
from .trajectory import TtcDescentPlanner
from .ttc import BboxTtcTracker


def self_check() -> None:
    config = StrikeConfig()
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation and observation.ttc_s > 0 and observation.scale_growth_px_s > 0
    tracker.update((0, 0, 30, int(config.commit_box_height_px)), 0.2)
    assert tracker.commit_ready, "A large bbox should arm terminal commit"
    planner = TtcDescentPlanner(config)
    trajectory = planner.command(1.0, config.takeoff_altitude_m)
    assert trajectory.forward_velocity_mps == config.forward_speed_mps
    assert trajectory.vertical_velocity_mps == -config.max_descent_velocity_mps
    assert planner.command(None, config.takeoff_altitude_m).vertical_velocity_mps == 0.0
    assert planner.command(None, config.takeoff_altitude_m).altitude_target_m == config.takeoff_altitude_m

    guidance = StrikeGuidance(config)
    reading = BarometerReading(config.takeoff_altitude_m, 0.0)
    guidance.update(GuidanceInput(0.0, reading, observation, observation, True, False))
    assert guidance.phase == FlightPhase.TRACK
    tracked = guidance.update(GuidanceInput(0.1, reading, observation, observation, True, True))
    assert tracked.pitch_target_rad > 0.0 and tracked.thrust_n > 0.0
    hold = guidance.update(GuidanceInput(0.2, reading, None, observation, True, True))
    assert hold.pitch_target_rad == config.nominal_pitch_rad and hold.trajectory.vertical_velocity_mps == 0.0
    committed = guidance.update(GuidanceInput(0.3, reading, None, observation, False, True))
    assert committed.phase == FlightPhase.COMMIT
    assert committed.thrust_n == hold.thrust_n and committed.pitch_target_rad == hold.pitch_target_rad
    print("TTC strike component self-check passed")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=35.0)
    parser.add_argument("--output-root", type=Path, default=Path("outputs/ttc_runs"))
    parser.add_argument("--run-name", type=str)
    parser.add_argument("--video", type=Path)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--no-plot", action="store_true")
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--no-csv", action="store_true")
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            config, scene = StrikeConfig(), SceneConfig()
            run_name = args.run_name or datetime.now().strftime("run-%Y%m%d-%H%M%S-%f")
            run_dir = args.output_root / run_name
            run_dir.mkdir(parents=True, exist_ok=False)
            (run_dir / "settings.json").write_text(json.dumps({"strike": asdict(config), "scene": asdict(scene)}, indent=2) + "\n")
            video = args.video or run_dir / "environment.mp4"
            plot = args.plot or run_dir / "telemetry.png"
            csv = args.csv or run_dir / "telemetry.csv"
            result = StrikeSimulation(config, scene).run(not args.headless, args.max_seconds, None if args.no_video else video, None if args.no_plot else plot, None if args.no_csv else csv)
            print(f"run folder: {run_dir}")
            if args.headless:
                assert result.success, f"Strike failed; impact speed was {result.impact_speed_mps:.1f} m/s"
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)
