"""Lifecycle orchestration shared by independent Module 6 topic examples."""

import argparse
from collections.abc import Callable

import numpy as np
import pybullet as p

from .drone_model import DroneProfile, load_drone_profile
from .pybullet_recording import save_gif
from .simulation_utils import wait_for_exit
from .telemetry import Sample, print_summary, save_results
from .tk_controls import TkSimulationControls

CreateWorld = Callable[[DroneProfile], tuple[int, DroneProfile]]
PyBulletLoop = Callable[[int, DroneProfile, argparse.Namespace, list[np.ndarray] | None, TkSimulationControls | None], list[Sample]]
ReducedLoop = Callable[[DroneProfile, argparse.Namespace], list[Sample]]
Validator = Callable[[list[Sample], DroneProfile, argparse.Namespace], None]


def run_topic(
    args: argparse.Namespace,
    *,
    create_world: CreateWorld,
    run_pybullet: PyBulletLoop,
    run_reduced: ReducedLoop,
    validate: Validator,
    graph_fields: tuple[str, ...],
    summary_title: str,
    graph_title: str,
    gif_fps: int = 12,
) -> None:
    """Run one topic while leaving its physics loops and validation topic-owned."""
    profile = load_drone_profile("real_reference")
    if args.backend == "reduced":
        if args.gif or args.output:
            raise ValueError("--gif and --output require the PyBullet backend")
        samples = run_reduced(profile, args)
        print_summary(samples, summary_title)
        if args.self_check:
            validate(samples, profile, args)
        return

    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    controls: TkSimulationControls | None = None
    try:
        drone, profile = create_world(profile)
        frames: list[object] | None = [] if args.gif else None
        if not args.headless and not args.self_check:
            controls = TkSimulationControls(summary_title)
        samples = run_pybullet(drone, profile, args, frames, controls)
        print_summary(samples, summary_title)
        if args.output:
            save_results(samples, args.output, profile, graph_fields, graph_title)
            print(f"Saved graph: {args.output}")
        if args.gif:
            save_gif(frames or [], args.gif, gif_fps)
            print(f"Saved GIF: {args.gif}")
        if args.self_check:
            validate(samples, profile, args)
            if args.gif:
                from PIL import Image

                with Image.open(args.gif) as animation:
                    assert animation.n_frames > 1, "The GIF should contain multiple scene frames"
        elif not args.headless and not (controls and controls.closed):
            print("Experiment complete. Press Q or Esc in the PyBullet window to exit.")
            wait_for_exit()
    finally:
        if controls is not None:
            controls.close()
        if p.isConnected(client):
            p.disconnect(client)
