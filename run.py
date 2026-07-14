#!/usr/bin/env python
"""
run.py — MMS GUI entry point.

This is the script path configured as the "Run command" for an algorithm in
the MMS simulator (see docs/mms.md, Step 3). It always talks to the real
simulator via MmsAPI (stdin/stdout protocol) — never SimAPI. For headless
batch runs against SimAPI, see experiments/run_batch.py.

Usage (as configured in MMS's "Run command" field):
    .venv/bin/python run.py --algo astar
    .venv/bin/python run.py --algo dstar_lite --goal 3 3 --goal 0 3
    .venv/bin/python run.py --algo astar --n-goals 4 --seed 42
"""
from __future__ import annotations

import argparse

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.mms_api import MmsAPI
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot

_ALGORITHMS = {
    "astar": AStarExplorer,
    "dstar_lite": DStarLiteExplorer,
}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a maze-solving algorithm in the MMS GUI simulator."
    )
    parser.add_argument(
        "--algo", choices=sorted(_ALGORITHMS), required=True,
        help="Algorithm to run",
    )
    parser.add_argument(
        "--goal", nargs=2, type=int, action="append", metavar=("X", "Y"), dest="goals",
        help="Goal cell (repeatable for multi-goal); mutually exclusive with --n-goals",
    )
    parser.add_argument(
        "--n-goals", type=int, default=None,
        help="Generate N random goal cells instead of an explicit --goal list",
    )
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Random seed used with --n-goals",
    )
    parser.add_argument(
        "--output-dir", default="results/logs/",
        help="Directory for the exported JSON log (default: results/logs/)",
    )
    args = parser.parse_args()
    if args.goals and args.n_goals is not None:
        parser.error("--goal and --n-goals are mutually exclusive")
    return args


def main() -> None:
    args = _parse_args()

    api = MmsAPI()
    width = api.maze_width()
    height = api.maze_height()
    maze_map = MazeMap(width, height)
    robot = Robot()
    logger = MetricsLogger(args.algo, f"mms_{width}x{height}")

    goals = [tuple(g) for g in args.goals] if args.goals else None
    algorithm = _ALGORITHMS[args.algo](
        api, maze_map, robot, logger,
        goals=goals, n_random_goals=args.n_goals, random_seed=args.seed,
    )

    algorithm.run()

    logger.export_json(output_dir=args.output_dir)


if __name__ == "__main__":
    main()
