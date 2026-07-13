"""
Experiment 01 — Smoke test: A* and D*-Lite on maze_test.txt
============================================================

Verifies correct end-to-end execution of both algorithms in headless mode
(SimAPI) on the standard 16×16 test maze.  Prints a summary table and saves
JSON logs to results/logs/.

Run from the repository root:
    python experiments/01_experiment.py

Optional flags:
    --goals X1,Y1 X2,Y2   explicit goal list (default: maze centre)
    --log-dir PATH         override log output directory
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# Ensure repo root is on sys.path when run as a standalone script
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.parser import parse_maze
from src.robot import Robot

# ---------------------------------------------------------------------------
# Experiment settings (default values)
# ---------------------------------------------------------------------------

"""Default settings for batch experiments."""

# Single test maze (used in development experiments)
MAZE_TEST = "mazes/maze_test.txt"

# Output directory
LOG_DIR = "results/logs"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run(
    AlgoClass,
    wall_matrix: list[list[int]],
    width: int,
    height: int,
    goals: list[tuple[int, int]] | None,
    maze_name: str,
    log_dir: str,
) -> tuple[dict, str]:
    """Run one algorithm on one maze and return (summary_dict, log_path)."""
    api = SimAPI(wall_matrix, width, height, start_x=0, start_y=0)
    maze_map = MazeMap(width, height)
    robot = Robot(x=0, y=0)
    algo_name = AlgoClass.__name__

    logger = MetricsLogger(algo_name, maze_name)
    algo = AlgoClass(api, maze_map, robot, logger, goals=goals)

    t0 = time.time()
    algo.run()
    wall_time = time.time() - t0

    log_path = logger.export_json(output_dir=log_dir)

    summary = {
        "algorithm":            algo_name,
        "maze":                 maze_name,
        "final_position":       robot.position,
        "goals":                algo._goals,
        "goal_reached":         robot.position in set(algo._goals),
        "forward_moves":        logger.forward_moves,
        "turns":                logger.turns,
        "total_moves":          logger.total_moves,
        "distinct_cells":       logger.distinct_cells_visited,
        "total_visits":         logger.total_visits,
        "replanning_events":    logger.total_replanning_events,
        "cumul_plan_time_s":    round(logger.cumulative_planning_time, 6),
        "cumul_nodes_expanded": logger.cumulative_nodes_expanded,
        "execution_time_s":     round(logger.execution_time or 0.0, 4),
        "wall_time_s":          round(wall_time, 4),
    }
    return summary, log_path


def _print_table(summaries: list[dict]) -> None:
    cols = [
        ("algorithm",            "<20"),
        ("goal_reached",         "<13"),
        ("forward_moves",        "<14"),
        ("total_moves",          "<13"),
        ("distinct_cells",       "<15"),
        ("replanning_events",    "<20"),
        ("cumul_plan_time_s",    "<20"),
        ("execution_time_s",     "<18"),
    ]
    header = "  ".join(f"{name:{fmt}}" for name, fmt in cols)
    sep = "  ".join("-" * int(fmt[1:]) for _, fmt in cols)
    print("\n" + header)
    print(sep)
    for s in summaries:
        row = "  ".join(f"{str(s[name]):{fmt}}" for name, fmt in cols)
        print(row)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Experiment 01: A* and D*-Lite on maze_test.txt")
    parser.add_argument(
        "--goals", nargs="*", metavar="X,Y",
        help="Goal coordinates as X,Y pairs (e.g. --goals 7,7 8,8). Default: maze centre.",
    )
    parser.add_argument("--log-dir", default=LOG_DIR, help=f"Log output directory (default: {LOG_DIR})")
    args = parser.parse_args()

    # Parse goals
    goals: list[tuple[int, int]] | None = None
    if args.goals:
        goals = []
        for token in args.goals:
            x_str, y_str = token.split(",")
            goals.append((int(x_str), int(y_str)))

    maze_path = MAZE_TEST
    maze_name = os.path.splitext(os.path.basename(maze_path))[0]

    print(f"Loading maze: {maze_path}")
    wall_matrix, width, height = parse_maze(maze_path)
    print(f"  Dimensions: {width}×{height}")
    print(f"  Goals: {'maze centre (default)' if goals is None else goals}")
    print(f"  Log directory: {args.log_dir}")
    print()

    summaries: list[dict] = []
    log_paths: list[str] = []

    for AlgoClass in (AStarExplorer, DStarLiteExplorer):
        print(f"Running {AlgoClass.__name__}...")
        summary, log_path = _run(
            AlgoClass, wall_matrix, width, height,
            goals, maze_name, args.log_dir,
        )
        summaries.append(summary)
        log_paths.append(log_path)
        goal_status = "✓ REACHED" if summary["goal_reached"] else "✗ NOT REACHED"
        print(f"  {goal_status} | final pos: {summary['final_position']} | "
              f"moves: {summary['total_moves']} | "
              f"replanning events: {summary['replanning_events']}")

    print("\n=== Summary ===")
    _print_table(summaries)

    print("Logs saved:")
    for p in log_paths:
        print(f"  {p}")

    # Exit with error if any algorithm failed to reach its goal
    failures = [s for s in summaries if not s["goal_reached"]]
    if failures:
        print(f"\n[FAIL] {len(failures)} algorithm(s) did not reach the goal.")
        sys.exit(1)
    else:
        print("[OK] Both algorithms reached their goals successfully.")


if __name__ == "__main__":
    main()
