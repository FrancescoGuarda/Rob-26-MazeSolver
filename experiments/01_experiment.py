"""
Experiment 01 — A* and D*-Lite over the MAZES list
============================================================

Runs both algorithms in headless mode (SimAPI) on every maze listed in
MAZES.  Prints a summary table and saves JSON logs to
results/logs/<goal-count>/<algorithm>/.

Run from the repository root:
    python experiments/01_experiment.py

Optional flags:
    --goals X1,Y1 X2,Y2   explicit goal list (default: maze centre),
                            applied to every maze
    -k/--k-goals N         goal-count threshold: for each maze, runs every
                            scenario k=1..N, goals placed automatically via
                            src.goal_placement.scenario_goals (k=1: classic
                            micromouse centre goal; k>=2: detour placement).
                            Mutually exclusive with --goals.
    --log-dir PATH         override log output directory
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

# Ensure repo root is on sys.path when run as a standalone script
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI
from src.goal_placement import scenario_goals
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.parser import parse_maze
from src.robot import Robot

# ---------------------------------------------------------------------------
# Experiment settings (default values)
# ---------------------------------------------------------------------------

"""Default settings for batch experiments."""

MAZE_DIR = "mazes/txt"

# Mazes to run: every .txt in MAZE_DIR, by bare name (sorted for stable order)
MAZES = sorted(
    os.path.splitext(os.path.basename(p))[0]
    for p in glob.glob(os.path.join(MAZE_DIR, "*.txt"))
)

# Planning heuristic passed to both algorithms. A* honours it ("manhattan":
# straight-line |dx|+|dy|; "min_path": wall-aware BFS); D*-Lite ignores it
# and always uses Manhattan internally.
HEURISTIC = "manhattan"

# Output directory
LOG_DIR = "results/logs"

# Log name per algorithm class. Must match run.py's _ALGORITHMS keys: the name
# picks the log subdirectory, so headless and MMS runs of the same algorithm
# have to agree or they end up in two separate buckets.
ALGO_LOG_NAMES = {
    AStarExplorer:     "astar",
    DStarLiteExplorer: "dstar_lite",
}


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
    scenario: tuple[str, int, list[tuple[tuple[int, int], float]]] | None = None,
) -> tuple[dict, str]:
    """Run one algorithm on one maze and return (summary_dict, log_path).

    scenario, if given, is (maze_file, k, [(cell, detour), ...]) and is
    recorded in the exported log via MetricsLogger.set_scenario.
    """
    api = SimAPI(wall_matrix, width, height, start_x=0, start_y=0)
    maze_map = MazeMap(width, height)
    robot = Robot(x=0, y=0)
    algo_name = AlgoClass.__name__

    logger = MetricsLogger(ALGO_LOG_NAMES[AlgoClass], maze_name)
    if scenario is not None:
        maze_file, k, pairs = scenario
        logger.set_scenario(maze_file, k, pairs)
    algo = AlgoClass(api, maze_map, robot, logger, goals=goals, heuristic=HEURISTIC)
    logger.set_goal_count(algo.goal_count)

    t0 = time.time()
    algo.run()
    wall_time = time.time() - t0

    log_path = logger.export_json(output_dir=log_dir)

    summary = {
        "algorithm":            algo_name,
        "maze":                 maze_name,
        "goal_count":           algo.goal_count,
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
        ("maze",                 "<15"),
        ("algorithm",            "<20"),
        ("goal_count",           "<11"),
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
    parser = argparse.ArgumentParser(description="Experiment 01: A* and D*-Lite over the MAZES list")
    parser.add_argument(
        "--goals", nargs="*", metavar="X,Y",
        help="Goal coordinates as X,Y pairs (e.g. --goals 7,7 8,8). Default: maze centre.",
    )
    parser.add_argument(
        "-k", "--k-goals", type=int, default=None,
        help=(
            "Goal-count threshold: run every scenario k=1..N per maze, goals "
            "placed via src.goal_placement.scenario_goals (k=1: classic "
            "micromouse centre goal; k>=2: detour-index placement). "
            "Mutually exclusive with --goals."
        ),
    )
    parser.add_argument("--log-dir", default=LOG_DIR, help=f"Log output directory (default: {LOG_DIR})")
    args = parser.parse_args()

    if args.goals and args.k_goals is not None:
        parser.error("--goals and --k-goals/-k are mutually exclusive")
    if args.k_goals is not None and args.k_goals < 1:
        parser.error("--k-goals/-k must be >= 1")

    # Explicit goals from the command line apply to every maze
    cli_goals: list[tuple[int, int]] | None = None
    if args.goals:
        cli_goals = []
        for token in args.goals:
            x_str, y_str = token.split(",")
            cli_goals.append((int(x_str), int(y_str)))

    print(f"Log directory: {args.log_dir}")

    summaries: list[dict] = []
    log_paths: list[str] = []

    for maze in MAZES:
        maze_file = maze if maze.endswith(".txt") else f"{maze}.txt"
        maze_path = os.path.join(MAZE_DIR, maze_file)
        maze_name = os.path.splitext(maze_file)[0]

        print(f"\nLoading maze: {maze_path}")
        wall_matrix, width, height = parse_maze(maze_path)
        print(f"  Dimensions: {width}×{height}")

        # Derive this maze's runs as (goals, scenario) pairs: one run for
        # explicit/default goals, or a k=1..N sweep for --k-goals
        runs: list[tuple[list[tuple[int, int]] | None,
                         tuple[str, int, list[tuple[tuple[int, int], float]]] | None]] = []

        if cli_goals is not None:
            print(f"  Goals: {cli_goals}")
            runs.append((cli_goals, None))
        elif args.k_goals is not None:
            for k in range(1, args.k_goals + 1):
                try:
                    pairs = scenario_goals(wall_matrix, width, height, (0, 0), k)
                except ValueError as exc:
                    parser.error(str(exc))
                print(f"  Goals (k={k} scenario):")
                for cell, detour in pairs:
                    print(f"    {cell}  detour {detour:.3f}")
                runs.append(([cell for cell, _ in pairs], (maze_path, k, pairs)))
        else:
            print("  Goals: maze centre (default)")
            runs.append((None, None))

        for goals, scenario in runs:
            for AlgoClass in (AStarExplorer, DStarLiteExplorer):
                k_note = f" (k={scenario[1]})" if scenario is not None else ""
                print(f"Running {AlgoClass.__name__}{k_note}...")
                summary, log_path = _run(
                    AlgoClass, wall_matrix, width, height,
                    goals, maze_name, args.log_dir, scenario=scenario,
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

    # Exit with error if any run failed to reach its goal
    failures = [s for s in summaries if not s["goal_reached"]]
    if failures:
        print(f"\n[FAIL] {len(failures)} run(s) did not reach the goal.")
        sys.exit(1)
    else:
        print(f"[OK] All {len(summaries)} runs reached their goals successfully.")


if __name__ == "__main__":
    main()
