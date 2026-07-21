"""
run_batch.py — Full-corpus batch test over mazes/txt/, all goal-count scenarios
=================================================================================

Runs both AStarExplorer and DStarLiteExplorer, headless (SimAPI), over every
maze in mazes/txt/, at four goal-count scenarios per maze:

    1 goal  — the default 4-cell centre area (goals=None; NOT a detour-index
              placement — see src.goal_placement.scenario_goals's k=1 note:
              that's a different, single fixed centre cell, a deliberately
              distinct scenario from this one)
    2, 3, 4 goals — placed automatically by src.goal_placement.scenario_goals,
              which maximizes the detour index (BFS distance / Manhattan
              distance) so goals land in the most deceptive-to-a-planner cells

Heuristic is fixed to "manhattan" for both algorithms.

Algorithms are constructed with verbose=False so
[WALL]/[REPLAN] stderr diagnostics don't flood the terminal across the
55 mazes x 4 goal-counts x 2 algorithms = up to 440 runs. Progress is a
single tqdm bar (live reached/failed counts in its postfix) plus a running
average table redrawn in place once per maze — via _LiveTable, using
tqdm.external_write_mode() so it never fights with the bar — rather than
appended anew each time, so the terminal doesn't scroll. On a non-TTY
stdout (piped to a file/CI log) the live redraw is skipped entirely; only
the final summary is printed, plainly, once.

Logs are written via MetricsLogger.export_json(), which already buckets by
goal count and algorithm (results/logs/<goal-count>/<algo>/).

Run from the repository root:
    python experiments/run_batch.py

Optional flags:
    --maze-dir PATH   override the maze corpus directory (default: mazes/txt)
    --log-dir PATH    override log output directory (default: results/logs)
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tqdm import tqdm

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI
from src.goal_placement import scenario_goals
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.parser import parse_maze
from src.robot import Robot

MAZE_DIR = "mazes/txt"
LOG_DIR = "results/logs"
GOAL_COUNTS = (1, 2, 3, 4)
HEURISTIC = "manhattan"
START = (0, 0)

ALGORITHMS = (AStarExplorer, DStarLiteExplorer)
ALGO_LOG_NAMES = {
    AStarExplorer:     "astar",
    DStarLiteExplorer: "dstar_lite",
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _discover_mazes(maze_dir: str) -> list[str]:
    """Bare maze names (no directory, no .txt), sorted, from every *.txt in maze_dir."""
    paths = sorted(glob.glob(os.path.join(maze_dir, "*.txt")))
    return [os.path.splitext(os.path.basename(p))[0] for p in paths]


def _resolve_goals(
    wall_matrix: list[list[int]], width: int, height: int, k: int,
) -> tuple[list[tuple[int, int]] | None, tuple[int, list] | None]:
    """Returns (goals, scenario) for goal-count k.

    k == 1 uses the default centre-area goal (goals=None, no scenario
    metadata). k >= 2 uses automated detour-index placement.
    """
    if k == 1:
        return None, None
    pairs = scenario_goals(wall_matrix, width, height, START, k)
    goals = [cell for cell, _ in pairs]
    return goals, (k, pairs)


def _run_one(
    AlgoClass,
    wall_matrix: list[list[int]],
    width: int,
    height: int,
    goals: list[tuple[int, int]] | None,
    maze_name: str,
    maze_path: str,
    log_dir: str,
    scenario: tuple[int, list] | None,
) -> dict:
    api = SimAPI(wall_matrix, width, height, start_x=START[0], start_y=START[1])
    maze_map = MazeMap(width, height)
    robot = Robot(x=START[0], y=START[1])

    logger = MetricsLogger(ALGO_LOG_NAMES[AlgoClass], maze_name)
    if scenario is not None:
        k, pairs = scenario
        logger.set_scenario(maze_path, k, pairs)

    algo = AlgoClass(
        api, maze_map, robot, logger,
        goals=goals, heuristic=HEURISTIC, verbose=False,
    )
    logger.set_goal_count(algo.goal_count)

    t0 = time.time()
    algo.run()
    wall_time = time.time() - t0

    log_path = logger.export_json(output_dir=log_dir)

    return {
        "maze":                 maze_name,
        "algorithm":            AlgoClass.__name__,
        "goal_count":           algo.goal_count,
        "goal_reached":         robot.position in set(algo._goals),
        "total_moves":          logger.total_moves,
        "replanning_events":    logger.total_replanning_events,
        "cumul_plan_time_s":    logger.cumulative_planning_time,
        "wall_time_s":          wall_time,
        "log_path":             log_path,
    }


def _bucket_key(result: dict) -> tuple[str, int]:
    return (result["algorithm"], result["goal_count"])


_SUMMARY_COLS = [
    ("algorithm",         "<20"),
    ("goal_count",        "<11"),
    ("runs",              "<6"),
    ("goal_reached_rate", "<18"),
    ("avg_total_moves",   "<16"),
    ("avg_replanning",    "<15"),
    ("avg_plan_time_s",   "<16"),
]


def _summary_lines(results: list[dict], failures: list[dict]) -> list[str]:
    """Render the running-average table + failures list as plain text lines.

    Pure (no I/O) so it can be reused both by the in-place live redraw during
    the run and by the plain one-shot print of the final summary.
    """
    buckets: dict[tuple[str, int], list[dict]] = defaultdict(list)
    for r in results:
        buckets[_bucket_key(r)].append(r)

    lines: list[str] = []
    header = "  ".join(f"{name:{fmt}}" for name, fmt in _SUMMARY_COLS)
    sep = "  ".join("-" * int(fmt[1:]) for _, fmt in _SUMMARY_COLS)
    lines.append(header)
    lines.append(sep)
    for (algo, goal_count), rows in sorted(buckets.items()):
        n = len(rows)
        reached_rate = sum(r["goal_reached"] for r in rows) / n
        avg_moves = sum(r["total_moves"] for r in rows) / n
        avg_replan = sum(r["replanning_events"] for r in rows) / n
        avg_plan_time = sum(r["cumul_plan_time_s"] for r in rows) / n
        row = {
            "algorithm":         algo,
            "goal_count":        goal_count,
            "runs":              n,
            "goal_reached_rate": f"{reached_rate:.0%}",
            "avg_total_moves":   f"{avg_moves:.1f}",
            "avg_replanning":    f"{avg_replan:.1f}",
            "avg_plan_time_s":   f"{avg_plan_time:.4f}",
        }
        lines.append("  ".join(f"{str(row[name]):{fmt}}" for name, fmt in _SUMMARY_COLS))
    if failures:
        lines.append(f"[SKIPPED/FAILED] {len(failures)} combination(s):")
        for f in failures:
            lines.append(f"  {f['maze']} | {f['algorithm']} | k={f['goal_count']} | {f['reason']}")
    return lines


class _LiveTable:
    """Redraws a small multi-line table in place, coexisting with a tqdm bar.

    On a real terminal, each render() erases the previous render (cursor up
    by its line count, clear to end of screen) inside tqdm.external_write_mode,
    which hides the progress bar for the duration of the write and redraws it
    fresh immediately below once we're done — so the bar never fights with
    our own cursor movement. When stdout isn't a TTY (piped to a file/CI
    log), rendering is a no-op: ANSI cursor codes would just corrupt a log
    file, and there is no "in place" on a stream with no cursor. Print the
    final summary separately, once, with plain tqdm.write calls instead.
    """

    def __init__(self) -> None:
        self._enabled = sys.stdout.isatty()
        self._prev_line_count = 0

    def render(self, lines: list[str]) -> None:
        if not self._enabled:
            return
        with tqdm.external_write_mode(file=sys.stdout):
            if self._prev_line_count:
                sys.stdout.write(f"\033[{self._prev_line_count}A\033[J")
            sys.stdout.write("\n".join(lines) + "\n")
            sys.stdout.flush()
        self._prev_line_count = len(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Batch-test A* and D*-Lite over the full mazes/txt/ corpus "
                     "at 1/2/3/4-goal scenarios."
    )
    parser.add_argument("--maze-dir", default=MAZE_DIR, help=f"Maze corpus directory (default: {MAZE_DIR})")
    parser.add_argument("--log-dir", default=LOG_DIR, help=f"Log output directory (default: {LOG_DIR})")
    args = parser.parse_args()

    maze_names = _discover_mazes(args.maze_dir)
    if not maze_names:
        parser.error(f"no *.txt mazes found in {args.maze_dir}")

    total = len(maze_names) * len(GOAL_COUNTS) * len(ALGORITHMS)
    results: list[dict] = []
    failures: list[dict] = []
    live_table = _LiveTable()
    reached_count = 0

    with tqdm(total=total, desc="run_batch", unit="run") as bar:
        for maze_name in maze_names:
            maze_path = os.path.join(args.maze_dir, f"{maze_name}.txt")
            wall_matrix, width, height = parse_maze(maze_path)

            for k in GOAL_COUNTS:
                try:
                    goals, scenario = _resolve_goals(wall_matrix, width, height, k)
                except ValueError as exc:
                    for AlgoClass in ALGORITHMS:
                        failures.append({
                            "maze": maze_name, "algorithm": AlgoClass.__name__,
                            "goal_count": k, "reason": str(exc),
                        })
                    bar.update(len(ALGORITHMS))
                    continue

                for AlgoClass in ALGORITHMS:
                    bar.set_description(f"{maze_name} | k={k} | {AlgoClass.__name__}")
                    try:
                        result = _run_one(
                            AlgoClass, wall_matrix, width, height,
                            goals, maze_name, maze_path, args.log_dir, scenario,
                        )
                        results.append(result)
                        reached_count += result["goal_reached"]
                    except Exception as exc:  # noqa: BLE001 - batch must not die on one bad run
                        failures.append({
                            "maze": maze_name, "algorithm": AlgoClass.__name__,
                            "goal_count": k, "reason": f"{type(exc).__name__}: {exc}",
                        })
                    bar.update(1)
                    bar.set_postfix(reached=f"{reached_count}/{len(results)}", failed=len(failures))

            live_table.render(_summary_lines(results, failures))

    tqdm.write("\n=== Final summary ===")
    for line in _summary_lines(results, failures):
        tqdm.write(line)
    tqdm.write(f"Logs saved under: {args.log_dir}")

    not_reached = [r for r in results if not r["goal_reached"]]
    if failures or not_reached:
        tqdm.write(
            f"\n[FAIL] {len(failures)} combination(s) failed/skipped, "
            f"{len(not_reached)} run(s) did not reach their goal."
        )
        sys.exit(1)
    else:
        tqdm.write(f"[OK] All {len(results)} runs reached their goals successfully.")


if __name__ == "__main__":
    main()
