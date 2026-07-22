#!/usr/bin/env python3
"""Manual inspection tool: print goal placement for a maze by detour index.

This is an inspection instrument only — nothing else in the repo reads its
output. Headless runs call ``src.goal_placement.scenario_goals`` directly at
load time (placement is deterministic and cheap, so there is no pre-step to
run and no file to keep in sync). Use this tool when you want to look at
where goals land in a maze and why, via the printed detour scores.

For a reference cell ``ref`` and a free cell ``c`` the detour index is

    detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)

where d_BFS is the true shortest-path distance in the maze (BFS,
4-connectivity). A cell with detour ~1 is "honest" about its distance;
a high-detour cell looks close but is far (e.g. just behind a wall).
Such deceptive cells defeat a greedy planner with a partial map, so the
detour index is the difficulty proxy for goal placement.

Placement algorithm (deterministic, no randomness):
  1. Goal 1 = argmax over free cells of detour(start, c). This is also the
     entire result for -k 1 — there is no special case; k=1 is simply the
     first step of the same algorithm, so it matches the first goal of any
     k>=2 placement.
  2. Goal k (k >= 2) = argmax of min(detour(ref, c) for ref in {start, goal_1,
     ..., goal_{k-1}}) — every previously placed goal is used as a reference,
     not just the last one.
  3. Ties break to the lowest (row, col) = lowest (y, x), y=0 at bottom.
  4. Scenarios are nested for every k >= 1: level L = the first L goals.

Usage:
    python3 tools/place_goals.py 2015japan
    python3 tools/place_goals.py maze_test.txt --start 0 0 -k 4
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.goal_placement import place_goals, Cell
from src.parser.maze_parser import parse_maze

_MAZE_DIR = Path("mazes/txt")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "maze",
        help="Maze name in mazes/txt, with or without the .txt extension "
        "(e.g. '2015japan' or '2015japan.txt')",
    )
    parser.add_argument(
        "--start", nargs=2, type=int, default=[0, 0], metavar=("X", "Y"),
        help="Start cell, y=0 at bottom (default: 0 0, the simulator convention)",
    )
    parser.add_argument(
        "-k", "--n-goals", type=int, default=4,
        help="Number of goals to place (default: 4)",
    )
    args = parser.parse_args()

    if args.n_goals < 1:
        parser.error(f"--n-goals must be >= 1, got {args.n_goals}")

    maze_name = args.maze if args.maze.endswith(".txt") else f"{args.maze}.txt"
    maze_path = _MAZE_DIR / maze_name

    try:
        wall_matrix, width, height = parse_maze(str(maze_path))
    except (FileNotFoundError, ValueError) as exc:
        sys.exit(f"error: {exc}")

    start: Cell = (args.start[0], args.start[1])
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        parser.error(f"start {start} out of bounds for {width}x{height} maze")

    print(f"Maze: {maze_path} ({width}x{height}), start {start}")

    steps = place_goals(wall_matrix, width, height, start, args.n_goals)
    if not steps:
        sys.exit(f"error: no reachable candidate cells from start {start}")

    for i, step in enumerate(steps, start=1):
        prev = (
            ", detour from goals: ["
            + ", ".join(f"{d:.3f}" for d in step.detour_from_goals)
            + "]"
            if step.detour_from_goals else ""
        )
        print(
            f"  Goal {i}: {step.goal}  score {step.score:.3f}"
            f"  (detour from start: {step.detour_from_start:.3f}{prev})"
        )

    gui_command = " ".join(f"--goal {g[0]} {g[1]}" for g in (step.goal for step in steps))
    print(f"\n  GUI command: {gui_command}")


if __name__ == "__main__":
    main()
