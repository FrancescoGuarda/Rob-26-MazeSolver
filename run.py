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
    .venv/bin/python run.py --algo astar --auto-goals 2015japan -k 4
"""
from __future__ import annotations

import argparse
import multiprocessing
import os
import signal
import sys
import tempfile
from pathlib import Path

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.mms_api import MmsAPI
from src.goal_placement import scenario_goals
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.parser.maze_parser import parse_maze
from src.robot import Robot

_ALGORITHMS = {
    "astar": AStarExplorer,
    "dstar_lite": DStarLiteExplorer,
}

# PID lock file used to close a leftover legend window from a previous MMS
# "Run" click: each click spawns a fresh run.py process (see docs/mms.md
# Step 5), so there is no long-lived parent process to hold an in-memory
# handle to a prior run's legend Process object.
_LEGEND_LOCK = Path(tempfile.gettempdir()) / "mazesolver_legend.pid"

# Anchors for --auto-goals maze lookup, resolved from this file's location so
# the flag does not depend on the working directory MMS launches run.py with.
_REPO_ROOT = Path(__file__).resolve().parent
_MAZE_DIR = _REPO_ROOT / "mazes" / "txt"


def _resolve_maze_path(maze: str) -> Path:
    """Resolve a --auto-goals argument to a maze file path.

    A bare name ('2015japan', '.txt' optional) resolves against mazes/txt/;
    anything containing a separator is a path, relative ones to the repo root.
    """
    name = maze if maze.endswith(".txt") else f"{maze}.txt"
    path = Path(name)
    if path.is_absolute():
        return path
    if len(path.parts) > 1:
        return _REPO_ROOT / path
    return _MAZE_DIR / name


def _auto_goals(
    maze: str, start: tuple[int, int], k: int, width: int, height: int,
) -> list[tuple[int, int]]:
    """Detour placement for *maze*, returned as an explicit goal list.

    The in-process equivalent of pasting the '--goal X Y ...' line printed by
    tools/place_goals.py: both call scenario_goals(), which is deterministic,
    so the goals are identical. The maze must be named explicitly because the
    MMS protocol exposes only its dimensions, never the loaded file; those
    dimensions are checked here to catch a stale name of a different size.

    Exits with a stderr message if the file is missing, malformed, disagrees
    with the simulator's dimensions, or admits no valid placement.
    """
    maze_path = _resolve_maze_path(maze)
    try:
        wall_matrix, maze_width, maze_height = parse_maze(str(maze_path))
    except (FileNotFoundError, ValueError) as exc:
        sys.exit(f"error: --auto-goals: {exc}")

    if (maze_width, maze_height) != (width, height):
        sys.exit(
            f"error: --auto-goals: '{maze_path}' is {maze_width}x{maze_height}, "
            f"but the simulator reports {width}x{height} — the maze loaded in "
            f"the GUI is not the one named here"
        )

    try:
        goals = scenario_goals(wall_matrix, maze_width, maze_height, start, k)
    except ValueError as exc:
        sys.exit(f"error: --auto-goals: {exc}")
    if not goals:
        sys.exit(f"error: --auto-goals: no reachable cells from start {start}")

    return [cell for cell, _detour in goals]


def _maze_name(args: argparse.Namespace, width: int, height: int) -> str:
    """Maze label for the log filename, best source first.

    MMS reports only the maze dimensions, never which file is loaded, so the
    name has to come from the invocation: --maze-name if given, otherwise the
    maze --auto-goals already named. Falls back to the dimensions alone.
    """
    if args.maze_name:
        return args.maze_name
    if args.auto_goals is not None:
        return _resolve_maze_path(args.auto_goals).stem
    return f"mms_{width}x{height}"


def _close_existing_legend() -> None:
    """Terminate a legend window left over from a previous run, if any."""
    if not _LEGEND_LOCK.exists():
        return
    try:
        pid = int(_LEGEND_LOCK.read_text().strip())
        os.kill(pid, signal.SIGTERM)
    except (ValueError, ProcessLookupError, PermissionError):
        pass
    _LEGEND_LOCK.unlink(missing_ok=True)


def _show_legend(algo_name: str, legend: list[tuple[str, str]]) -> None:
    """Display a static Tkinter legend window mapping GUI symbols to meanings.

    Runs in its own process (see main()): Tkinter must own the main thread of
    whatever process drives it, and on macOS running it on a background *thread*
    of the main process fails, so it gets a dedicated process instead. The
    window is purely informational and never updates after creation.
    """
    import tkinter as tk

    from src.constants import COLOR_HEX

    _LEGEND_LOCK.write_text(str(os.getpid()))
    try:
        root = tk.Tk()
        root.title(f"{algo_name} — GUI legend")
        tk.Label(root, text="GUI Legend", font=("Arial", 12, "bold")).grid(
            row=0, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 10),
        )
        for row, (symbol, meaning) in enumerate(legend, start=1):
            code = symbol.split()[0]  # leading token, e.g. 'b' or 'f-XXX'
            hex_color = COLOR_HEX.get(code)  # None for text-format rows
            canvas = tk.Canvas(
                root, width=22, height=22,
                highlightthickness=1, highlightbackground="black",
            )
            if hex_color:
                canvas.create_rectangle(0, 0, 22, 22, fill=hex_color, outline=hex_color)
            canvas.grid(row=row, column=0, padx=6, pady=2)
            tk.Label(root, text=symbol, anchor="w", font=("Arial", 10)).grid(
                row=row, column=1, sticky="w", padx=6, pady=2,
            )
            tk.Label(root, text=meaning, anchor="w", font=("Arial", 10)).grid(
                row=row, column=2, sticky="w", padx=6, pady=2,
            )
        root.mainloop()
    finally:
        _LEGEND_LOCK.unlink(missing_ok=True)


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
        help="Goal cell (repeatable for multi-goal); mutually exclusive with "
        "--n-goals and --auto-goals",
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
        "--auto-goals", metavar="MAZE", default=None,
        help="Place goals by detour index in MAZE (name in mazes/txt, .txt "
        "optional, or a path) instead of listing them with --goal; must name "
        "the maze loaded in the GUI. Mutually exclusive with --goal/--n-goals",
    )
    parser.add_argument(
        "-k", "--n-auto-goals", type=int, default=None, metavar="N",
        help="Number of goals to place with --auto-goals (default: 4)",
    )
    parser.add_argument(
        "--heuristic", choices=["min_path", "manhattan"], default="min_path",
        help="Planning heuristic (astar only; ignored by dstar_lite)",
    )
    parser.add_argument(
        "--maze-name",
        help="Maze name recorded in the log (default: the --auto-goals maze, "
             "else mms_<width>x<height>; MMS never reports the loaded file)",
    )
    parser.add_argument(
        "--output-dir", default="results/logs/",
        help="Base directory for the exported JSON log; written to "
        "<dir>/<goal-count>/<algo>/ (default: results/logs/)",
    )
    parser.add_argument(
        "--no-log", action="store_true",
        help="Skip writing the JSON metrics log (stderr diagnostics are unaffected)",
    )
    args = parser.parse_args()
    given = [
        name for name, value in (
            ("--goal", args.goals),
            ("--n-goals", args.n_goals),
            ("--auto-goals", args.auto_goals),
        ) if value is not None
    ]
    if len(given) > 1:
        parser.error(f"{', '.join(given[:-1])} and {given[-1]} are mutually exclusive")
    # Flags that only mean something alongside the option they modify: reject
    # them outright rather than accepting a value that would be ignored.
    if args.seed is not None and args.n_goals is None:
        parser.error("--seed requires --n-goals")
    if args.n_auto_goals is not None:
        if args.auto_goals is None:
            parser.error("-k/--n-auto-goals requires --auto-goals")
        if args.n_auto_goals < 1:
            parser.error(f"-k/--n-auto-goals must be >= 1, got {args.n_auto_goals}")
    return args


def main() -> None:
    args = _parse_args()

    # Launch the GUI legend in its own process (read off the class, before
    # constructing the algorithm). Daemon so it dies with the main process
    # when the MMS run ends. Close any window left over from a previous run
    # first, so consecutive MMS "Run" clicks don't stack up legend windows.
    legend = _ALGORITHMS[args.algo].LEGEND
    if legend:
        _close_existing_legend()
        multiprocessing.Process(
            target=_show_legend, args=(args.algo, legend), daemon=True,
        ).start()

    api = MmsAPI()
    width = api.maze_width()
    height = api.maze_height()
    maze_map = MazeMap(width, height)
    robot = Robot()

    if args.auto_goals is not None:
        k = 4 if args.n_auto_goals is None else args.n_auto_goals
        goals = _auto_goals(args.auto_goals, robot.position, k, width, height)
    else:
        goals = [tuple(g) for g in args.goals] if args.goals else None

    logger = MetricsLogger(args.algo, _maze_name(args, width, height))
    algorithm = _ALGORITHMS[args.algo](
        api, maze_map, robot, logger,
        goals=goals, n_random_goals=args.n_goals, random_seed=args.seed,
        heuristic=args.heuristic,
    )
    # Read back from the algorithm rather than from args: it is what resolved
    # --goal / --n-goals / --auto-goals / the default centre goal into cells.
    logger.set_goal_count(algorithm.goal_count)

    algorithm.run()

    if not args.no_log:
        logger.export_json(output_dir=args.output_dir)


if __name__ == "__main__":
    main()
