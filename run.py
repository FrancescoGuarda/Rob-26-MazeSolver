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
import multiprocessing
import os
import signal
import tempfile
from pathlib import Path

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

# PID lock file used to close a leftover legend window from a previous MMS
# "Run" click: each click spawns a fresh run.py process (see docs/mms.md
# Step 5), so there is no long-lived parent process to hold an in-memory
# handle to a prior run's legend Process object.
_LEGEND_LOCK = Path(tempfile.gettempdir()) / "mazesolver_legend.pid"


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
        "--heuristic", choices=["min_path", "manhattan"], default="min_path",
        help="Planning heuristic (astar only; ignored by dstar_lite)",
    )
    parser.add_argument(
        "--output-dir", default="results/logs/",
        help="Directory for the exported JSON log (default: results/logs/)",
    )
    parser.add_argument(
        "--no-log", action="store_true",
        help="Skip writing the JSON metrics log (stderr diagnostics are unaffected)",
    )
    args = parser.parse_args()
    if args.goals and args.n_goals is not None:
        parser.error("--goal and --n-goals are mutually exclusive")
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
    logger = MetricsLogger(args.algo, f"mms_{width}x{height}")

    goals = [tuple(g) for g in args.goals] if args.goals else None
    algorithm = _ALGORITHMS[args.algo](
        api, maze_map, robot, logger,
        goals=goals, n_random_goals=args.n_goals, random_seed=args.seed,
        heuristic=args.heuristic,
    )

    algorithm.run()

    if not args.no_log:
        logger.export_json(output_dir=args.output_dir)


if __name__ == "__main__":
    main()
