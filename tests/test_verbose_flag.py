"""Verbose flag: verbose=False must suppress all [WALL]/[REPLAN] stderr output.

Reuses tests.test_integration's _single_path_maze_with_wall fixture, which is
guaranteed to produce at least one [WALL] line (the hidden internal wall) and
at least one [REPLAN] line (the replan it triggers) for both explorers.
"""
from __future__ import annotations

import pytest

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot
from tests.test_integration import _single_path_maze_with_wall

GOAL = [(2, 0)]


def _run(AlgoClass, verbose: bool | None):
    wall_matrix = _single_path_maze_with_wall()
    width, height = 3, 2
    api = SimAPI(wall_matrix, width, height, start_x=0, start_y=0)
    maze_map = MazeMap(width, height)
    robot = Robot(x=0, y=0)
    logger = MetricsLogger(AlgoClass.__name__, "test_maze")
    kwargs = {} if verbose is None else {"verbose": verbose}
    algo = AlgoClass(api, maze_map, robot, logger, goals=GOAL, **kwargs)
    algo.run()
    return logger


@pytest.mark.parametrize("AlgoClass", [AStarExplorer, DStarLiteExplorer])
def test_verbose_false_suppresses_stderr(AlgoClass, capsys):
    logger = _run(AlgoClass, verbose=False)
    captured = capsys.readouterr()
    assert captured.err == ""
    # Suppressing stderr must not affect what gets logged.
    assert logger.total_replanning_events >= 1


@pytest.mark.parametrize("AlgoClass", [AStarExplorer, DStarLiteExplorer])
def test_verbose_true_preserves_stderr(AlgoClass, capsys):
    _run(AlgoClass, verbose=True)
    captured = capsys.readouterr()
    assert "[WALL]" in captured.err
    assert "[REPLAN]" in captured.err


@pytest.mark.parametrize("AlgoClass", [AStarExplorer, DStarLiteExplorer])
def test_verbose_defaults_to_true(AlgoClass, capsys):
    """Omitting verbose entirely must reproduce current (pre-flag) behavior."""
    _run(AlgoClass, verbose=None)
    captured = capsys.readouterr()
    assert "[WALL]" in captured.err
    assert "[REPLAN]" in captured.err
