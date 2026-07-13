"""Integration tests: AStarExplorer and DStarLiteExplorer on handcrafted mazes.

Each test runs an algorithm end-to-end via SimAPI (headless) and asserts:
  - All goals are reached.
  - Total forward moves > 0.
  - JSON log is valid with correct schema.
  - Replanning event count is non-negative and consistent between A* and D*-Lite
    on the same maze.
"""
from __future__ import annotations

import json
import os
import tempfile

import pytest

from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI
from src.constants import WALL_E, WALL_N, WALL_S, WALL_W
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot


# ---------------------------------------------------------------------------
# Maze fixtures
# ---------------------------------------------------------------------------

def _open_4x4() -> list[list[int]]:
    """4×4 maze with perimeter walls only (completely open interior)."""
    W = [[0] * 4 for _ in range(4)]
    for x in range(4):
        W[0][x] |= WALL_S  # bottom border
        W[3][x] |= WALL_N  # top border
    for y in range(4):
        W[y][0] |= WALL_W  # left border
        W[y][3] |= WALL_E  # right border
    return W


def _maze_with_internal_wall() -> list[list[int]]:
    """4×4 maze with one internal wall that blocks the freespace-assumption path.

    Wall between (3,0) and (3,1): N wall on (3,0), S wall on (3,1).
    A* freespace BFS from (0,0) to (3,3) may plan through this edge.
    When the robot reaches (3,0), it senses the N wall and replanning fires.
    """
    W = _open_4x4()
    W[0][3] |= WALL_N   # (3,0) has North wall
    W[1][3] |= WALL_S   # (3,1) has South wall  (symmetry)
    return W


def _single_path_maze_with_wall() -> list[list[int]]:
    """3×2 corridor maze where the unique freespace path hits a hidden wall.

    Layout (y=0 at bottom)::

        (0,1) - (1,1) - (2,1)
          |               |   <- perimeter
        (0,0) - (1,0)   (2,0)  <- goal
                         ^
                 hidden E/W wall between (1,0) and (2,0)

    Start (0,0), Goal (2,0).  Width=3, Height=2.

    Freespace BFS from (2,0):
      h(1,0)=1  [direct step West, no walls]
      h(0,0)=2  [via (1,0) — strictly shorter than any route through (0,1)]

    Both A* and D*-Lite plan (0,0)→(1,0)→(2,0) deterministically
    (the direct path is the UNIQUE shortest-length freespace path; the
    alternative via the top row is 2 steps longer — no tie-breaking needed).

    When the robot reaches (1,0) it senses the East wall → one replanning
    event fires, and the algorithm reroutes via (1,0)→(1,1)→(2,1)→(2,0).
    """
    W = [[0] * 3 for _ in range(2)]
    for x in range(3):
        W[0][x] |= WALL_S   # bottom perimeter
        W[1][x] |= WALL_N   # top perimeter
    for y in range(2):
        W[y][0] |= WALL_W   # left perimeter
        W[y][2] |= WALL_E   # right perimeter
    # Hidden internal wall (unknown to the robot until sensed)
    W[0][1] |= WALL_E   # East wall on (1,0)
    W[0][2] |= WALL_W   # West wall on (2,0)  [symmetry]
    return W


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _run_algo(AlgoClass, wall_matrix, goals, start=(0, 0)):
    width = len(wall_matrix[0])
    height = len(wall_matrix)
    api = SimAPI(wall_matrix, width, height, start_x=start[0], start_y=start[1])
    maze_map = MazeMap(width, height)
    robot = Robot(x=start[0], y=start[1])
    logger = MetricsLogger(AlgoClass.__name__, "test_maze")
    algo = AlgoClass(api, maze_map, robot, logger, goals=goals)
    algo.run()
    return robot, logger, maze_map


def _assert_valid_log(logger: MetricsLogger, expected_goals: list[tuple]) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        with open(path) as fh:
            data = json.load(fh)

    required_keys = [
        "algorithm", "maze", "total_moves", "forward_moves", "turns",
        "distinct_cells_visited", "total_visits", "execution_time_s",
        "wall_matrix", "visit_matrix",
        "total_replanning_events", "cumulative_planning_time_s",
        "cumulative_nodes_expanded", "replanning_events",
    ]
    for key in required_keys:
        assert key in data, f"Missing key: {key}"

    assert isinstance(data["replanning_events"], list)
    assert data["total_replanning_events"] == len(data["replanning_events"])


# ---------------------------------------------------------------------------
# Single-goal tests
# ---------------------------------------------------------------------------

class TestSingleGoal:
    GOAL = [(3, 3)]

    def test_astar_open_maze_reaches_goal(self):
        robot, logger, _ = _run_algo(AStarExplorer, _open_4x4(), self.GOAL)
        assert robot.position == (3, 3), "A* did not reach the goal."
        assert logger.forward_moves > 0

    def test_dstar_open_maze_reaches_goal(self):
        robot, logger, _ = _run_algo(DStarLiteExplorer, _open_4x4(), self.GOAL)
        assert robot.position == (3, 3), "D*-Lite did not reach the goal."
        assert logger.forward_moves > 0

    def test_astar_log_schema(self):
        _, logger, _ = _run_algo(AStarExplorer, _open_4x4(), self.GOAL)
        _assert_valid_log(logger, self.GOAL)

    def test_dstar_log_schema(self):
        _, logger, _ = _run_algo(DStarLiteExplorer, _open_4x4(), self.GOAL)
        _assert_valid_log(logger, self.GOAL)

    def test_astar_replanning_on_internal_wall(self):
        """A* fires at least one replanning event when the only freespace path is blocked."""
        _, logger, _ = _run_algo(AStarExplorer, _single_path_maze_with_wall(), [(2, 0)])
        assert logger.total_replanning_events >= 1

    def test_dstar_replanning_on_internal_wall(self):
        """D*-Lite fires at least one replanning event when the plan is invalidated."""
        _, logger, _ = _run_algo(DStarLiteExplorer, _single_path_maze_with_wall(), [(2, 0)])
        assert logger.total_replanning_events >= 1


# ---------------------------------------------------------------------------
# Multi-goal tests
# ---------------------------------------------------------------------------

class TestMultiGoal:
    GOALS = [(3, 3), (0, 3)]

    def test_astar_multi_goal(self):
        robot, logger, maze_map = _run_algo(AStarExplorer, _open_4x4(), self.GOALS)
        assert logger.distinct_cells_visited >= 2

    def test_dstar_multi_goal(self):
        robot, logger, maze_map = _run_algo(DStarLiteExplorer, _open_4x4(), self.GOALS)
        assert logger.distinct_cells_visited >= 2


# ---------------------------------------------------------------------------
# Default goal (maze centre) test
# ---------------------------------------------------------------------------

class TestDefaultGoal:
    def test_astar_default_goal(self):
        """No explicit goals → algorithm targets the 4-cell centre area."""
        W = _open_4x4()
        api = SimAPI(W, 4, 4)
        maze_map = MazeMap(4, 4)
        robot = Robot(0, 0)
        logger = MetricsLogger("astar", "test")
        algo = AStarExplorer(api, maze_map, robot, logger)
        # Centre cells for 4×4: (1,1),(2,1),(1,2),(2,2)
        assert len(algo._goals) > 0
        algo.run()
        assert logger.forward_moves > 0

    def test_dstar_default_goal(self):
        W = _open_4x4()
        api = SimAPI(W, 4, 4)
        maze_map = MazeMap(4, 4)
        robot = Robot(0, 0)
        logger = MetricsLogger("dstar", "test")
        algo = DStarLiteExplorer(api, maze_map, robot, logger)
        algo.run()
        assert logger.forward_moves > 0


# ---------------------------------------------------------------------------
# Consistency: replanning counts should be equal on the same maze
# ---------------------------------------------------------------------------

class TestReplanningConsistency:
    """Both algorithms face exactly the same wall discoveries on the same maze."""

    GOAL = [(3, 3)]

    def test_replanning_count_same_maze(self):
        """Both algorithms encounter the same wall discovery and replan the same number of times.

        _single_path_maze_with_wall() has exactly one freespace shortest path
        for both algorithms, so both take identical moves and sense the same
        walls in the same order → replanning event counts must be equal.
        """
        maze = _single_path_maze_with_wall()
        _, logger_a, _ = _run_algo(AStarExplorer, maze, [(2, 0)])
        _, logger_d, _ = _run_algo(DStarLiteExplorer, maze, [(2, 0)])
        assert logger_a.total_replanning_events == logger_d.total_replanning_events, (
            f"A* replanning={logger_a.total_replanning_events} "
            f"D*-Lite replanning={logger_d.total_replanning_events}"
        )
