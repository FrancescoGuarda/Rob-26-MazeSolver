"""Unit tests for src/goal_placement.py."""
from __future__ import annotations

import math
import os

import pytest

from src.constants import WALL_E, WALL_N, WALL_S, WALL_W
from src.goal_placement import place_goals, scenario_goals
from src.parser.maze_parser import parse_maze

MAZE_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'mazes', 'maze_test.txt')
)


@pytest.fixture(scope="module")
def parsed():
    return parse_maze(MAZE_PATH)


def _open_maze(width: int, height: int) -> list[list[int]]:
    """A fully-open maze (no internal walls) of the given size."""
    return [[0] * width for _ in range(height)]


def _sealed_cell_maze(width: int, height: int, sealed: tuple[int, int]) -> list[list[int]]:
    """An open maze except *sealed* is walled off on all four sides."""
    wall_matrix = _open_maze(width, height)
    sx, sy = sealed
    wall_matrix[sy][sx] = WALL_N | WALL_E | WALL_S | WALL_W
    # Also wall the neighbors' sides facing the sealed cell, so it's truly
    # unreachable (a one-sided wall would still let BFS enter from outside).
    if sy + 1 < height:
        wall_matrix[sy + 1][sx] |= WALL_S
    if sy - 1 >= 0:
        wall_matrix[sy - 1][sx] |= WALL_N
    if sx + 1 < width:
        wall_matrix[sy][sx + 1] |= WALL_W
    if sx - 1 >= 0:
        wall_matrix[sy][sx - 1] |= WALL_E
    return wall_matrix


# ---------------------------------------------------------------------------
# Determinism
# ---------------------------------------------------------------------------

def test_place_goals_deterministic(parsed):
    wall_matrix, width, height = parsed
    steps1 = place_goals(wall_matrix, width, height, (0, 0), 4)
    steps2 = place_goals(wall_matrix, width, height, (0, 0), 4)
    assert [s.goal for s in steps1] == [s.goal for s in steps2]
    assert [s.score for s in steps1] == [s.score for s in steps2]


def test_scenario_goals_deterministic(parsed):
    wall_matrix, width, height = parsed
    a = scenario_goals(wall_matrix, width, height, (0, 0), 3)
    b = scenario_goals(wall_matrix, width, height, (0, 0), 3)
    assert a == b


# ---------------------------------------------------------------------------
# Detour values
# ---------------------------------------------------------------------------

def test_detour_always_at_least_one(parsed):
    wall_matrix, width, height = parsed
    steps = place_goals(wall_matrix, width, height, (0, 0), 4)
    for s in steps:
        assert s.score >= 1.0
        assert s.detour_from_start >= 1.0
        for d in s.detour_from_goals:
            assert d >= 1.0


# ---------------------------------------------------------------------------
# Nesting (k >= 2)
# ---------------------------------------------------------------------------

def test_nesting_k_geq_2(parsed):
    wall_matrix, width, height = parsed
    goals_2 = scenario_goals(wall_matrix, width, height, (0, 0), 2)
    goals_3 = scenario_goals(wall_matrix, width, height, (0, 0), 3)
    assert goals_3[:2] == goals_2


# ---------------------------------------------------------------------------
# No duplicates / never the start
# ---------------------------------------------------------------------------

def test_place_goals_never_repeats_or_returns_start(parsed):
    wall_matrix, width, height = parsed
    start = (0, 0)
    steps = place_goals(wall_matrix, width, height, start, 4)
    cells = [s.goal for s in steps]
    assert start not in cells
    assert len(cells) == len(set(cells))


# ---------------------------------------------------------------------------
# k = 1: classic micromouse centre goal
# ---------------------------------------------------------------------------

def test_k1_returns_centre_cell(parsed):
    wall_matrix, width, height = parsed
    goals = scenario_goals(wall_matrix, width, height, (0, 0), 1)
    assert len(goals) == 1
    cell, detour = goals[0]
    assert cell == (width // 2 - 1, height // 2 - 1)
    assert cell == (7, 7)  # maze_test.txt is 16x16
    assert detour >= 1.0


def test_k1_centre_matches_manual_computation(parsed):
    wall_matrix, width, height = parsed
    from src.goal_placement import bfs_distance_map, manhattan

    goals = scenario_goals(wall_matrix, width, height, (0, 0), 1)
    cell, detour = goals[0]
    dist = bfs_distance_map(wall_matrix, width, height, (0, 0))
    expected = dist[cell[1]][cell[0]] / manhattan((0, 0), cell)
    assert detour == pytest.approx(expected)


def test_k1_raises_when_centre_is_start():
    # 2x2 open maze: centre = (2//2-1, 2//2-1) = (0, 0) = start.
    wall_matrix = _open_maze(2, 2)
    with pytest.raises(ValueError, match="coincides with start"):
        scenario_goals(wall_matrix, 2, 2, (0, 0), 1)


def test_k1_raises_when_centre_unreachable():
    width, height = 8, 8
    centre = (width // 2 - 1, height // 2 - 1)
    wall_matrix = _sealed_cell_maze(width, height, centre)
    with pytest.raises(ValueError, match="unreachable"):
        scenario_goals(wall_matrix, width, height, (0, 0), 1)


def test_k1_raises_on_out_of_bounds_would_not_happen_for_valid_dims():
    # Sanity: for any width/height >= 2, the centre cell is always in bounds
    # by construction (width // 2 - 1 in [0, width-2], same for height).
    for w in (2, 3, 4, 16):
        for h in (2, 3, 4, 16):
            cx, cy = w // 2 - 1, h // 2 - 1
            assert 0 <= cx < w
            assert 0 <= cy < h


def test_scenario_goals_rejects_k_less_than_one(parsed):
    wall_matrix, width, height = parsed
    with pytest.raises(ValueError, match="k must be >= 1"):
        scenario_goals(wall_matrix, width, height, (0, 0), 0)
