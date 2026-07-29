"""Unit tests for experiments/run_batch.py's goal-resolution helper."""
from __future__ import annotations

import os

import pytest

from experiments.run_batch import START, _resolve_goals
from src.goal_placement import scenario_goals
from src.parser.maze_parser import parse_maze

MAZE_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'mazes', 'maze_test.txt')
)


@pytest.fixture(scope="module")
def parsed():
    return parse_maze(MAZE_PATH)


def test_resolve_goals_k1_matches_scenario_goals(parsed):
    """k=1 must be resolved via scenario_goals(), not a goals=None fallback."""
    wall_matrix, width, height = parsed
    goals, scenario = _resolve_goals(wall_matrix, width, height, 1)
    pairs = scenario_goals(wall_matrix, width, height, START, 1)

    assert goals == [cell for cell, _ in pairs]
    k, returned_pairs = scenario
    assert k == 1
    assert returned_pairs == pairs
    assert len(goals) == 1


def test_resolve_goals_k1_nests_into_k2(parsed):
    """k=1 and k>=2 must go through the exact same placement call."""
    wall_matrix, width, height = parsed
    goals_1, scenario_1 = _resolve_goals(wall_matrix, width, height, 1)
    goals_2, scenario_2 = _resolve_goals(wall_matrix, width, height, 2)

    assert goals_2[:1] == goals_1
    assert scenario_2[1][:1] == scenario_1[1]
