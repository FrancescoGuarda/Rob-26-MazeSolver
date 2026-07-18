"""Unit tests for run.py's --auto-goals helpers."""
from __future__ import annotations

import os

import pytest

import run
from src.goal_placement import scenario_goals
from src.parser.maze_parser import parse_maze

MAZE_NAME = "2015japan"
MAZE_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'mazes', 'txt', f'{MAZE_NAME}.txt')
)
START = (0, 0)


@pytest.fixture(scope="module")
def parsed():
    return parse_maze(MAZE_PATH)


# ----------------------------------------------------------------------
# Maze path resolution
# ----------------------------------------------------------------------

@pytest.mark.parametrize("argument", [MAZE_NAME, f"{MAZE_NAME}.txt"])
def test_bare_name_resolves_against_maze_dir(argument):
    """A bare name, .txt optional, resolves into mazes/txt/."""
    assert run._resolve_maze_path(argument) == run._MAZE_DIR / f"{MAZE_NAME}.txt"


def test_relative_path_resolves_against_repo_root():
    """A path with a separator is taken as a path, anchored at the repo root."""
    resolved = run._resolve_maze_path("mazes/maze_test.txt")
    assert resolved == run._REPO_ROOT / "mazes" / "maze_test.txt"


def test_absolute_path_is_used_as_given():
    assert str(run._resolve_maze_path(MAZE_PATH)) == MAZE_PATH


# ----------------------------------------------------------------------
# Placement equivalence
# ----------------------------------------------------------------------

@pytest.mark.parametrize("k", [1, 2, 4])
def test_matches_scenario_goals(parsed, k):
    """--auto-goals returns exactly what the shared placement routine yields.

    This is the contract that lets the flag replace pasting the goal list
    printed by tools/place_goals.py: same maze, same start, same k, same goals.
    """
    wall_matrix, width, height = parsed
    expected = [cell for cell, _detour in
                scenario_goals(wall_matrix, width, height, START, k)]

    assert run._auto_goals(MAZE_NAME, START, k, width, height) == expected


def test_placement_is_nested_for_k_ge_2(parsed):
    """Level L of a k >= 2 placement is the first L goals (see goal_placement)."""
    _, width, height = parsed
    assert (run._auto_goals(MAZE_NAME, START, 2, width, height)
            == run._auto_goals(MAZE_NAME, START, 4, width, height)[:2])


# ----------------------------------------------------------------------
# Failure modes: all abort rather than falling back to other goals
# ----------------------------------------------------------------------

def test_dimension_mismatch_aborts(parsed):
    """A maze of a different size than the simulator's is a stale --auto-goals."""
    _, width, height = parsed
    with pytest.raises(SystemExit) as exc:
        run._auto_goals(MAZE_NAME, START, 4, width // 2, height // 2)
    assert "not the one named here" in str(exc.value)


def test_missing_maze_aborts(parsed):
    _, width, height = parsed
    with pytest.raises(SystemExit) as exc:
        run._auto_goals("no_such_maze", START, 4, width, height)
    assert "--auto-goals" in str(exc.value)


def test_invalid_k_aborts(parsed):
    """k < 1 is rejected by scenario_goals and surfaced as a clean exit."""
    wall_matrix, width, height = parsed
    with pytest.raises(SystemExit) as exc:
        run._auto_goals(MAZE_NAME, START, 0, width, height)
    assert "--auto-goals" in str(exc.value)
