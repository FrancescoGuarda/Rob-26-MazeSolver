"""Unit tests for maze_parser (ASCII .txt format)."""
from __future__ import annotations

import os
import pytest

from src.constants import WALL_E, WALL_N, WALL_S, WALL_W
from src.parser.maze_parser import parse_maze

# Resolve path relative to this test file so tests run from any cwd.
MAZE_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), '..', 'mazes', 'maze_test.txt')
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def parsed():
    """Parse maze_test.txt once for all tests in this module."""
    wall_matrix, width, height = parse_maze(MAZE_PATH)
    return wall_matrix, width, height


# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

def test_parse_maze_test_dimensions(parsed):
    _, width, height = parsed
    assert width == 16
    assert height == 16


def test_wall_matrix_shape(parsed):
    wall_matrix, width, height = parsed
    assert len(wall_matrix) == height
    assert all(len(row) == width for row in wall_matrix)


# ---------------------------------------------------------------------------
# Perimeter walls — all border cells must have their outer wall set
# ---------------------------------------------------------------------------

def test_perimeter_south_row(parsed):
    """Every cell in the bottom row must have a South perimeter wall."""
    wall_matrix, width, _ = parsed
    for x in range(width):
        assert wall_matrix[0][x] & WALL_S, f"Cell ({x},0) missing South wall"


def test_perimeter_north_row(parsed):
    """Every cell in the top row must have a North perimeter wall."""
    wall_matrix, width, height = parsed
    for x in range(width):
        assert wall_matrix[height - 1][x] & WALL_N, \
            f"Cell ({x},{height-1}) missing North wall"


def test_perimeter_west_col(parsed):
    """Every cell in the left column must have a West perimeter wall."""
    wall_matrix, _, height = parsed
    for y in range(height):
        assert wall_matrix[y][0] & WALL_W, f"Cell (0,{y}) missing West wall"


def test_perimeter_east_col(parsed):
    """Every cell in the right column must have an East perimeter wall."""
    wall_matrix, width, height = parsed
    for y in range(height):
        assert wall_matrix[y][width - 1] & WALL_E, \
            f"Cell ({width-1},{y}) missing East wall"


# ---------------------------------------------------------------------------
# Start cell (0, 0) — should have West and South perimeter walls at minimum
# ---------------------------------------------------------------------------

def test_start_cell_south_wall(parsed):
    wall_matrix, _, _ = parsed
    assert wall_matrix[0][0] & WALL_S, "Start cell (0,0) missing South wall"


def test_start_cell_west_wall(parsed):
    wall_matrix, _, _ = parsed
    assert wall_matrix[0][0] & WALL_W, "Start cell (0,0) missing West wall"


# ---------------------------------------------------------------------------
# Wall symmetry — shared walls must be consistent across neighbours
# ---------------------------------------------------------------------------

def test_wall_symmetry_north_south(parsed):
    """If cell (x, y) has a North wall, cell (x, y+1) must have a South wall."""
    wall_matrix, width, height = parsed
    for y in range(height - 1):
        for x in range(width):
            has_n = bool(wall_matrix[y][x] & WALL_N)
            has_s = bool(wall_matrix[y + 1][x] & WALL_S)
            assert has_n == has_s, (
                f"Asymmetric N/S wall between ({x},{y}) and ({x},{y+1})"
            )


def test_wall_symmetry_east_west(parsed):
    """If cell (x, y) has an East wall, cell (x+1, y) must have a West wall."""
    wall_matrix, width, height = parsed
    for y in range(height):
        for x in range(width - 1):
            has_e = bool(wall_matrix[y][x] & WALL_E)
            has_w = bool(wall_matrix[y][x + 1] & WALL_W)
            assert has_e == has_w, (
                f"Asymmetric E/W wall between ({x},{y}) and ({x+1},{y})"
            )


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

def test_file_not_found():
    with pytest.raises(FileNotFoundError):
        parse_maze("/nonexistent/path/maze.txt")
