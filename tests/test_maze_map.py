"""Unit tests for MazeMap."""
import pytest
from src.constants import Direction
from src.maze_map import MazeMap


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_map(w: int = 4, h: int = 4) -> MazeMap:
    return MazeMap(w, h)


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def test_init_all_zero():
    m = make_map()
    for y in range(4):
        for x in range(4):
            assert m.get_walls(x, y) == 0
            assert m.get_visit_count(x, y) == 0


def test_dimensions():
    m = MazeMap(6, 3)
    assert m.width == 6
    assert m.height == 3


# ---------------------------------------------------------------------------
# Wall set / clear
# ---------------------------------------------------------------------------

def test_set_wall_single():
    m = make_map()
    m.set_wall(1, 1, Direction.N)
    assert m.has_wall(1, 1, Direction.N)


def test_set_wall_does_not_pollute_others():
    m = make_map()
    m.set_wall(1, 1, Direction.N)
    assert not m.has_wall(1, 1, Direction.E)
    assert not m.has_wall(1, 1, Direction.S)
    assert not m.has_wall(1, 1, Direction.W)


def test_set_wall_symmetry_north():
    """set_wall(x, y, N) must set the S-wall of the cell above."""
    m = make_map()
    m.set_wall(1, 1, Direction.N)
    assert m.has_wall(1, 2, Direction.S)


def test_set_wall_symmetry_east():
    m = make_map()
    m.set_wall(1, 1, Direction.E)
    assert m.has_wall(2, 1, Direction.W)


def test_set_wall_symmetry_south():
    m = make_map()
    m.set_wall(1, 1, Direction.S)
    assert m.has_wall(1, 0, Direction.N)


def test_set_wall_symmetry_west():
    m = make_map()
    m.set_wall(1, 1, Direction.W)
    assert m.has_wall(0, 1, Direction.E)


def test_set_wall_perimeter_no_error():
    """Setting a wall on the perimeter must not raise an IndexError."""
    m = make_map(4, 4)
    m.set_wall(0, 3, Direction.N)   # top row, north → neighbour out of bounds
    assert m.has_wall(0, 3, Direction.N)

    m.set_wall(3, 0, Direction.E)   # right col, east → neighbour out of bounds
    assert m.has_wall(3, 0, Direction.E)


def test_clear_wall_and_symmetry():
    m = make_map()
    m.set_wall(1, 1, Direction.N)
    m.clear_wall(1, 1, Direction.N)
    assert not m.has_wall(1, 1, Direction.N)
    assert not m.has_wall(1, 2, Direction.S)


def test_set_multiple_walls_bitmask():
    m = make_map()
    m.set_wall(0, 0, Direction.N)
    m.set_wall(0, 0, Direction.E)
    # WALL_N=1, WALL_E=2 → bitmask should be 3
    assert m.get_walls(0, 0) == 3


# ---------------------------------------------------------------------------
# Visit tracking
# ---------------------------------------------------------------------------

def test_mark_visit_single():
    m = make_map()
    m.mark_visit(0, 0)
    assert m.get_visit_count(0, 0) == 1


def test_mark_visit_counts():
    m = make_map()
    m.mark_visit(0, 0)
    m.mark_visit(0, 0)
    assert m.get_visit_count(0, 0) == 2


def test_distinct_cells_visited():
    m = make_map()
    m.mark_visit(0, 0)
    m.mark_visit(1, 0)
    m.mark_visit(2, 1)
    assert m.distinct_cells_visited() == 3


def test_distinct_cells_unvisited_map():
    m = make_map()
    assert m.distinct_cells_visited() == 0


def test_total_visits():
    m = make_map()
    m.mark_visit(0, 0)
    m.mark_visit(0, 0)
    m.mark_visit(1, 0)
    assert m.total_visits() == 3


def test_total_visits_empty():
    m = make_map()
    assert m.total_visits() == 0


# ---------------------------------------------------------------------------
# Export (deep copy)
# ---------------------------------------------------------------------------

def test_export_walls_deep_copy():
    m = make_map()
    m.set_wall(0, 0, Direction.E)
    exported = m.export_walls()
    original_value = exported[0][0]
    exported[0][0] = 0          # mutate the exported list
    assert m.get_walls(0, 0) == original_value  # internal state unchanged


def test_export_visits_deep_copy():
    m = make_map()
    m.mark_visit(0, 0)
    exported = m.export_visits()
    exported[0][0] = 999
    assert m.get_visit_count(0, 0) == 1
