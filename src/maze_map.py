"""
MazeMap: 2-D wall matrix and visit-count matrix for maze representation.

Used both as the robot's incrementally-built known map during exploration
and as the ground-truth maze container loaded by SimAPI.

Coordinate convention: (x, y) where x is column (0 = left) and y is row
(0 = bottom), matching the MMS simulator origin (bottom-left).
Internal storage: _walls[y][x], _visits[y][x].
"""
from __future__ import annotations

from src.constants import Direction, DIR_TO_DELTA, DIR_TO_WALL, OPPOSITE_DIR


class MazeMap:
    """Represents a maze as two 2-D integer matrices.

    _walls[y][x]  — bitmask of walls at cell (x, y); 0 = fully unexplored
    _visits[y][x] — number of times cell (x, y) has been visited

    Wall bitmask encoding: N=1, E=2, S=4, W=8 (additive).
    """

    def __init__(self, width: int, height: int) -> None:
        self._width = width
        self._height = height
        self._walls: list[list[int]] = [[0] * width for _ in range(height)]
        self._visits: list[list[int]] = [[0] * width for _ in range(height)]

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def width(self) -> int:
        """Number of columns."""
        return self._width

    @property
    def height(self) -> int:
        """Number of rows."""
        return self._height

    # ------------------------------------------------------------------
    # Wall queries and mutations
    # ------------------------------------------------------------------

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self._width and 0 <= y < self._height

    def get_walls(self, x: int, y: int) -> int:
        """Return the raw wall bitmask for cell (x, y)."""
        return self._walls[y][x]

    def has_wall(self, x: int, y: int, direction: Direction) -> bool:
        """Return True if cell (x, y) has a wall in *direction*."""
        return bool(self._walls[y][x] & DIR_TO_WALL[direction])

    def set_wall(self, x: int, y: int, direction: Direction) -> None:
        """Set a wall at cell (x, y) in *direction*.

        Also sets the symmetric wall on the neighbouring cell (if in-bounds).
        """
        self._walls[y][x] |= DIR_TO_WALL[direction]
        dx, dy = DIR_TO_DELTA[direction]
        nx, ny = x + dx, y + dy
        if self._in_bounds(nx, ny):
            self._walls[ny][nx] |= DIR_TO_WALL[OPPOSITE_DIR[direction]]

    def clear_wall(self, x: int, y: int, direction: Direction) -> None:
        """Clear the wall at cell (x, y) in *direction*.

        Also clears the symmetric wall on the neighbouring cell (if in-bounds).
        """
        self._walls[y][x] &= ~DIR_TO_WALL[direction]
        dx, dy = DIR_TO_DELTA[direction]
        nx, ny = x + dx, y + dy
        if self._in_bounds(nx, ny):
            self._walls[ny][nx] &= ~DIR_TO_WALL[OPPOSITE_DIR[direction]]

    # ------------------------------------------------------------------
    # Visit tracking
    # ------------------------------------------------------------------

    def mark_visit(self, x: int, y: int) -> None:
        """Increment the visit counter for cell (x, y)."""
        self._visits[y][x] += 1

    def get_visit_count(self, x: int, y: int) -> int:
        """Return the number of times cell (x, y) has been visited."""
        return self._visits[y][x]

    # ------------------------------------------------------------------
    # Exports (deep copies to prevent external mutation of internal state)
    # ------------------------------------------------------------------

    def export_walls(self) -> list[list[int]]:
        """Return a deep copy of the wall matrix (row-major, y=0 is bottom)."""
        return [row[:] for row in self._walls]

    def export_visits(self) -> list[list[int]]:
        """Return a deep copy of the visit-count matrix."""
        return [row[:] for row in self._visits]

    def distinct_cells_visited(self) -> int:
        """Return the number of distinct cells visited at least once."""
        return sum(
            1
            for y in range(self._height)
            for x in range(self._width)
            if self._visits[y][x] > 0
        )

    def total_visits(self) -> int:
        """Return the total number of cell visits (including revisits)."""
        return sum(
            self._visits[y][x]
            for y in range(self._height)
            for x in range(self._width)
        )
