"""
SimAPI: headless BaseAPI implementation backed by a loaded maze file.

Answers sensor queries by consulting the ground-truth wall matrix, tracks
robot position and heading internally, and raises MouseCrashedError on
invalid moves. Enables fully automated pytest runs without the MMS GUI.
"""
from __future__ import annotations

from src.api.base_api import BaseAPI
from src.api.mms_api import MouseCrashedError
from src.constants import Direction, DIR_TO_DELTA, DIR_TO_WALL


class SimAPI(BaseAPI):
    """Headless simulator that implements BaseAPI against a preloaded maze.

    Args:
        wall_matrix:    Ground-truth wall bitmasks as wall_matrix[y][x],
                        using N=1, E=2, S=4, W=8 encoding.
        width:          Number of columns in the maze.
        height:         Number of rows in the maze.
        start_x:        Starting column (default 0).
        start_y:        Starting row (default 0).
        start_heading:  Starting direction (default Direction.N).
    """

    def __init__(
        self,
        wall_matrix: list[list[int]],
        width: int,
        height: int,
        start_x: int = 0,
        start_y: int = 0,
        start_heading: Direction = Direction.N,
    ) -> None:
        self._walls = wall_matrix
        self._width = width
        self._height = height
        self._start_x = start_x
        self._start_y = start_y
        self._start_heading = start_heading
        self._x = start_x
        self._y = start_y
        self._heading = start_heading
        self._reset_flag = False

    # ------------------------------------------------------------------
    # Maze information
    # ------------------------------------------------------------------

    def maze_width(self) -> int:
        return self._width

    def maze_height(self) -> int:
        return self._height

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _abs_dir(self, offset: int) -> Direction:
        """Return the Direction that is *offset* clockwise steps from heading."""
        return Direction((self._heading + offset) % 4)

    def _has_wall(self, direction: Direction) -> bool:
        return bool(self._walls[self._y][self._x] & DIR_TO_WALL[direction])

    # ------------------------------------------------------------------
    # Wall sensing
    # ------------------------------------------------------------------

    def wall_front(self) -> bool:
        return self._has_wall(self._heading)

    def wall_back(self) -> bool:
        return self._has_wall(self._abs_dir(2))

    def wall_left(self) -> bool:
        # (heading - 1) % 4  ==  (heading + 3) % 4  in Python modular arithmetic
        return self._has_wall(self._abs_dir(3))

    def wall_right(self) -> bool:
        return self._has_wall(self._abs_dir(1))

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def move_forward(self) -> None:
        if self.wall_front():
            raise MouseCrashedError("SimAPI: robot crashed into a wall.")
        dx, dy = DIR_TO_DELTA[self._heading]
        self._x += dx
        self._y += dy

    def turn_right(self) -> None:
        self._heading = Direction((self._heading + 1) % 4)

    def turn_left(self) -> None:
        self._heading = Direction((self._heading - 1) % 4)

    # ------------------------------------------------------------------
    # Display — all no-ops in headless mode
    # ------------------------------------------------------------------

    def set_wall(self, x: int, y: int, direction: str) -> None:
        pass

    def clear_wall(self, x: int, y: int, direction: str) -> None:
        pass

    def set_color(self, x: int, y: int, color: str) -> None:
        pass

    def clear_color(self, x: int, y: int) -> None:
        pass

    def clear_all_color(self) -> None:
        pass

    def set_text(self, x: int, y: int, text: str) -> None:
        pass

    def clear_text(self, x: int, y: int) -> None:
        pass

    def clear_all_text(self) -> None:
        pass

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    def was_reset(self) -> bool:
        return self._reset_flag

    def ack_reset(self) -> None:
        self._x = self._start_x
        self._y = self._start_y
        self._heading = self._start_heading
        self._reset_flag = False

    # ------------------------------------------------------------------
    # Statistics — not tracked in headless mode
    # ------------------------------------------------------------------

    def get_stat(self, stat: str) -> int:
        return -1

    # ------------------------------------------------------------------
    # Additional read-only properties (for tests and algorithm access)
    # ------------------------------------------------------------------

    @property
    def position(self) -> tuple[int, int]:
        """Current robot position as (x, y)."""
        return (self._x, self._y)

    @property
    def heading(self) -> Direction:
        """Current robot heading."""
        return self._heading
