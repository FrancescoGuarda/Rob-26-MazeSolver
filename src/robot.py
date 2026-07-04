"""
Robot: position and heading tracker for maze navigation.

Pure state object — no I/O, no wall checking.
The caller is responsible for verifying a move is legal before calling
move_forward().
"""
from __future__ import annotations

from src.constants import Direction, DIR_TO_DELTA


class Robot:
    """Tracks the robot's (x, y) position and heading Direction.

    Coordinate convention: x = column (0 = left), y = row (0 = bottom),
    matching the MMS simulator origin.

    Heading uses Direction (N=0, E=1, S=2, W=3) in clockwise order:
        turn_right: heading = (heading + 1) % 4
        turn_left:  heading = (heading - 1) % 4
    """

    def __init__(
        self,
        x: int = 0,
        y: int = 0,
        heading: Direction = Direction.N,
    ) -> None:
        self._x = x
        self._y = y
        self._heading = heading

    # ------------------------------------------------------------------
    # State properties
    # ------------------------------------------------------------------

    @property
    def x(self) -> int:
        return self._x

    @property
    def y(self) -> int:
        return self._y

    @property
    def heading(self) -> Direction:
        return self._heading

    @property
    def position(self) -> tuple[int, int]:
        """Current position as (x, y)."""
        return (self._x, self._y)

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def turn_right(self) -> None:
        """Rotate 90° clockwise."""
        self._heading = Direction((self._heading + 1) % 4)

    def turn_left(self) -> None:
        """Rotate 90° counter-clockwise."""
        self._heading = Direction((self._heading - 1) % 4)

    def move_forward(self) -> None:
        """Advance one cell in the current heading direction.

        Does NOT check for walls — caller must verify the move is legal.
        """
        dx, dy = DIR_TO_DELTA[self._heading]
        self._x += dx
        self._y += dy

    # ------------------------------------------------------------------
    # Absolute wall directions relative to current heading
    # ------------------------------------------------------------------

    def wall_front_dir(self) -> Direction:
        """Absolute direction in front of the robot."""
        return self._heading

    def wall_right_dir(self) -> Direction:
        """Absolute direction to the robot's right."""
        return Direction((self._heading + 1) % 4)

    def wall_left_dir(self) -> Direction:
        """Absolute direction to the robot's left."""
        return Direction((self._heading - 1) % 4)

    def wall_back_dir(self) -> Direction:
        """Absolute direction behind the robot."""
        return Direction((self._heading + 2) % 4)

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------

    def reset(
        self,
        x: int = 0,
        y: int = 0,
        heading: Direction = Direction.N,
    ) -> None:
        """Restore robot state to the given position and heading."""
        self._x = x
        self._y = y
        self._heading = heading
