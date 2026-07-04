"""
Shared constants for the Rob-26-MazeSolver project.

Defines cardinal directions, wall bitmask encoding (N=1, E=2, S=4, W=8),
movement deltas, and MMS display codes used across all modules.
"""
from __future__ import annotations

from enum import IntEnum


class Direction(IntEnum):
    """Cardinal directions for robot heading and wall orientation.

    Values encode clockwise order: (heading + 1) % 4 turns right,
    (heading - 1) % 4 turns left.
    """
    N = 0
    E = 1
    S = 2
    W = 3


# Wall bitmask constants — additive encoding (N=1, E=2, S=4, W=8)
WALL_N: int = 1
WALL_E: int = 2
WALL_S: int = 4
WALL_W: int = 8

# Direction → wall bitmask
DIR_TO_WALL: dict[Direction, int] = {
    Direction.N: WALL_N,
    Direction.E: WALL_E,
    Direction.S: WALL_S,
    Direction.W: WALL_W,
}

# Direction → (dx, dy) movement delta
# y increases upward — matches the MMS coordinate system (origin at bottom-left)
DIR_TO_DELTA: dict[Direction, tuple[int, int]] = {
    Direction.N: (0, 1),
    Direction.E: (1, 0),
    Direction.S: (0, -1),
    Direction.W: (-1, 0),
}

# Direction → opposite direction
OPPOSITE_DIR: dict[Direction, Direction] = {
    Direction.N: Direction.S,
    Direction.E: Direction.W,
    Direction.S: Direction.N,
    Direction.W: Direction.E,
}

# Direction → MMS wall string (used in setWall / clearWall API calls)
DIR_TO_STR: dict[Direction, str] = {
    Direction.N: 'n',
    Direction.E: 'e',
    Direction.S: 's',
    Direction.W: 'w',
}

# MMS cell color: character → color name (all 15 supported colors)
COLORS: dict[str, str] = {
    'k': 'Black',
    'b': 'Blue',
    'a': 'Gray',
    'c': 'Cyan',
    'g': 'Green',
    'o': 'Orange',
    'r': 'Red',
    'w': 'White',
    'y': 'Yellow',
    'B': 'Dark Blue',
    'C': 'Dark Cyan',
    'A': 'Dark Gray',
    'G': 'Dark Green',
    'R': 'Dark Red',
    'Y': 'Dark Yellow',
}
