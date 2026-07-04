"""
maze_parser: ASCII .txt maze file parser.

Reads the standard MMS ASCII map format and returns a wall matrix using
the N=1, E=2, S=4, W=8 bitmask encoding.

ASCII format geometry for a W-column × H-row maze:
  - File lines:     2H + 1
  - Chars per line: 4W + 1
  - Cell (x, y) with y=0 at bottom-left (MMS origin):
      top edge row:  r_top = 2 * (H - 1 - y)
      interior row:  r_mid = r_top + 1
      left column:   c     = 4 * x

Wall detection: any non-space character at the probe position = wall present.
  North → lines[r_top    ][c + 2]
  East  → lines[r_mid    ][c + 4]
  South → lines[r_top + 2][c + 2]
  West  → lines[r_mid    ][c    ]
"""
from __future__ import annotations

from src.constants import WALL_E, WALL_N, WALL_S, WALL_W


def parse_maze(filepath: str) -> tuple[list[list[int]], int, int]:
    """Parse an ASCII .txt maze file and return a wall matrix.

    Args:
        filepath: Path to the ASCII maze file (.txt, MMS map format).

    Returns:
        A tuple ``(wall_matrix, width, height)`` where:
          - ``wall_matrix[y][x]`` is an integer bitmask (N=1, E=2, S=4, W=8),
            with y=0 being the bottom row (MMS origin at bottom-left).
          - ``width``  is the number of columns.
          - ``height`` is the number of rows.

    Raises:
        FileNotFoundError: If *filepath* does not exist.
        ValueError: If the file does not match the expected ASCII format.
    """
    with open(filepath, 'r') as fh:
        # Strip only trailing newline/carriage-return; preserve interior spaces.
        lines = [line.rstrip('\n\r') for line in fh.readlines()]

    # Remove fully-empty trailing lines (some editors append a blank line).
    while lines and lines[-1] == '':
        lines.pop()

    if not lines:
        raise ValueError(f"Maze file is empty: {filepath}")

    n_lines = len(lines)
    n_cols = len(lines[0])

    if (n_lines - 1) % 2 != 0:
        raise ValueError(
            f"Malformed maze file: expected an odd number of lines (2H+1), "
            f"got {n_lines} in '{filepath}'"
        )
    H = (n_lines - 1) // 2

    if (n_cols - 1) % 4 != 0:
        raise ValueError(
            f"Malformed maze file: expected line length 4W+1, "
            f"got {n_cols} in '{filepath}'"
        )
    W = (n_cols - 1) // 4

    # Pad any short lines to avoid index errors (some editors strip trailing spaces).
    lines = [line.ljust(n_cols) for line in lines]

    wall_matrix: list[list[int]] = [[0] * W for _ in range(H)]

    for y in range(H):
        r_top = 2 * (H - 1 - y)
        r_mid = r_top + 1
        for x in range(W):
            c = 4 * x
            mask = 0
            if lines[r_top    ][c + 2] != ' ':
                mask |= WALL_N
            if lines[r_mid    ][c + 4] != ' ':
                mask |= WALL_E
            if lines[r_top + 2][c + 2] != ' ':
                mask |= WALL_S
            if lines[r_mid    ][c    ] != ' ':
                mask |= WALL_W
            wall_matrix[y][x] = mask

    return wall_matrix, W, H
