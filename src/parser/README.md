# `src/parser` — Maze File Parsers

Parses maze files into a uniform internal representation (wall matrix) consumed by [`SimAPI`](../api/README.md#sim_apipy--headless-simulator) and [`MazeMap`](../README.md#maze_mappy).

## Module

| Module | Status | Description |
|--------|--------|-------------|
| [`maze_parser.py`](#maze_parserpy) | ✓ implemented | ASCII `.txt` map format → wall matrix |

**Numerical `.num` format** (coordinate + N E S W flags) is deferred to Phase 4 if needed.

---

## `maze_parser.py`

### `parse_maze(filepath)`

```python
parse_maze(filepath: str) -> tuple[list[list[int]], int, int]
```

Reads an ASCII `.txt` maze file and returns `(wall_matrix, width, height)`.

- `wall_matrix[y][x]` — integer bitmask using **N=1, E=2, S=4, W=8** encoding
- `y=0` is the **bottom row** (MMS coordinate origin at bottom-left)
- `width` — number of columns; `height` — number of rows

**Raises:**
- `FileNotFoundError` — if the file does not exist
- `ValueError` — if the file does not match the expected ASCII format

### ASCII format geometry

For a maze of W columns × H rows:

| Property | Value |
|----------|-------|
| Total lines | `2H + 1` |
| Chars per line | `4W + 1` |
| Example (16×16) | 33 lines × 65 chars |

Cell `(x, y)` maps to file positions:

| Variable | Formula |
|----------|---------|
| `r_top` — top edge row | `2 * (H - 1 - y)` |
| `r_mid` — interior row | `r_top + 1` |
| `c` — left column | `4 * x` |

Wall detection — any **non-space** character at the probe position = wall present:

| Direction | Bitmask | Probe position |
|-----------|---------|----------------|
| North | 1 | `lines[r_top][c + 2]` |
| East  | 2 | `lines[r_mid][c + 4]` |
| South | 4 | `lines[r_top + 2][c + 2]` |
| West  | 8 | `lines[r_mid][c]` |

### Usage

```python
from src.parser import parse_maze

wall_matrix, width, height = parse_maze("mazes/maze_test.txt")
# wall_matrix[0][0] → bitmask for bottom-left cell
```

### Wall encoding reference

See [`mazes/README.md`](../../mazes/README.md) for the full 16-entry wall configuration dictionary.
