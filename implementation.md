# Implementation Roadmap — Phase 2

## Current Status

Phase 1 of the [Implementation Roadmap](docs/implementation_roadmap.md) is complete. Deliverables:

- Python virtual environment set up with dependencies from `requirements.txt` (`numpy`, `matplotlib`, `pytest`)
- MMS simulator app installed and verified to launch correctly
- [`src/api/mms_api.py`](src/api/mms_api.py) — adapted (unmodified) from [`mackorone/mms-python`](https://github.com/mackorone/mms-python) (`API.py`); module-level functions communicating via `stdin`/`stdout`
- [`src/algorithms/base.py`](src/algorithms/base.py) — adapted from `mackorone/mms-python` (`Main.py`); wall-following test script extended to exercise additional `mms_api.py` methods; **not** the `BaseAlgorithm` abstract class (that is Phase 3d)
- [`mazes/maze_test.txt`](mazes/maze_test.txt) — single 16×16 ASCII-format competition maze (difficulty classification deferred to Phase 4)
- [`mazes/README.md`](mazes/README.md) — maze file format documentation and wall encoding dictionary (encoding updated in Phase 2, see below)
- [`src/api/README.md`](src/api/README.md) — full API contract documentation
- [`docs/mms.md`](docs/mms.md) — MMS simulator + venv integration notes
- `src/__init__.py` — package root with version string
- `src/api/__init__.py`, `src/algorithms/__init__.py` — empty package markers (updated in Phase 2)

## Current Repo Structure

```
Rob-26-MazeSolver/
├── requirements.txt
├── README.md
├── LICENSE
├── src/
│   ├── __init__.py
│   ├── api/
│   │   ├── __init__.py
│   │   ├── mms_api.py
│   │   └── README.md
│   └── algorithms/
│       ├── __init__.py
│       └── base.py            # test wall-follower; Phase 3d will replace with BaseAlgorithm
├── mazes/
│   ├── maze_test.txt
│   └── README.md
├── tests/
│   └── .gitkeep
└── docs/
    ├── Rob_26_proposal.md
    ├── Rob-26-MazeSolver_report.md
    ├── mms.md
    ├── notes.md
    ├── prompts.md
    ├── articles/
    └── res/
```

---

# Phase 2 — Core Infrastructure (with resolved decisions)

**Objective:** Build the shared data structures, common interface, and headless simulator that all algorithms depend on, enabling fully automated `pytest` runs without launching the MMS GUI.

## Design Decisions

### Wall bitmask encoding

**Decision: use the additive bitmask scheme N=1, E=2, S=4, W=8 (values 0–15).**

Rationale: standard in micromouse literature; bitmask composition and decomposition are direct (`walls & WALL_N` checks North, `walls | WALL_E` adds East). The previous encoding in `mazes/README.md` (E=bit0, N=bit1, W=bit2, S=bit3) was non-standard and inconsistent with the roadmap. `mazes/README.md` is updated in this phase.

### `mms_api.py` refactoring strategy

**Decision: keep all existing module-level functions unchanged; add a `MmsAPI(BaseAPI)` class at the bottom of the file whose methods delegate to the corresponding module-level functions.**

Rationale: preserves backward compatibility with `base.py` (test wall-follower) and the original `mackorone/mms-python` API.py design. Algorithm code uses the class-based `BaseAPI` interface; `base.py` continues to work with module-level calls.

### `algorithms/base.py` status

**Decision: leave `src/algorithms/base.py` unchanged in Phase 2.**

It is a test script used to verify the MMS simulator end-to-end. Phase 3d will define `BaseAlgorithm` in `src/algorithms/base_algorithm.py` (distinct filename to avoid collision). Phase 3a will implement the production `WallFollower` in `src/algorithms/wall_following.py`.

### `maze_parser.py` location and scope

**Decision: implement `src/parser/maze_parser.py` (ASCII `.txt` format only).**

The roadmap's `offline/maze_loader.py` (both ASCII and numerical formats) is deferred to Phase 4. The parser is placed under `src/parser/` because it is a shared core utility required by `SimAPI` in Phase 2. The numerical format parser (`.num`) will be added if needed in Phase 4.

### `Robot` class

**Decision: add `src/robot.py` — `Robot` class — to Phase 2 scope.**

The roadmap lists `Robot` in Phase 2 but it was missing from the initial Phase 2 description. It is required by `SimAPI` (heading tracking) and by all algorithm implementations.

### `MetricsLogger` export format

**Decision: use JSON as the primary export format (`export_json()`).**

JSON handles nested structures (wall matrix, visit matrix) natively. A flat `export_csv()` can be added later for tabular summaries used by Phase 6 analysis scripts.

---

## Files to Create / Edit

### New files

| File | Purpose |
|------|---------|
| `src/constants.py` | `Direction` enum, wall bitmask constants, lookup tables, color codes |
| `src/maze_map.py` | `MazeMap` class: 2-D wall + visit matrices |
| `src/robot.py` | `Robot` class: position and heading tracker |
| `src/api/base_api.py` | `BaseAPI` abstract class |
| `src/api/sim_api.py` | `SimAPI(BaseAPI)` headless simulator |
| `src/parser/__init__.py` | Package marker for `parser` subpackage |
| `src/parser/maze_parser.py` | ASCII maze file parser → wall matrix |
| `src/metrics/__init__.py` | Package marker for `metrics` subpackage |
| `src/metrics/logger.py` | `MetricsLogger` class |
| `tests/test_maze_map.py` | Unit tests for `MazeMap` |
| `tests/test_robot.py` | Unit tests for `Robot` |
| `tests/test_maze_parser.py` | Unit tests for `maze_parser` |
| `results/logs/.gitkeep` | Track empty `results/logs/` directory |

### Files to edit

| File | Change |
|------|--------|
| `src/api/mms_api.py` | Add `MmsAPI(BaseAPI)` class (keep module-level functions) |
| `src/api/__init__.py` | Add docstring; expose `BaseAPI`, `MmsAPI`, `SimAPI` |
| `src/algorithms/__init__.py` | Add docstring |
| `src/__init__.py` | Update description |
| `mazes/README.md` | Update encoding scheme to N=1, E=2, S=4, W=8; recompute full dictionary |
| `src/api/README.md` | Update status of `base_api.py` and `sim_api.py` to ✓ implemented |

## Target Repo Structure After Phase 2

```
Rob-26-MazeSolver/
├── requirements.txt
├── README.md
├── LICENSE
├── src/
│   ├── __init__.py            # updated
│   ├── constants.py           # NEW
│   ├── maze_map.py            # NEW
│   ├── robot.py               # NEW
│   ├── api/
│   │   ├── __init__.py        # updated
│   │   ├── base_api.py        # NEW
│   │   ├── mms_api.py         # updated (MmsAPI class added)
│   │   ├── sim_api.py         # NEW
│   │   └── README.md          # updated (status)
│   ├── algorithms/
│   │   ├── __init__.py        # updated
│   │   └── base.py            # unchanged (test wall-follower)
│   ├── parser/
│   │   ├── __init__.py        # NEW
│   │   └── maze_parser.py     # NEW
│   └── metrics/
│       ├── __init__.py        # NEW
│       └── logger.py          # NEW
├── mazes/
│   ├── maze_test.txt          # unchanged
│   └── README.md              # updated (encoding)
├── tests/
│   ├── test_maze_map.py       # NEW
│   ├── test_robot.py          # NEW
│   └── test_maze_parser.py    # NEW
├── results/
│   └── logs/
│       └── .gitkeep           # NEW
└── docs/
    └── ...
```

---

# Implementation-Ready Detailed Plan

## 1. `src/constants.py`

Define all project-wide constants. No external dependencies.

**`Direction` enum** (`IntEnum`): `N=0, E=1, S=2, W=3`
- Integer values encode clockwise rotation order; enables `(heading + 1) % 4` for right turns and `(heading - 1) % 4` for left turns.

**Wall bitmask constants** (module-level): `WALL_N = 1`, `WALL_E = 2`, `WALL_S = 4`, `WALL_W = 8`

**Lookup tables** (all keyed by `Direction`):

| Name | Type | Content |
|------|------|---------|
| `DIR_TO_WALL` | `dict[Direction, int]` | `{N:1, E:2, S:4, W:8}` |
| `DIR_TO_DELTA` | `dict[Direction, tuple[int,int]]` | `{N:(0,1), E:(1,0), S:(0,-1), W:(-1,0)}`; y increases upward (MMS origin) |
| `OPPOSITE_DIR` | `dict[Direction, Direction]` | `{N:S, E:W, S:N, W:E}` |
| `DIR_TO_STR` | `dict[Direction, str]` | `{N:'n', E:'e', S:'s', W:'w'}` (for `setWall`/`clearWall` API calls) |

**`COLORS: dict[str, str]`** — maps MMS color character to color name (all 15 entries from the MMS spec: `k,b,a,c,g,o,r,w,y,B,C,A,G,R,Y`).

---

## 2. `src/maze_map.py` — `MazeMap` class

Represents the robot's **known** view of the maze (during exploration) or the ground-truth maze (for `SimAPI`).

**Constructor**: `MazeMap(width: int, height: int)`

**Internal state**:
- `_walls: list[list[int]]` — `_walls[y][x]`; bitmask of walls at cell; 0 = fully unexplored; y=0 is bottom row (MMS origin at bottom-left)
- `_visits: list[list[int]]` — `_visits[y][x]`; visit count

**Public interface**:

| Method | Signature | Description |
|--------|-----------|-------------|
| `width` | `@property → int` | Number of columns |
| `height` | `@property → int` | Number of rows |
| `get_walls` | `(x, y) → int` | Raw bitmask for cell |
| `has_wall` | `(x, y, d: Direction) → bool` | Single-direction wall check |
| `set_wall` | `(x, y, d: Direction) → None` | Set wall on cell **and** symmetric neighbour wall |
| `clear_wall` | `(x, y, d: Direction) → None` | Clear wall on cell **and** neighbour (if in-bounds) |
| `mark_visit` | `(x, y) → None` | Increment visit counter |
| `get_visit_count` | `(x, y) → int` | Read visit counter |
| `export_walls` | `() → list[list[int]]` | Deep copy of `_walls` |
| `export_visits` | `() → list[list[int]]` | Deep copy of `_visits` |
| `distinct_cells_visited` | `() → int` | Count cells with visits > 0 |
| `total_visits` | `() → int` | Sum of all visit counts |

**Symmetry invariant**: `set_wall(x, y, N)` must also set the S-wall on neighbour `(x, y+1)`. Perimeter cells: silently skip out-of-bounds neighbour update.

**Private helper**: `_in_bounds(x, y) → bool`.

---

## 3. `src/robot.py` — `Robot` class

Tracks position and heading. Pure state; no I/O; no wall checking.

**Constructor**: `Robot(x: int = 0, y: int = 0, heading: Direction = Direction.N)`

**Public interface**:

| Method | Signature | Description |
|--------|-----------|-------------|
| `x`, `y`, `heading`, `position` | `@property` | Read current state; `position → (int, int)` |
| `turn_right` | `() → None` | `heading = (heading + 1) % 4` |
| `turn_left` | `() → None` | `heading = (heading - 1) % 4` |
| `move_forward` | `() → None` | Apply `DIR_TO_DELTA[heading]`; **no wall check** (caller's responsibility) |
| `wall_front_dir` | `() → Direction` | Absolute direction in front of robot |
| `wall_right_dir` | `() → Direction` | Absolute direction to the right |
| `wall_left_dir` | `() → Direction` | Absolute direction to the left |
| `wall_back_dir` | `() → Direction` | Absolute direction behind |
| `reset` | `(x=0, y=0, heading=N) → None` | Restore state to given values |

---

## 4. `src/api/base_api.py` — `BaseAPI` abstract class

`class BaseAPI(ABC)` — all methods `@abstractmethod`. No logic, no I/O. Sole purpose: enforce interface contract.

**Method groups**:

| Group | Methods |
|-------|---------|
| Maze info | `maze_width() → int`, `maze_height() → int` |
| Wall sensing | `wall_front() → bool`, `wall_back() → bool`, `wall_left() → bool`, `wall_right() → bool` |
| Movement | `move_forward() → None` (raises `MouseCrashedError` when blocked), `turn_right() → None`, `turn_left() → None` |
| Display | `set_wall(x,y,d:str)`, `clear_wall(x,y,d:str)`, `set_color(x,y,c:str)`, `clear_color(x,y)`, `clear_all_color()`, `set_text(x,y,text:str)`, `clear_text(x,y)`, `clear_all_text()` |
| Control | `was_reset() → bool`, `ack_reset() → None` |
| Stats | `get_stat(stat: str) → int \| float` |

Imports: `abc.ABC`, `abc.abstractmethod` only.

---

## 5. `src/api/mms_api.py` — add `MmsAPI(BaseAPI)` class

**Keep all existing module-level functions unchanged.**

Append at the bottom of the file (after existing code):

```python
from src.api.base_api import BaseAPI

class MmsAPI(BaseAPI):
    """BaseAPI implementation for the MMS GUI simulator.
    Delegates to module-level functions; all I/O via stdin/stdout."""

    def maze_width(self) -> int:        return mazeWidth()
    def maze_height(self) -> int:       return mazeHeight()
    def wall_front(self) -> bool:       return wallFront()
    def wall_back(self) -> bool:        return wallBack()
    def wall_left(self) -> bool:        return wallLeft()
    def wall_right(self) -> bool:       return wallRight()
    def move_forward(self) -> None:     moveForward()   # raises MouseCrashedError on crash
    def turn_right(self) -> None:       turnRight()
    def turn_left(self) -> None:        turnLeft()
    def set_wall(self, x, y, d):        setWall(x, y, d)
    def clear_wall(self, x, y, d):      clearWall(x, y, d)
    def set_color(self, x, y, c):       setColor(x, y, c)
    def clear_color(self, x, y):        clearColor(x, y)
    def clear_all_color(self):          clearAllColor()
    def set_text(self, x, y, text):     setText(x, y, text)
    def clear_text(self, x, y):         clearText(x, y)
    def clear_all_text(self):           clearAllText()
    def was_reset(self) -> bool:        return wasReset()
    def ack_reset(self) -> None:        ackReset()
    def get_stat(self, stat: str):      return command(["getStat", stat], return_type=str)
```

---

## 6. `src/api/sim_api.py` — `SimAPI(BaseAPI)` class

**Constructor**: `SimAPI(wall_matrix: list[list[int]], width: int, height: int, start_x: int = 0, start_y: int = 0, start_heading: Direction = Direction.N)`

- `wall_matrix[y][x]`: ground-truth bitmask from `maze_parser` (N=1, E=2, S=4, W=8)
- Stores `_start_{x,y,heading}` for use in `ack_reset()`
- `_reset_flag: bool = False`

**Wall sensing**: resolve relative direction to absolute direction, then look up `_walls[_y][_x] & DIR_TO_WALL[abs_dir]`:
- `wall_front()` → checks `_heading`
- `wall_back()` → checks `Direction((_heading + 2) % 4)`
- `wall_left()` → checks `Direction((_heading - 1) % 4)`
- `wall_right()` → checks `Direction((_heading + 1) % 4)`

**Movement**:
- `move_forward()`: if `wall_front()` → raise `MouseCrashedError("crashed")`; else apply `DIR_TO_DELTA[_heading]` to `(_x, _y)`
- `turn_right()` / `turn_left()`: update `_heading`

**Display methods**: all no-ops (`pass`)

**Control**:
- `was_reset() → bool`: returns `_reset_flag`
- `ack_reset()`: restores `(_x, _y, _heading)` to stored start values; sets `_reset_flag = False`

**Stats**: `get_stat()` always returns -1

**Additional read-only properties**: `position → tuple[int,int]`, `heading → Direction`

---

## 7. `src/parser/maze_parser.py`

**Public function**: `parse_maze(filepath: str) -> tuple[list[list[int]], int, int]`

Returns `(wall_matrix, width, height)` where `wall_matrix[y][x]` uses N=1, E=2, S=4, W=8 and y=0 is the bottom row.

**ASCII format geometry** (for a W-column × H-row maze):
- File lines: `2H + 1`
- Chars per line: `4W + 1`
- Example: 16×16 → 33 lines × 65 chars

**Cell (x, y) → file coordinates** (y=0 = bottom-left, MMS origin):

| Variable | Formula |
|----------|---------|
| `r_top` (top edge row) | `2 * (H - 1 - y)` |
| `r_mid` (interior row) | `2 * (H - 1 - y) + 1` |
| `c` (left column) | `4 * x` |

**Wall detection** — any non-space character at the probe position = wall present:

| Direction | Bitmask | Probe position |
|-----------|---------|----------------|
| North | 1 | `lines[r_top][c + 2]` |
| East  | 2 | `lines[r_mid][c + 4]` |
| South | 4 | `lines[r_top + 2][c + 2]` |
| West  | 8 | `lines[r_mid][c]` |

**Implementation steps**:
1. Open file; read all lines; strip trailing `\n` only (preserve interior spaces)
2. Infer dimensions: `H = (len(lines) - 1) // 2`, `W = (len(lines[0]) - 1) // 4`
3. Validate: `len(lines) == 2*H + 1`; raise `ValueError("malformed maze file: ...")` on failure
4. Build `wall_matrix` as `[[0]*W for _ in range(H)]`
5. Iterate `y` from 0 to H−1, `x` from 0 to W−1; compute `r_top, r_mid, c`; accumulate bitmask

---

## 8. `src/metrics/logger.py` — `MetricsLogger` class

**Constructor**: `MetricsLogger(algorithm_name: str, maze_name: str)`

**State**:
- `_algo: str`, `_maze: str`
- `_forward_moves: int = 0`, `_turns: int = 0`
- `_start_time: float | None = None`, `_end_time: float | None = None`
- `_wall_matrix: list[list[int]] | None = None`
- `_visit_matrix: list[list[int]] | None = None`

**Methods**:

| Method | Description |
|--------|-------------|
| `start() → None` | Record `_start_time = time.monotonic()` |
| `log_move() → None` | `_forward_moves += 1` |
| `log_turn() → None` | `_turns += 1` |
| `stop() → None` | Record `_end_time = time.monotonic()` |
| `set_matrices(walls, visits) → None` | Snapshot from `MazeMap.export_walls()` / `export_visits()` |

**Computed metrics** (read-only properties):

| Property | Returns | Description |
|----------|---------|-------------|
| `forward_moves` | `int` | Raw forward moves |
| `turns` | `int` | Raw turn count |
| `total_moves` | `int` | `_forward_moves + _turns` |
| `distinct_cells_visited` | `int` | Cells with visit count > 0 in `_visit_matrix` |
| `total_visits` | `int` | Sum of all values in `_visit_matrix` |
| `execution_time` | `float \| None` | `_end_time - _start_time`; `None` if not yet stopped |

**`export_json(output_dir: str = "results/logs/") → str`**:
- Creates `output_dir` with `os.makedirs(exist_ok=True)`
- Filename: `{algo}_{maze}_{timestamp}.json` where timestamp = `datetime.now().strftime("%Y%m%d_%H%M%S")`
- JSON payload keys: `algorithm`, `maze`, `timestamp`, `total_moves`, `forward_moves`, `turns`, `distinct_cells_visited`, `total_visits`, `execution_time_s`, `wall_matrix`, `visit_matrix`
- Returns full file path as string

---

## 9. `__init__.py` updates

**`src/__init__.py`**: update docstring to describe the full package.

**`src/api/__init__.py`**:
```python
"""API layer: BaseAPI contract, MmsAPI (MMS GUI simulator), SimAPI (headless)."""
from src.api.base_api import BaseAPI
from src.api.mms_api import MmsAPI, MouseCrashedError
from src.api.sim_api import SimAPI
```

**`src/algorithms/__init__.py`**:
```python
"""Maze-solving algorithm implementations."""
```

**`src/parser/__init__.py`** (new file):
```python
"""Maze file parsers. parse_maze() reads ASCII .txt format and returns a wall matrix."""
from src.parser.maze_parser import parse_maze
```

**`src/metrics/__init__.py`** (new file):
```python
"""Metrics collection and export. MetricsLogger records per-run performance statistics."""
from src.metrics.logger import MetricsLogger
```

---

## 10. `mazes/README.md` encoding update

Update **Encoding Scheme** section bits assignment and full 16-entry dictionary table to use **N=1, E=2, S=4, W=8**:

| Bit | Direction | Value |
|-----|-----------|-------|
| bit 0 | North | 1 |
| bit 1 | East  | 2 |
| bit 2 | South | 4 |
| bit 3 | West  | 8 |

Recomputed full dictionary (0–15), ordered by integer value:

| Value | N | E | S | W | Description |
|-------|---|---|---|---|-------------|
| 0  | 0 | 0 | 0 | 0 | No walls |
| 1  | 1 | 0 | 0 | 0 | North only |
| 2  | 0 | 1 | 0 | 0 | East only |
| 3  | 1 | 1 | 0 | 0 | North + East |
| 4  | 0 | 0 | 1 | 0 | South only |
| 5  | 1 | 0 | 1 | 0 | North + South |
| 6  | 0 | 1 | 1 | 0 | East + South |
| 7  | 1 | 1 | 1 | 0 | North + East + South |
| 8  | 0 | 0 | 0 | 1 | West only |
| 9  | 1 | 0 | 0 | 1 | North + West |
| 10 | 0 | 1 | 0 | 1 | East + West |
| 11 | 1 | 1 | 0 | 1 | North + East + West |
| 12 | 0 | 0 | 1 | 1 | South + West |
| 13 | 1 | 0 | 1 | 1 | North + South + West |
| 14 | 0 | 1 | 1 | 1 | East + South + West |
| 15 | 1 | 1 | 1 | 1 | All walls |

---

## 11. Unit tests

### `tests/test_maze_map.py`

| Test | Assertion |
|------|-----------|
| `test_init_all_zero` | All walls and visits are 0 at construction |
| `test_set_wall_single` | `set_wall(1,1,N)` → `has_wall(1,1,N)` is True |
| `test_set_wall_symmetry` | `set_wall(1,1,N)` → `has_wall(1,2,S)` is True |
| `test_set_wall_perimeter_no_error` | `set_wall(0, H-1, N)` (top row) raises no exception |
| `test_clear_wall_and_symmetry` | Set then clear: `has_wall(1,1,N)` and `has_wall(1,2,S)` both False |
| `test_mark_visit_counts` | Mark `(0,0)` twice: `get_visit_count(0,0) == 2` |
| `test_export_deep_copy` | Mutating exported list does not change internal `_walls` |
| `test_distinct_cells_visited` | Mark 3 distinct cells → returns 3 |
| `test_total_visits` | Mark `(0,0)` twice, `(1,0)` once → returns 3 |

### `tests/test_robot.py`

| Test | Assertion |
|------|-----------|
| `test_initial_state` | `x=0, y=0, heading=N` |
| `test_turn_right_cycle` | 4 right turns → heading back to N |
| `test_turn_left_cycle` | 4 left turns → heading back to N |
| `test_move_forward_north` | heading=N, move → y=1 |
| `test_move_forward_east` | heading=E, move → x=1 |
| `test_move_forward_south` | heading=S from (0,1), move → y=0 |
| `test_move_forward_west` | heading=W from (1,0), move → x=0 |
| `test_wall_dir_methods` | heading=E: front=E, right=S, left=N, back=W |
| `test_reset` | Restores to (0, 0, N) after moves and turns |

### `tests/test_maze_parser.py`

| Test | Assertion |
|------|-----------|
| `test_parse_maze_test_dimensions` | Parse `mazes/maze_test.txt` → width=16, height=16 |
| `test_start_cell_south_wall` | `(0,0)` has S wall (bottom perimeter) |
| `test_start_cell_west_wall` | `(0,0)` has W wall (left perimeter); matches "S" start label |
| `test_top_right_perimeter` | `(15,15)` has N wall and E wall (top-right corner) |
| `test_perimeter_south_row` | All cells `(x, 0)` for x in 0..15 have S wall |
| `test_known_interior_cell` | Manually verified cell bitmask matches visual inspection of maze_test.txt |

---

> **Milestone M2 — Infrastructure ready:** `python -m pytest tests/` passes all unit tests. A toy script can instantiate `MazeMap` + `Robot`, call `SimAPI` with a parsed maze, and produce a valid JSON log in `results/logs/` — without MMS running.
