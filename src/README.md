# `src` — Source Package

This package contains all project source code. The top-level modules define shared data structures and constants used across every algorithm and backend. Subpackage READMEs cover their own contents in detail.

## Top-level modules

| Module | Description |
|--------|-------------|
| [`constants.py`](#constantspy) | Shared enums, wall bitmask constants, and lookup tables |
| [`maze_map.py`](#maze_mappy) | `MazeMap` — 2-D wall matrix and visit-count matrix |
| [`robot.py`](#robotpy) | `Robot` — position and heading tracker |

## Subpackages

| Package | README | Description |
|---------|--------|-------------|
| `api/` | [src/api/README.md](api/README.md) | `BaseAPI` contract, `MmsAPI` (GUI), `SimAPI` (headless) |
| `parser/` | [src/parser/README.md](parser/README.md) | ASCII maze file parser |
| `metrics/` | [src/metrics/README.md](metrics/README.md) | Per-run metrics collection and JSON export |
| `algorithms/` | *(Phase 3)* | Wall-following, flood fill, A\* implementations |

---

## `constants.py`

Defines all project-wide constants. No external dependencies.

### `Direction` enum (`IntEnum`)

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `N` | North |
| 1 | `E` | East |
| 2 | `S` | South |
| 3 | `W` | West |

Values encode clockwise rotation order: `(heading + 1) % 4` turns right, `(heading - 1) % 4` turns left.

### Wall bitmask constants

| Constant | Value | Bit |
|----------|-------|-----|
| `WALL_N` | 1 | bit 0 |
| `WALL_E` | 2 | bit 1 |
| `WALL_S` | 4 | bit 2 |
| `WALL_W` | 8 | bit 3 |

A cell's encoding is the **sum** of its wall values (e.g. North + West = 1 + 8 = 9).

### Lookup tables

| Name | Key → Value | Usage |
|------|-------------|-------|
| `DIR_TO_WALL` | `Direction → int` | Get bitmask for a direction |
| `DIR_TO_DELTA` | `Direction → (dx, dy)` | Movement delta; y increases upward |
| `OPPOSITE_DIR` | `Direction → Direction` | N↔S, E↔W |
| `DIR_TO_STR` | `Direction → str` | MMS API strings `'n'`, `'e'`, `'s'`, `'w'` |
| `COLORS` | `str → str` | MMS color character → color name (15 entries) |

---

## `maze_map.py`

`MazeMap(width, height)` — represents a maze as two 2-D integer matrices stored row-major as `matrix[y][x]` with `y=0` at the bottom-left (MMS origin).

Used in two roles:
- **Robot's known map** — built incrementally during exploration; cells start at 0 (unexplored) and walls are added as sensors fire.
- **Ground-truth map** — loaded from a parsed maze file and passed to `SimAPI`.

### Public interface

| Method / Property | Signature | Description |
|-------------------|-----------|-------------|
| `width` | `@property → int` | Number of columns |
| `height` | `@property → int` | Number of rows |
| `get_walls` | `(x, y) → int` | Raw wall bitmask for a cell |
| `has_wall` | `(x, y, d: Direction) → bool` | Check single-direction wall |
| `set_wall` | `(x, y, d: Direction) → None` | Set wall **and** symmetric neighbour wall |
| `clear_wall` | `(x, y, d: Direction) → None` | Clear wall **and** neighbour (perimeter-safe) |
| `mark_visit` | `(x, y) → None` | Increment visit counter |
| `get_visit_count` | `(x, y) → int` | Read visit counter |
| `export_walls` | `() → list[list[int]]` | Deep copy of wall matrix |
| `export_visits` | `() → list[list[int]]` | Deep copy of visit-count matrix |
| `distinct_cells_visited` | `() → int` | Cells visited ≥ 1 time |
| `total_visits` | `() → int` | Sum of all visit counts |

**Symmetry invariant:** `set_wall(x, y, N)` always also sets the S-wall on `(x, y+1)`. Perimeter walls that have no neighbour are silently skipped.

---

## `robot.py`

`Robot(x=0, y=0, heading=Direction.N)` — pure-state position and heading tracker. No I/O; no wall checking.

### Public interface

| Method / Property | Signature | Description |
|-------------------|-----------|-------------|
| `x`, `y` | `@property → int` | Current column / row |
| `heading` | `@property → Direction` | Current heading |
| `position` | `@property → (int, int)` | `(x, y)` tuple |
| `turn_right` | `() → None` | Rotate 90° clockwise |
| `turn_left` | `() → None` | Rotate 90° counter-clockwise |
| `move_forward` | `() → None` | Advance one cell; **no wall check** |
| `wall_front_dir` | `() → Direction` | Absolute direction in front |
| `wall_right_dir` | `() → Direction` | Absolute direction to the right |
| `wall_left_dir` | `() → Direction` | Absolute direction to the left |
| `wall_back_dir` | `() → Direction` | Absolute direction behind |
| `reset` | `(x=0, y=0, heading=N) → None` | Restore state |
