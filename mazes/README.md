# Maze Files Directory

This directory contains the maze files used for testing and evaluating the project's two maze-solving algorithms: online **A*** (`AStarExplorer`) and **D*-Lite** (`DStarLiteExplorer`). See [`src/algorithms/README.md`](../src/algorithms/README.md) for algorithm internals.

## Organization

All maze files live flat in [`txt/`](txt/).

## Difficulty via Goal Placement

Exploration difficulty is determined by **where the goals are placed** within a maze. Goal placement is scored by the **detour index** (BFS path length ÷ Manhattan distance): cells that look close but are actually far behind walls score highest, and are the most deceptive to a planner working from a partial map.

Placement is computed by `src/goal_placement.py` (`scenario_goals()`, called automatically by headless batch runs and by `run.py --auto-goals`) and can be inspected manually via the `tools/place_goals.py` CLI. See [`tools/README.md`](../tools/README.md) for the full detour-index algorithm description, the placement rules, and known limitations — it is not repeated here.

## File Format

### File Extension

All maze files must have the **`.txt`** extension for compatibility with the MMS simulator.

### Supported Formats

The MMS simulator supports two maze file formats:

1. **ASCII `.txt` format (Map format)** — Human-readable grid representation
   - Example: `maze_test.txt` (a classic 16×16 Micromouse maze)
   - Each cell is 5 spaces wide and 3 spaces tall
   - Walls are represented by non-space characters (`-`, `|`, `+`, `o`)
   - See the "Maze Map Format" section below for details

2. **Numerical `.txt` format (Num format)** — One cell per line
   - Format: `X Y N E S W` (coordinate + wall flags)
   - `N`, `E`, `S`, `W` are `1` if a wall exists in that direction, `0` otherwise
   - More compact and programmatically easier to parse

### Maze Requirements

For a maze to be valid in the simulator, it must satisfy:

**Basic requirements:**
- Nonempty
- Rectangular (uniform dimensions)
- Fully enclosed (perimeter walls on all sides)

**Micromouse competition requirements (official mazes):**
- No inaccessible locations (all cells reachable from start) — this is also a
  hard dependency of the metrics subsystem: `residual_distance` in exported
  logs is only guaranteed finite because every maze in the corpus satisfies
  full connectivity (enforced by `tools/filter_connected.py`). Any maze added
  outside that tool must be run through it (or an equivalent connectivity
  check) before use.
- Exactly three starting walls (around the start position)
- Only one entrance to the center goal area
- Hollow center (center peg has no walls)
- Walls on every peg except the center peg
- Unsolvable by simple wall-following (ensures variety in algorithm performance)

## Maze Map Format (ASCII)

### Structure

Each cell occupies a **5 characters wide × 3 lines tall** area in the file:

```
o x o
x   x
o x o
```

Walls are determined by checking the **marked positions (x)** for non-space characters:
- **Top wall**: Check position `(x+2, y)`
- **Right wall**: Check position `(x+4, y+1)`
- **Bottom wall**: Check position `(x+2, y+2)`
- **Left wall**: Check position `(x, y+1)`

### Example: 3×3 Maze

```
o---o---o---o
|       |   |
+   +   +   +
|   |       |
+---+---+---+
```

In this maze:
- Cell (0,0) has walls on North, West, and South
- Cell (1,0) has no walls (open passage)
- Cell (2,0) has walls on North, East, and a partial South

A full example of a standard 16×16 competition maze is provided in [maze_test.txt](maze_test.txt).

### Interpretation Rules

- **All non-space characters** count as walls (including `+`, `-`, `|`, `o`, etc.)
- **Spaces** represent open passages
- The grid must be rectangular and fully enclosed by a perimeter of walls

## Wall Configuration Dictionary 

Hereafter is reported the numerical dictionary corresponding to all possible configurations of walls around a cell in the maze. The dictionary is used to convert the ASCII representation of the maze into a numerical representation, easier to handle programmatically. 

### Encoding Scheme

Each cell is encoded as an integer from **0 to 15**, representing the bitmask of walls around that cell:
- **North (N)**: bit 0 → value **1**
- **East (E)**: bit 1 → value **2**
- **South (S)**: bit 2 → value **4**
- **West (W)**: bit 3 → value **8**

The decimal value of a cell's encoding is the **sum** of the bitmask values of its walls (e.g., a cell with North and West walls = 1 + 8 = 9). This is the standard additive bitmask encoding used in micromouse literature.

### Dictionary

( read binary from left to right: N, E, S, W; MSB to the left )

| Value | N | E | S | W | View | Description |
|-------|---|---|---|---|------|-------------|
| 0  | 0 | 0 | 0 | 0 |<pre>·  ·<br>·  ·</pre>| No walls (open cell) |
| 1  | 1 | 0 | 0 | 0 |<pre>╶──╴<br>·  ·</pre>|North only |
| 2  | 0 | 1 | 0 | 0 |<pre>·  ╷<br>·  ╵</pre>|East only |
| 3  | 1 | 1 | 0 | 0 |<pre>╶──┐<br>·  ╵</pre>|North + East |
| 4  | 0 | 0 | 1 | 0 |<pre>·  ·<br>╶──╴</pre>|South only |
| 5  | 1 | 0 | 1 | 0 |<pre>╶──╴<br>╶──╴</pre>|North + South |
| 6  | 0 | 1 | 1 | 0 |<pre>·  ╷<br>╶──┘</pre>|East + South |
| 7  | 1 | 1 | 1 | 0 |<pre>╶──┐<br>╶──┘</pre>|North + East + South |
| 8  | 0 | 0 | 0 | 1 |<pre>╷  ·<br>╵  ·</pre>|West only |
| 9  | 1 | 0 | 0 | 1 |<pre>┌──╴<br>╵  ·</pre>|North + West |
| 10 | 0 | 1 | 0 | 1 |<pre>╷  ╷<br>╵  ╵</pre>|East + West |
| 11 | 1 | 1 | 0 | 1 |<pre>┌──┐<br>╵  ╵</pre>|North + East + West |
| 12 | 0 | 0 | 1 | 1 |<pre>╷  ·<br>└──╴</pre>|South + West |
| 13 | 1 | 0 | 1 | 1 |<pre>┌──╴<br>└──╴</pre>|North + South + West |
| 14 | 0 | 1 | 1 | 1 |<pre>╷  ╷<br>└──┘</pre>|East + South + West |
| 15 | 1 | 1 | 1 | 1 |<pre>┌──┐<br>└──┘</pre>|All walls (full cell) |

### Usage in Code

The wall encoding is used internally by the project to represent the maze as a 2-D integer matrix; the ASCII map is parsed and converted to this numerical encoding for efficient lookup and manipulation.
