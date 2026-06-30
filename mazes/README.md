# Maze Files Directory

This directory contains the maze files used for testing and evaluating the three maze-solving algorithms: **wall-following**, **flood fill**, and **A***.

## Organization

Mazes are organized into three difficulty levels:
- **`level1/`** — Simple mazes (8×8 or 16×16, tree-structured, no loops)
- **`level2/`** — Intermediate mazes (16×16, with loops and dead-ends)
- **`level3/`** — Complex mazes (16×16, with islands, multiple loops, wall-following-proof)

TODO: Add difficulty classification methodology and closed-list size thresholds.

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
- No inaccessible locations (all cells reachable from start)
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
- **East (E)**: bit 0 
- **North (N)**: bit 1 
- **West (W)**: bit 2 
- **South (S)**: bit 3 

### Dictionary

| Value | Walls (E, N, W, S) | Visual | Description |
|-------|---------------------|--------|-----|
| 0  | (1, 0, 0, 0) | <pre>╷  ·<br>╵  ·</pre> | East wall only |
| 1  | (0, 1, 0, 0) | <pre>╶──╴<br>·  ·</pre> | North wall only |
| 2  | (0, 0, 1, 0) | <pre>·  ╷<br>·  ╵</pre> | West wall only |
| 3  | (0, 0, 0, 1) | <pre>·  ·<br>╶──╴</pre> | South wall only |
| 4  | (1, 0, 0, 1) | <pre>╷  ·<br>└──╴</pre> | East + South |
| 5  | (0, 0, 1, 1) | <pre>·  ╷<br>╶──┘</pre> | West + South |
| 6  | (0, 1, 1, 0) | <pre>╶──┐<br>·  ╵</pre> | North + West |
| 7  | (1, 1, 0, 0) | <pre>┌──╴<br>╵  ·</pre> | North + East |
| 8  | (1, 0, 1, 0) | <pre>╷  ╷<br>╵  ╵</pre> | East + West |
| 9  | (0, 1, 0, 1) | <pre>╶──╴<br>╶──╴</pre> | North + South |
| 10 | (0, 1, 1, 1) | <pre>╶──┐<br>╶──┘</pre> | North + West + South |
| 11 | (1, 0, 1, 1) | <pre>╷  ╷<br>└──┘</pre> | East + West + South |
| 12 | (1, 1, 0, 1) | <pre>┌──╴<br>└──╴</pre> | North + East + South |
| 13 | (1, 1, 1, 0) | <pre>┌──┐<br>╵  ╵</pre> | North + East + West |
| 14 | (1, 1, 1, 1) | <pre>┌──┐<br>└──┘</pre> | All walls (full cell) |
| 15 | (0, 0, 0, 0) | <pre>·  ·<br>·  ·</pre> | No walls (open cell) |

### Usage in Code

The wall encoding is used internally by the project to represent the maze as a 2-D integer matrix; the ASCII map is parsed and converted to this numerical encoding for efficient lookup and manipulation.
