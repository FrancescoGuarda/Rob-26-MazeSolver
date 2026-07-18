# Repository Structure

```
Rob-26-MazeSolver/
├── run.py
├── requirements.txt
├── README.md
├── LICENSE
│
├── src/
│   ├── api/
│   │   ├── base_api.py
│   │   ├── mms_api.py
│   │   └── sim_api.py
│   ├── core/
│   │   ├── constants.py
│   │   ├── maze_map.py
│   │   └── robot.py
│   ├── algorithms/
│   │   ├── base.py
│   │   ├── wall_following.py
│   │   ├── flood_fill.py
│   │   └── astar.py
│   ├── offline/
│   │   ├── pathfinder.py
│   │   └── maze_loader.py
│   └── metrics/
│       └── logger.py
│
├── mazes/
│   ├── level1/
│   ├── level2/
│   └── level3/
│
├── results/
│   ├── logs/          # <goal-count>/<algo>/*.json, e.g. one_goal/astar/
│   └── plots/
│
├── scripts/
│   ├── batch_run.py
│   ├── analyze.py
│   └── assess_difficulty.py
│
├── tests/
│   ├── conftest.py
│   ├── test_maze_map.py
│   ├── test_robot.py
│   ├── test_maze_loader.py
│   ├── test_pathfinder.py
│   ├── test_algorithms.py
│   └── test_metrics_logger.py
│
└── docs/
    ├── Rob_26_proposal.md
    ├── Rob-26-MazeSolver_report.md
    ├── notes.md
    ├── prompts.md
    ├── articles/
    └── res/
```

---

## Directory and File Descriptions

### `run.py`
The single entry point configured in MMS ("Run command: `python run.py --algo flood_fill --goal 7 7`"). It parses CLI arguments, wires together `MmsAPI`, the selected algorithm class, and `MetricsLogger`, then calls `algorithm.run()`. Keeping this at the root makes the MMS "Directory" field point directly to the repo root without any path gymnastics.

---

### `/src`
All Python source code for the project. Splitting source from the repo root avoids name collisions with config files and keeps `import` paths predictable.

#### `/src/api/`
Houses the API layer that decouples algorithms from the I/O backend.

- **`base_api.py`** — Abstract base class (`BaseAPI`) declaring every MMS command as an abstract method. All algorithms are coded against this interface, not against a concrete implementation.
- **`mms_api.py`** — Concrete implementation for live MMS runs: writes commands to `sys.stdout`, reads responses from `sys.stdin`, prints debug messages to `sys.stderr` (as required by MMS). This is essentially the canonical `mackorone/mms-python/API.py` adapted to the `BaseAPI` contract.
- **`sim_api.py`** — Concrete headless implementation backed by a `MazeMap` loaded from a file. Answers wall-sensor queries by consulting the ground-truth wall matrix, tracks robot position internally, raises a `CrashError` on invalid moves, and optionally records a step-by-step trace. Enables fully automated `pytest` and batch-run testing with no GUI.

#### `/src/core/`
Domain primitives shared across the entire codebase.

- **`constants.py`** — Enumerations and dictionaries: `Direction` (N, E, S, W with integer values), wall-bitmask encoding (N=1, E=2, S=4, W=8, combinations 0–15), MMS color character codes. Centralising these avoids magic strings scattered across algorithm files.
- **`maze_map.py`** — `MazeMap` class: owns two R×C NumPy arrays (`walls` and `visits`). Provides `set_wall(x, y, d)`, `get_walls(x, y)`, `record_visit(x, y)`, `is_explored(x, y)`, and serialisation to/from the JSON log format (nested lists). This is the shared state that the algorithm, the visualiser, and the offline path-finder all read.
- **`robot.py`** — `Robot` class: tracks `(x, y, heading)`. Provides `turn_left()`, `turn_right()`, `step_forward()` (updates position from heading), and `absolute_walls(front_wall, right_wall, left_wall, back_wall)` (translates relative sensor readings into absolute N/E/S/W booleans). Isolates coordinate-frame arithmetic to one place.

#### `/src/algorithms/`
One module per exploration strategy, all implementing the same interface.

- **`base.py`** — `BaseAlgorithm` abstract class: stores references to `BaseAPI`, `MazeMap`, `Robot`, `MetricsLogger`, goal coordinates. Implements the common sense-act loop skeleton (read sensors → update map → decide action → move → log) and reset handling (`was_reset` / `ack_reset`). Concrete algorithms override the decision method only.
- **`wall_following.py`** — `WallFollower`: right-hand (or configurable hand) rule. Simple and fast but not complete on mazes with disconnected island walls; documents this limitation explicitly.
- **`flood_fill.py`** — `FloodFill`: maintains a flood-value grid initialized from BFS distances to the goal. On each step moves to the accessible neighbour with the lowest flood value. Re-floods only the cells affected by a newly discovered wall (incremental update). Includes a navigation subroutine to route to the nearest unexplored frontier when locally trapped.
- **`astar.py`** — `AStarExplorer`: runs A\* on the current partial map (unknown cells treated as open), executes the first step of the computed path, and replans whenever a new wall invalidates the current plan. Uses Manhattan distance as the heuristic.

#### `/src/offline/`
Tools that operate on maze files or completed maps without the MMS simulator.

- **`maze_loader.py`** — Parsers for the two MMS-supported formats: ASCII `.maz` (grid of `+`, `-`, `|`, space characters) and `.num` (one row per cell: `X Y N E S W` flags). Returns a uniform `(width, height, wall_matrix)` tuple used by both `SimAPI` and `Pathfinder`.
- **`pathfinder.py`** — `Pathfinder` class: builds a graph from any wall matrix (complete or partial) and runs BFS (with an optional Dijkstra variant for weighted mazes). Exposes `shortest_path(start, goal)` → path length, and `closed_list_size(start, goal)` → number of nodes expanded, the difficulty metric from the referenced article.

#### `/src/metrics/`

- **`logger.py`** — `MetricsLogger`: incremented by `BaseAlgorithm` on every move and turn. At run completion, computes derived metrics (distinct cells, wall-matrix export, offline path lengths via `Pathfinder`) and writes a single JSON file and an optional CSV row to `results/logs/<goal-count>/<algo>/`. Filename convention: `<goal-count>/<algo>/<algo>_<maze_name>_<YYYYMMDD_HHMMSS>.json` — runs are bucketed first by how many goals they targeted (`one_goal/`, `two_goals/`, … `ten_goals/`, then numeric `12_goals/`) and then by algorithm, so the two algorithms sit side by side at each difficulty level and a long campaign remains browsable. Colliding filenames within the same second get a `_1`, `_2`, … suffix instead of overwriting.

---

### `/mazes/`
Maze files sourced from `tcp4me.com/mmr/mazes/` and `micromouseonline/mazefiles`, organised by difficulty. Each subdirectory contains both the `.maz` (ASCII) and `.num` formats of the same maze. Keeping mazes version-controlled ensures reproducibility of all experimental results.

- **`level1/`** — Simple mazes: ≤ 8×8 or 16×16 tree-structured (no loops, no islands). Suitable for algorithm correctness verification.
- **`level2/`** — Intermediate mazes: 16×16 with loops and dead-ends. Exercises backtracking and frontier selection.
- **`level3/`** — Complex mazes: 16×16 with islands, multiple loops, and structures that defeat wall-following. Stress-tests flood fill and A\* robustness.

---

### `/results/`
All generated artefacts. **Not committed to git** (add `results/logs/` and `results/plots/` to .gitignore) to avoid bloating the repository with binary and data files.

- **`logs/`** — One JSON file per `(algorithm, maze, goal count)` run, filed under `<goal-count>/<algo>/`. Each file contains all seven proposal metrics plus the serialised wall and visit matrices. Because the tree is two levels deep, analysis code should glob `results/logs/*/*/*.json` (or `rglob("*.json")`) rather than a flat pattern.
- **`plots/`** — PNG figures: heatmaps, barplots, and path-overlay visualisations. Named to match the corresponding log file.

---

### `/scripts/`
Standalone utility scripts that orchestrate the pipeline. Not imported as library code.

- **`batch_run.py`** — Iterates over all `(algorithm, maze_file)` pairs, instantiates `SimAPI` + the algorithm + a logger, runs to completion, saves logs, and prints a summary table. Allows full experimental evaluation without launching MMS.
- **`analyze.py`** — Reads all JSON logs from `results/logs/`, aggregates statistics by algorithm and difficulty level, and renders all figures to `results/plots/` using `matplotlib`.
- **`assess_difficulty.py`** — Loads every maze in `mazes/`, computes `Pathfinder.closed_list_size(start, goal)` on the full map, and prints a ranked table to help select the 9+ representative mazes and calibrate the level-1/2/3 thresholds.

---

### `/tests/`
`pytest` test suite. Tests use `SimAPI` exclusively (no MMS process needed in CI).

- **`conftest.py`** — Shared fixtures: a 4×4 hardcoded maze, a pre-instantiated `MazeMap` and `Robot`, and factory functions for each algorithm backed by `SimAPI`.
- **`test_maze_map.py`** — Unit tests for wall setting/getting, visit counting, neighbour queries, and matrix export.
- **`test_robot.py`** — Unit tests for heading arithmetic, position updates, and wall-direction translation.
- **`test_maze_loader.py`** — Roundtrip tests: parse a known `.maz` and `.num` file, assert wall matrix equality.
- **`test_pathfinder.py`** — Tests on mazes with known shortest-path lengths; tests `closed_list_size` monotonicity across difficulty levels.
- **`test_algorithms.py`** — Integration tests: each algorithm reaches the goal on the 4×4 fixture; wall-follower fails (loops) on a known island maze.
- **`test_metrics_logger.py`** — Tests move/turn counting, JSON schema validity, and offline path-length fields.

---

### docs
Project documentation. The existing structure is preserved and extended.

- **Rob_26_proposal.md** — Original project proposal (read-only reference).
- **`Rob-26-MazeSolver_report.md`** — The evolving final report, structured per the proposal index.
- **`notes.md`** — Running developer notes: MMS quirks, algorithm design decisions, closed-list thresholds, maze selection rationale.
- **`prompts.md`** — Prompt engineering notes (existing file).
- **`articles/`** — PDFs or links to referenced papers.
- **`res/`** — Images referenced by documentation (wall-configuration dictionary diagram, etc.).

---

### requirements.txt
Python package dependencies pinned to minor versions. At minimum: `numpy`, `matplotlib`, `pytest`. Adding `scipy` is optional for statistical analysis in Phase 7.