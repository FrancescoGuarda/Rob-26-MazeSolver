# High-Level Implementation Roadmap

## Phase 1 – Environment Setup and Familiarization

**Objective:** Establish a working development environment and verify end-to-end communication with the MMS simulator before writing any algorithm code.

- [X] Download and install the MMS simulator binary (or build from source) and verify it launches correctly
- [X] Clone the `mackorone/mms-python` template, configure it in MMS, and confirm that the left-wall-follower example runs successfully
- [X] Set up a Python virtual environment and requirements.txt with initial dependencies (`numpy`, `matplotlib`, `pytest`)
- [X] Scaffold the repository directory structure as defined in the *Repository Structure* section below
- [X] Download at least three candidate maze files from `tcp4me.com/mmr/mazes/` (`.maz` and `.num` formats) and inspect their structure manually
- [X] Document the MMS API contract (command names, expected responses, crash/reset behaviour) in notes.md

> **Milestone M1 — Environment verified:** MMS launches a Python algorithm, the algorithm reads sensor data, and moves the robot.

---

## Phase 2 – Core Infrastructure

**Objective:** Build the shared data structures and interfaces that all three algorithms will depend on, enabling independent development in Phase 3.

> See [`implementation.md`](../implementation.md) for the full implementation-ready detailed plan, all design decisions, and per-file specifications.

**Design decisions (resolved):**
- Wall bitmask encoding: **N=1, E=2, S=4, W=8** (values 0–15); `mazes/README.md` updated accordingly
- `mms_api.py`: keep module-level functions; add `MmsAPI(BaseAPI)` class that delegates to them (backward-compatible)
- `algorithms/base.py`: left unchanged (test wall-follower); `BaseAlgorithm` abstract class will go in `algorithms/base_algorithm.py` in Phase 3d
- `maze_parser.py` (ASCII only) placed under `src/parser/`; roadmap's `offline/maze_loader.py` (both formats) deferred to Phase 4
- `MetricsLogger` primary export: JSON via `export_json()`

**Checklist:**
- [X] `src/constants.py`: `Direction` enum (N=0,E=1,S=2,W=3), wall bitmask constants (N=1,E=2,S=4,W=8), `DIR_TO_WALL`, `DIR_TO_DELTA`, `OPPOSITE_DIR`, `DIR_TO_STR`, `COLORS`
- [X] `src/maze_map.py` — `MazeMap(width, height)`: `_walls[y][x]` and `_visits[y][x]` matrices; `set_wall`/`clear_wall` with symmetric neighbour update; `mark_visit`; `export_walls`/`export_visits`; `distinct_cells_visited`; `total_visits`
- [X] `src/robot.py` — `Robot(x, y, heading)`: `turn_right`/`turn_left`/`move_forward` (no wall check); `wall_front/right/left/back_dir()`; `reset()`
- [X] `src/api/base_api.py` — `BaseAPI(ABC)`: abstract methods for all protocol categories (maze info, wall sensing, movement, display, control, stats)
- [X] `src/api/mms_api.py` — add `MmsAPI(BaseAPI)` class delegating to existing module-level functions; add `get_stat` via `command(["getStat", stat])`
- [X] `src/api/sim_api.py` — `SimAPI(BaseAPI)`: headless simulator backed by `wall_matrix`; sensor queries via bitmask lookup; `MouseCrashedError` on blocked `move_forward`; display methods are no-ops
- [X] `src/parser/maze_parser.py` — `parse_maze(filepath) → (wall_matrix, width, height)`: ASCII `.txt` format; cell `(x,y)` maps to `lines[2*(H-1-y)][4*x]`; returns N=1,E=2,S=4,W=8 bitmasks
- [X] `src/metrics/logger.py` — `MetricsLogger(algo, maze)`: `start`/`log_move`/`log_turn`/`stop`/`set_matrices`; properties `total_moves`, `distinct_cells_visited`, `total_visits`, `execution_time`; `export_json(output_dir)`
- [X] Update `mazes/README.md`: encoding scheme and dictionary to N=1, E=2, S=4, W=8
- [X] Update `__init__.py` files: `src/api/`, `src/algorithms/`, new `src/parser/`, new `src/metrics/`
- [X] Create `results/logs/.gitkeep`
- [X] `tests/test_maze_map.py`: init, set/clear wall with symmetry, perimeter no-error, visit counts, export deep copy
- [X] `tests/test_robot.py`: turn cycles, move_forward all directions, wall_dir methods, reset
- [X] `tests/test_maze_parser.py`: dimensions, perimeter walls, known interior cell bitmask

> **Milestone M2 — Infrastructure ready:** `python -m pytest tests/` passes all unit tests. A toy script can instantiate `MazeMap` + `Robot`, call `SimAPI` with a parsed maze, and produce a valid JSON log in `results/logs/` — without MMS running.

---

## Phase 3 – Algorithm Implementation *(can proceed in parallel after Phase 2)*

**Objective:** Implement the three exploration strategies as independent, interchangeable modules built on the shared infrastructure.

### 3a – Wall-Following *(Member A)*
- [ ] Implement `WallFollower` in `algorithms/wall_following.py` extending `BaseAlgorithm`: right-hand (or left-hand) rule with heading tracking
- [ ] Handle the reset case (`was_reset` → reset internal state → `ack_reset`)
- [ ] Test on a simple tree-structured maze via `SimAPI`; verify the agent reaches the goal and that it fails (loops indefinitely) on an island maze — document this failure mode

### 3b – Flood Fill *(Member B)*
- [ ] Implement `FloodFill` in `algorithms/flood_fill.py`: BFS-based distance propagation from the goal across known open passages; re-flood only affected cells upon new wall discovery (incremental update)
- [ ] Define frontier selection strategy: always move to the accessible neighbour with the lowest flood value; if all neighbours are walled, use a navigation subroutine to route to the nearest unvisited cell
- [ ] Handle dead-end backtracking and the case where the flood value of all reachable cells is stale after new wall discovery
- [ ] Test on level-2 mazes (loops, dead-ends) via `SimAPI`

### 3c – Online A\* *(Member B)*
- [ ] Implement `AStarExplorer` in `algorithms/astar.py`: A\* on the partially known map, treating unknown cells as passable; re-plan from current position each time a new wall is discovered (replanning trigger)
- [ ] Use Manhattan distance to the goal as the admissible heuristic
- [ ] Implement a navigation executor that translates the planned path into a sequence of `turn_left`/`turn_right`/`move_forward` commands
- [ ] Test on level-2 and level-3 mazes via `SimAPI`; verify graceful handling of frequent re-planning on complex mazes

### 3d – Common base
- [ ] Define `BaseAlgorithm` abstract class in `algorithms/base_algorithm.py` (note: `algorithms/base.py` is reserved for the Phase 1 test wall-follower): constructor takes a `BaseAPI` instance, a `MazeMap`, a `Robot`, a `MetricsLogger`, and goal coordinates; exposes a single `run()` method; handles the common sense-log-act loop and reset detection
- [ ] Write integration tests that run each algorithm end-to-end on a small (4×4) handcrafted maze via `SimAPI` and assert goal-reached and log correctness

> **Milestone M3 — All algorithms pass:** All three algorithms reach the goal on at least one maze per difficulty level using the headless `SimAPI`.

---

## Phase 4 – Offline Path Search and Maze Difficulty Assessment

**Objective:** Implement the offline analysis tools used to compute solution-quality metrics and to categorise mazes by difficulty.

- [ ] Implement `Pathfinder` in `offline/pathfinder.py`: BFS (and optionally Dijkstra) on a graph built from a wall matrix (either the agent's partial map or the full maze); returns shortest path length and the set of expanded nodes (closed-list)
- [ ] Expose `shortest_path(wall_matrix, start, goal)` and `closed_list_size(wall_matrix, start, goal)` as the public interface
- [ ] Implement `scripts/assess_difficulty.py`: loads every maze in `mazes/`, runs BFS from start to goal on the full map, records closed-list size, and prints a ranked table — use this to assign mazes to levels 1/2/3 and select the final 9+ test mazes
- [ ] Select and commit the final 9+ mazes to `mazes/level1/`, `mazes/level2/`, `mazes/level3/`; record closed-list thresholds for each level in notes.md
- [ ] Write unit tests for `Pathfinder` on known mazes with pre-computed solutions

---

## Phase 5 – End-to-End Integration

**Objective:** Wire all modules together into a single runnable entry point, verify the complete pipeline, and implement the batch runner.

- [ ] Implement `run.py` at the repository root: parses `--algo {wall_following,flood_fill,astar}` and `--goal X Y` command-line arguments, instantiates `MmsAPI`, the chosen algorithm, and a `MetricsLogger`, then calls `algorithm.run()`; this is the script path configured in MMS
- [ ] Verify manual runs of all three algorithms in MMS on at least one maze per difficulty level; confirm that walls, cell colours, and cell text are rendered correctly in the simulator UI
- [ ] Implement `scripts/batch_run.py`: iterates over all `(algorithm, maze_file)` combinations, runs each via `SimAPI`, saves the metrics log, and reports a summary table (success/failure, move count, distinct cells)
- [ ] Run the batch suite on all 9 mazes; fix any integration bugs surfaced
- [ ] Confirm that offline path lengths (internal map vs. full maze) are computed and saved as part of each metrics log

> **Milestone M4 — Pipeline complete:** `python scripts/batch_run.py` produces one JSON log per run for all 27 (3 × 9) combinations with no unhandled exceptions.

---

## Phase 6 – Experimental Evaluation

**Objective:** Execute the full evaluation campaign and collect all data needed for the report.

- [ ] Run the complete batch suite; archive raw logs in `results/logs/` with filenames encoding `<algo>_<maze_name>_<timestamp>`
- [ ] Implement `scripts/analyze.py`: reads all logs, aggregates per-algorithm statistics, and generates:
  - Heatmaps of the visit-count matrix for each run (one subplot per algorithm per maze)
  - Barplots comparing total moves, distinct cells visited, and total visits across algorithms for each maze
  - Path-overlay visualisations comparing offline shortest path on internal map vs. full maze
- [ ] Save all figures to `results/plots/`
- [ ] Manually verify at least two runs per algorithm in the MMS GUI to confirm that the headless `SimAPI` behaviour matches the real simulator
- [ ] Document any anomalies (e.g. wall-follower looping on island maze, A\* replanning frequency) in notes.md

> **Milestone M5 — Data collected:** All plots generated; all metrics tabulated.

---

## Phase 7 – Analysis and Final Report

**Objective:** Critically interpret the results and produce the final deliverable.

- [ ] Analyse the aggregated data: compare algorithms across all seven metrics defined in the proposal for each difficulty level
- [ ] Identify and explain edge cases (island failure for wall-follower, replanning overhead for A\*, convergence behaviour of flood fill)
- [ ] Discuss the relationship between the BFS closed-list difficulty index and observed algorithm performance degradation
- [ ] Write the final report in Rob-26-MazeSolver_report.md following the structure defined in the proposal
- [ ] Review and finalise README.md with setup instructions, run commands, and a brief results summary