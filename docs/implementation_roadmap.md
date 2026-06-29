# High-Level Implementation Roadmap

## Phase 1 – Environment Setup and Familiarization

**Objective:** Establish a working development environment and verify end-to-end communication with the MMS simulator before writing any algorithm code.

- [ ] Download and install the MMS simulator binary (or build from source) and verify it launches correctly
- [ ] Clone the `mackorone/mms-python` template, configure it in MMS, and confirm that the left-wall-follower example runs successfully
- [ ] Set up a Python virtual environment and requirements.txt with initial dependencies (`numpy`, `matplotlib`, `pytest`)
- [ ] Scaffold the repository directory structure as defined in the *Repository Structure* section below
- [ ] Download at least three candidate maze files from `tcp4me.com/mmr/mazes/` (`.maz` and `.num` formats) and inspect their structure manually
- [ ] Document the MMS API contract (command names, expected responses, crash/reset behaviour) in notes.md

> **Milestone M1 — Environment verified:** MMS launches a Python algorithm, the algorithm reads sensor data, and moves the robot.

---

## Phase 2 – Core Infrastructure

**Objective:** Build the shared data structures and interfaces that all three algorithms will depend on, enabling independent development in Phase 3.

- [ ] Define `constants.py`: cardinal direction enum, wall bitmask encoding (N=1, E=2, S=4, W=8, values 0–15), MMS color and text codes
- [ ] Implement `MazeMap` in `maze_map.py`: 2-D wall matrix (R×C integers, 0 = unexplored) and 2-D visit-count matrix; methods to query/set walls for a cell and its neighbour, mark a visit, and export both matrices as nested lists
- [ ] Implement `Robot` in `robot.py`: position (x, y) and heading; methods for turning left/right, moving forward (updating position), and computing absolute wall directions from relative sensor readings (`wallFront`, `wallLeft`, `wallRight`, `wallBack`)
- [ ] Implement `MmsAPI` in `api/mms_api.py`: a thin, stateless wrapper around the MMS stdin/stdout protocol (mirrors `mackorone/mms-python/API.py`); all writes go to `sys.stdout`, all reads come from `sys.stdin`, all debug output goes to `sys.stderr`
- [ ] Define `BaseAPI` abstract class in `api/base_api.py` specifying the API contract (`maze_width`, `maze_height`, `wall_front`, `wall_right`, `wall_left`, `wall_back`, `move_forward`, `turn_left`, `turn_right`, `set_wall`, `set_color`, `set_text`, `get_stat`, `was_reset`, `ack_reset`)
- [ ] Implement `SimAPI` in `api/sim_api.py`: a headless implementation of `BaseAPI` backed by a loaded maze file; answers sensor queries by consulting the real maze structure, tracks robot position internally, and raises `Crash` on invalid moves — enabling fully automated batch testing without MMS
- [ ] Implement `maze_loader.py` in `offline/`: parsers for both the `.maz` (ASCII map) and `.num` (coordinate + NESW flags) maze file formats; returns a uniform internal representation (R×C wall matrix)
- [ ] Implement `MetricsLogger` in `metrics/logger.py`: accumulates moves, turns, distinct-cell count, total-visit count, and wall/visit matrices during a run; provides `export_json()` and `export_csv()` methods; writes output to `results/logs/`
- [ ] Write unit tests for `MazeMap`, `Robot`, `maze_loader`, and `MetricsLogger`

> **Milestone M2 — Infrastructure ready:** A toy algorithm can instantiate `MazeMap` + `Robot`, call `SimAPI`, and produce a valid metrics log without MMS running.

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
- [ ] Define `BaseAlgorithm` abstract class in `algorithms/base.py`: constructor takes a `BaseAPI` instance, a `MazeMap`, a `Robot`, a `MetricsLogger`, and goal coordinates; exposes a single `run()` method; handles the common sense-log-act loop and reset detection
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