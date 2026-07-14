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

## Phase 3 – Algorithm Implementation

**Objective:** Implement two planning/replanning exploration algorithms built on the Phase 2 infrastructure, a shared `BaseAlgorithm` abstract class, and the extended `MetricsLogger`. Both algorithms adopt the **freespace assumption**: all unexplored cells are treated as passable until a wall is confirmed. Headless execution via `SimAPI` is already available from Phase 2.

**Wall sensing protocol (both algorithms):** After every `move_forward`, call `wall_front()`, `wall_back()`, `wall_left()`, `wall_right()` and update `MazeMap` with any newly discovered walls. A **replanning event** is triggered when a newly confirmed wall lies on the current plan.

### 3a – A* (Replanning from Scratch, baseline)
- [X] Implement `AStarExplorer` in `src/algorithms/astar.py` extending `BaseAlgorithm`
- [X] **Freespace assumption:** treat all cells with wall bitmask 0 (unexplored) as passable; movement cost = 1 for all traversable edges
- [X] **Heuristic:** before each A* search, run a multi-source BFS backward from the current goal set on the partial map (freespace assumption) to compute `h(s)` = exact shortest distance from `s` to the nearest goal under current knowledge; recomputed at every replanning event
- [X] **Replanning trigger:** after each `move_forward`, sense all four walls; if any newly confirmed wall lies on the current plan, replan from scratch from the current position; otherwise continue executing
- [X] **Plan execution:** translate the planned path into `turn_left` / `turn_right` / `move_forward` commands using `Robot.heading`
- [X] **Multi-goal:** when a goal cell is reached, remove it from the remaining-goals set; recompute BFS heuristic over the updated goal set; plan to the next nearest goal; repeat until all goals are reached
- [X] Log each replanning event via `MetricsLogger.log_replanning_event()` (see §8.1 of [`implementation_roadmap_revision.md`](instructions/implementation_roadmap_revision.md))
- [ ] **GUI display (reopened):** the current implementation only displays the f-value of cells on the returned path, redisplayed on each replanning event — extend this to show live search progress:

  | Element | Condition | Action |
  |---------|-----------|--------|
  | Cell text | Initial state | Empty (`""`) for all cells except start: display `f = h(start)` |
  | Cell text | After A* expansion | `set_text(x, y, str(f_value))` for each expanded cell and each cell added to the open list |
  | Cell color | When expanded | `set_color(x, y, 'b')` (Blue) |
  | Cell color | When added to open list | `set_color(x, y, 'R')` (Dark Red) |
  | Cell color | Replanning event | Clear all cell colors with `clearAllColor()` |
  | Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
  | Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
  | Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |
  | Cell text | Replanning event | Clear all text with `clear_all_text()`; redisplay f-values from the new search |

  Cell text format: `f-XXX` (3-digit f-value) or `f-inf ` for infinity (10-character display budget: two lines of five characters). Requires extending `_a_star` to return the actual expanded-cell and open-list-cell sets, not just their counts, so the display method can color/text every such cell instead of only the returned path
- [ ] Test end-to-end on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached and log correctness

### 3b – D*-Lite
- [X] Implement `DStarLiteExplorer` in `src/algorithms/dstar_lite.py` extending `BaseAlgorithm`
- [X] **Freespace assumption:** same as A*; edge cost = 1 for passable edges, `math.inf` for confirmed walls
- [X] **Heuristic:** compute once via BFS from the robot's start position on the initial partial map (all cells free); `h(s)` = BFS distance from `s` to start; consistent throughout the episode — the `km` accumulator accounts for agent movement
- [X] **Initialisation:** `g(s) = rhs(s) = ∞` for all cells except goal cells (`rhs(goal) = 0`); run `ComputeShortestPath` to produce the initial plan
- [X] **Replanning trigger:** after each `move_forward`, sense all four walls; for each newly confirmed wall `(u, v)`, set `c(u,v) = c(v,u) = ∞`, update rhs of affected nodes, insert inconsistent nodes into the priority queue, then call `ComputeShortestPath` to repair the plan incrementally
- [X] **Computational cost per event:** count only states extracted from the inconsistency queue whose key is non-stale at extraction time; stale-key re-insertions do not count as expansions
- [X] **Multi-goal:** initialise all goal cells with `rhs = 0`; when a goal is reached, set `rhs(reached_goal) = ∞`, update neighbours' rhs values, increment `km`, call `ComputeShortestPath` to re-route to the next goal; heuristic `h(s)` unchanged
- [X] Log each replanning event via `MetricsLogger.log_replanning_event()`
- [ ] **GUI display (reopened):** cell text must reflect the current `g`/`rhs` values of every affected cell whenever either changes — the current implementation only refreshes text for cells popped from the priority queue, which can leave stale values displayed between planning cycles:

  | Element | Condition | Action |
  |---------|-----------|--------|
  | Cell text | Initial state | `set_text(x, y, "g-inf r-inf")` for all cells; goal cells: `"g-inf r-0"` |
  | Cell text | After update | `set_text(x, y, f"g-{g} r-{rhs}")` whenever `g` or `rhs` changes, for every affected cell |
  | Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
  | Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
  | Inconsistent node | Inserted into queue | `set_color(x, y, 'R')` (Dark Red) |
  | Consistent node | Removed from queue (expanded) | `set_color(x, y, 'b')` (Blue) |
  | Trivial node | `g = rhs = ∞` | `clear_color(x, y)` |
  | Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |

  Cell text format: `g-XXXr-YYY` (`XXX`/`YYY` are the g-value/rhs-value, or `inf`; 10-character display budget). Requires a `set_text` call at every site that mutates `_g`/`_rhs`, not only inside `ComputeShortestPath` — at minimum in the rhs-recomputation routine and in the goal-reached/wall-discovery handling of the main loop
- [X] Test end-to-end on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached and log correctness

### 3c – `MetricsLogger` Extension
- [X] Extend `src/metrics/logger.py` with replanning-event tracking (additive; all existing Phase 2 metrics retained): new state `_replanning_events: list[dict]`, methods `start_plan_timer()` and `log_replanning_event(position, nodes_expanded, residual_distance, memory_occupancy)`, properties `total_replanning_events`, `cumulative_planning_time`, `cumulative_nodes_expanded`; extend `export_json` payload with `total_replanning_events`, `cumulative_planning_time_s`, `cumulative_nodes_expanded`, `replanning_events` (see §8.1 of [`implementation_roadmap_revision.md`](instructions/implementation_roadmap_revision.md) for the full per-event record schema)
- [X] Update `src/metrics/README.md` accordingly
- [X] Add `tests/test_metrics_logger.py` covering: event logging, per-event payload correctness, `export_json` schema validation

### 3d – Common Base (`BaseAlgorithm`)
- [X] Define `BaseAlgorithm` abstract class in `src/algorithms/base_algorithm.py` (note: `src/algorithms/base.py` remains as the Phase 1 test wall-follower)
- [X] **Constructor:** `BaseAlgorithm(api, maze_map, robot, logger, goals=None, n_random_goals=None, random_seed=None)` — goal handling resolved at construction: no args → maze centre; single cell list → single-goal; multiple cells → multi-goal (greedy nearest-goal visitation order); `n_random_goals=k, random_seed=s` → generate `k` random free cells with `random.Random(s)`
- [X] **Abstract method:** `run() → None` — sense → plan → act → log loop
- [ ] **Reset handling (reopened):** the loop above was marked complete assuming it already polled `was_reset()`/`ack_reset()`, but neither `AStarExplorer.run()` nor `DStarLiteExplorer.run()` actually does. Add a check once per iteration of the main sense→plan→act loop in both; on `was_reset() == True`, call `ack_reset()` and reset internal state: re-sync `robot` to the maze's start position/heading (capture `self._start_heading = robot.heading` in `BaseAlgorithm.__init__`, alongside the existing `self._start_pos`, and reset via those captured values rather than a bare `robot.reset()`, whose `(0,0,N)` default is only valid for a real MMS run — MMS mice always start at `(0,0)` facing North — and would be wrong for a headless `SimAPI` run started elsewhere). D*-Lite must also clear its search state (`_g`, `_rhs`, `_U`, `_km`) on reset
- [X] **Protected utilities:** `_sense_and_update(maze_map, robot, api) → list[tuple[Direction, bool]]`; `_execute_path(path, robot, api, logger) → bool`; `_compute_goal_heuristic(maze_map, goals) → dict`; `_compute_start_heuristic(maze_map, start) → dict`
- [X] Write integration tests: run each algorithm on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached, replanning event counts consistent between A* and D*-Lite on the same maze, JSON log validates against the extended schema

> **Milestone M3 — Both algorithms pass:** `AStarExplorer` and `DStarLiteExplorer` each reach all goals on at least one maze per difficulty level using the headless `SimAPI`. `python -m pytest tests/` passes including new metrics and integration tests.

---

## Phase 4 – Offline Path Search and Maze Difficulty Assessment

**Objective:** Implement the offline analysis tools used to compute solution-quality metrics and to categorise mazes by difficulty.

**Two difficulty levels** (original level 1 — tree-structured, no loops — removed as irrelevant for A* vs D*-Lite comparison):

| Level | Characteristics |
|-------|-----------------|
| Level 1 | 16×16, loops and dead ends, no islands |
| Level 2 | 16×16, islands and multiple loops; unsolvable by simple wall-following |

- [ ] Implement `Pathfinder` in `src/offline/pathfinder.py`: BFS on a graph built from a wall matrix (either the agent's partial map or the full maze); returns shortest path length and the set of expanded nodes (closed-list)
- [ ] Expose `shortest_path(wall_matrix, start, goal) → list[tuple]` and `closed_list_size(wall_matrix, start, goal) → int` as the public interface
- [ ] Implement `experiments/assess_difficulty.py`: loads every maze in `mazes/`, runs BFS from start to goal on the full map, records closed-list size, and prints a ranked table — use to assign mazes to **two levels** and select the final 6+ test mazes (≥ 3 per level)
- [ ] Select and commit the final 6+ mazes to `mazes/level1/` and `mazes/level2/`; record closed-list thresholds for each level in `docs/notes.md`
- [ ] Write unit tests for `Pathfinder` in `tests/test_pathfinder.py` on mazes with pre-computed solutions

> **Milestone M4 — Mazes classified:** All selected mazes parsed and classified. `Pathfinder` unit tests pass.

---

## Phase 5 – End-to-End Integration

**Objective:** Wire all modules together into a single runnable entry point, verify the complete pipeline, and implement the batch runner. All batch and analysis execution uses headless `SimAPI`; `run.py` at the repository root is the MMS GUI entry point only.

- [ ] Implement `run.py` at the repository root: parses `--algo {astar,dstar_lite}`, `--goal X Y` (repeatable for multi-goal), `--n-goals N`, `--seed S`; instantiates `MmsAPI`, `MazeMap`, `Robot`, the chosen algorithm, and a `MetricsLogger`; calls `algorithm.run()`, then `logger.export_json(output_dir=...)` (e.g. `results/logs/`) so GUI runs produce a log comparable to headless `SimAPI` runs; this is the script path configured in MMS
- [ ] Update `docs/mms.md` Step 3 ("Run Command Format"): the documented run command still invokes an algorithm module directly (`.venv/bin/python -m src.algorithms.[algorithm_name]`), which predates `run.py` and no longer applies — rewrite it to invoke `run.py` instead (`.venv/bin/python run.py --algo [astar|dstar_lite] [--goal X Y ...] [--n-goals N] [--seed S]`), and remove the `--log`/`--verbose` "common flags" example, since those flags belong to `run.py`'s own CLI, not to an `argparse` config on the algorithm modules (neither `astar.py` nor `dstar_lite.py` has one)
- [ ] Add a guard (test or lint rule) asserting no `print(` calls exist under `src/algorithms/` or `src/api/mms_api.py` outside the intended protocol I/O — MMS communicates over raw stdin/stdout, so a stray `print()` would desync the protocol; this class of bug is invisible to the existing `SimAPI`-based test suite (which never touches real stdin/stdout) and would only surface when running in the real MMS GUI
- [ ] Verify manual runs of both algorithms in MMS on at least one maze per difficulty level; confirm that walls, cell colours, and cell text are rendered correctly per the GUI specs in §9 of [`implementation_roadmap_revision.md`](instructions/implementation_roadmap_revision.md)
- [ ] Implement `experiments/run_batch.py`: iterates over all `(algorithm, maze_file)` combinations for both difficulty levels, runs each via `SimAPI`, saves the extended JSON log, reports a summary table (goal reached, total moves, replanning events, cumulative planning time)
- [ ] Run the batch suite on all selected mazes; fix any integration bugs surfaced
- [ ] Confirm that offline path lengths (partial map vs. full maze via `Pathfinder`) are computed and included in each JSON log

> **Milestone M5 — Pipeline complete:** `python experiments/run_batch.py` produces one JSON log per run for all combinations (2 algorithms × 2 levels × ≥ 3 mazes = ≥ 12 runs) with no unhandled exceptions.

---

## Phase 6 – Experimental Evaluation

**Objective:** Execute the full evaluation campaign and collect all data needed for the report.

- [ ] Run the complete batch suite; archive raw logs in `results/logs/` with filenames encoding `<algo>_<maze_name>_<timestamp>`
- [ ] Implement `experiments/analyze.py`: reads all JSON logs, aggregates per-algorithm statistics, and generates the following figures (saved to `results/plots/`):
  - Scatter plot of *computational cost per replanning event* vs *residual distance to goal*, A* and D*-Lite overlaid on the same graph (one plot per maze, or one combined plot per difficulty level)
  - Cumulative planning time over replanning events along an episode: stepped curve expected for A*, near-flat for D*-Lite
  - Memory occupancy of search structures over replanning events: bounded (resetting) for A*, monotonically increasing for D*-Lite
  - Heatmaps of the visit-count matrix for each run
  - Bar charts comparing total moves, distinct cells visited, and total visits across algorithms and difficulty levels
- [ ] Save all figures to `results/plots/`
- [ ] Manually verify at least two runs per algorithm in the MMS GUI to confirm that the headless `SimAPI` behaviour matches the real simulator
- [ ] Document any anomalies (e.g. D*-Lite memory growth on high-complexity mazes, divergent performance between difficulty levels) in `docs/notes.md`

> **Milestone M6 — Data collected:** All plots generated; all metrics tabulated for both algorithms across both difficulty levels.

---

## Phase 7 – Analysis and Final Report

**Objective:** Critically interpret the results and produce the final deliverable.

- [ ] Analyse the aggregated data: compare A* and D*-Lite across all metrics (moves, replanning events, cumulative planning time, per-event computational cost, memory occupancy) for each difficulty level
- [ ] Identify and explain edge cases (e.g. D*-Lite memory growth on high-complexity mazes, A* replanning overhead scaling with maze complexity, performance divergence between difficulty levels)
- [ ] Discuss the relationship between the BFS closed-list difficulty index and observed algorithm performance differences
- [ ] Write the final report in Rob-26-MazeSolver_report.md following the structure defined in the proposal
- [ ] Review and finalise README.md with setup instructions, run commands, and a brief results summary