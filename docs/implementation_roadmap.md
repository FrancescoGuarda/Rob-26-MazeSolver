# High-Level Implementation Roadmap

## Phase 1 – Environment Setup and Familiarization

**Objective:** Establish a working development environment and verify end-to-end communication with the MMS simulator before writing any algorithm code.

- [X] Download and install the MMS simulator binary (or build from source) and verify it launches correctly
- [X] Clone the `mackorone/mms-python` template, configure it in MMS, and confirm that the left-wall-follower example runs successfully
- [X] Set up a Python virtual environment and requirements.txt with initial dependencies (`numpy`, `matplotlib`, `pytest`)
- [X] Scaffold the repository directory structure as defined in [`repo_structure.md`](repo_structure.md)
- [X] Download candidate maze files from `tcp4me.com/mmr/mazes/` and inspect their structure manually
- [X] Document the MMS API contract (command names, expected responses, crash/reset behaviour) in `notes.md`

> **Milestone M1 — Environment verified:** MMS launches a Python algorithm, the algorithm reads sensor data, and moves the robot.

---

## Phase 2 – Core Infrastructure

**Objective:** Build the shared data structures and interfaces that every algorithm depends on.

**Design:**
- Wall bitmask encoding: **N=1, E=2, S=4, W=8** (values 0–15), documented in `mazes/README.md`
- `src/api/mms_api.py` keeps its module-level functions (adapted from `mackorone/mms-python`'s `API.py`); `MmsAPI(BaseAPI)` wraps them for the shared `BaseAPI` contract
- `src/algorithms/base.py` is a self-contained Phase 1 wall-follower script, independent of the `BaseAlgorithm` hierarchy built in Phase 3
- `src/parser/maze_parser.py` parses the ASCII `.txt` Map format — the only maze format used anywhere in the project

**Checklist:**
- [X] `src/constants.py`: `Direction` enum (N=0,E=1,S=2,W=3), wall bitmask constants (N=1,E=2,S=4,W=8), `DIR_TO_WALL`, `DIR_TO_DELTA`, `OPPOSITE_DIR`, `DIR_TO_STR`, `COLORS`, `COLOR_HEX`
- [X] `src/maze_map.py` — `MazeMap(width, height)`: `_walls[y][x]` and `_visits[y][x]` matrices; `set_wall`/`clear_wall` with symmetric neighbour update; `mark_visit`; `export_walls`/`export_visits`; `distinct_cells_visited`; `total_visits`
- [X] `src/robot.py` — `Robot(x, y, heading)`: `turn_right`/`turn_left`/`move_forward` (no wall check); `wall_front/right/left/back_dir()`; `reset()`
- [X] `src/api/base_api.py` — `BaseAPI(ABC)`: abstract methods for all protocol categories (maze info, wall sensing, movement, display, control, stats)
- [X] `src/api/mms_api.py` — `MmsAPI(BaseAPI)` class delegating to the module-level functions; `get_stat` via `command(["getStat", stat])`
- [X] `src/api/sim_api.py` — `SimAPI(BaseAPI)`: headless simulator backed by an in-memory `wall_matrix`; sensor queries via bitmask lookup; `MouseCrashedError` on blocked `move_forward`; display methods are no-ops
- [X] `src/parser/maze_parser.py` — `parse_maze(filepath) → (wall_matrix, width, height)`: ASCII `.txt` format; cell `(x,y)` maps to `lines[2*(H-1-y)][4*x]`; returns N=1,E=2,S=4,W=8 bitmasks
- [X] `src/metrics/logger.py` — `MetricsLogger(algo, maze)`: `start`/`log_move`/`log_turn`/`stop`/`set_matrices`; properties `total_moves`, `distinct_cells_visited`, `total_visits`, `execution_time`; `export_json(output_dir)`
- [X] `mazes/README.md`: maze file format and wall-bitmask dictionary
- [X] `__init__.py` files for `src/api/`, `src/algorithms/`, `src/parser/`, `src/metrics/`
- [X] `tests/test_maze_map.py`: init, set/clear wall with symmetry, perimeter no-error, visit counts, export deep copy
- [X] `tests/test_robot.py`: turn cycles, move_forward all directions, wall_dir methods, reset
- [X] `tests/test_maze_parser.py`: dimensions, perimeter walls, known interior cell bitmask

> **Milestone M2 — Infrastructure ready:** `python -m pytest tests/` passes all unit tests. A toy script can instantiate `MazeMap` + `Robot`, call `SimAPI` with a parsed maze, and produce a valid JSON log in `results/logs/` — without MMS running.

---

## Phase 3 – Algorithm Implementation

**Objective:** Implement two planning/replanning exploration algorithms — A\* and D\*-Lite — sharing a common base class and metrics logger. Both adopt the **freespace assumption**: every unexplored cell is treated as passable until a wall is sensed.

**Wall sensing protocol (both algorithms):** after every `move_forward`, call `wall_front()`, `wall_back()`, `wall_left()`, `wall_right()` and update `MazeMap` with any newly discovered walls. A **replanning event** is triggered whenever a newly confirmed wall lies on the current plan.

### 3a – A\* (`src/algorithms/astar.py`, `AStarExplorer`)
- [X] Runs A\* from scratch on the current partial map (unknown cells treated as passable); movement cost is 1 for every traversable edge
- [X] **Heuristic:** `h(s)` defaults to a multi-source BFS run backward from the current goal set over the partial map (`"min_path"`, the wall-aware exact shortest distance under current knowledge), recomputed at every replanning event. A `--heuristic manhattan` mode is also available: straight-line distance to the nearest goal, ignoring wall knowledge
- [X] **Replanning trigger:** after each `move_forward`, sense all four walls; if any newly confirmed wall lies on the current plan, replan from scratch from the current position; otherwise continue executing
- [X] **Plan execution:** the planned path is translated into `turn_left`/`turn_right`/`move_forward` commands driven by `Robot.heading`
- [X] **Multi-goal:** on reaching a goal cell, remove it from the remaining-goals set, recompute the heuristic over what's left, and plan to the next nearest goal
- [X] Every replanning event is logged via `MetricsLogger.log_replanning_event()`
- [X] **GUI display:**

  | Element | Condition | Action |
  |---------|-----------|--------|
  | Cell text | Initial state | Empty (`""`) for all cells except start: display `f = h(start)` |
  | Cell text | After A\* expansion | `set_text(x, y, ...)` for each expanded cell and each cell added to the open list |
  | Cell color | When expanded | `set_color(x, y, 'b')` (Blue) |
  | Cell color | When added to open list | `set_color(x, y, 'R')` (Dark Red) |
  | Cell color | Replanning event | Clear all cell colors with `clear_all_color()` |
  | Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
  | Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
  | Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |
  | Cell text | Replanning event | Clear all text with `clear_all_text()`; redisplay values from the new search |

  Cell text format: `f-XXXh-YYY` (f-value and h-value, each shown as a 3-digit number, or `inf`; 10-character display budget, two lines of five characters)
- [X] Tested end-to-end on a 4×4 handcrafted maze via `SimAPI`; asserts all goals reached and logs correctness

### 3b – D\*-Lite (`src/algorithms/dstar_lite.py`, `DStarLiteExplorer`)
- [X] Incremental D\*-Lite: maintains `g`/`rhs` values over a priority queue and repairs the plan from newly discovered walls rather than replanning from scratch; edge cost is 1 for a passable edge, infinite for a confirmed wall
- [X] **Heuristic:** `h(s)` is the Manhattan distance from `s` to `s_start`, the robot's current position — recomputed as `s_start` moves; the `km` accumulator corrects the priority-queue keys for this drift, preserving D\*-Lite's incremental-repair guarantee without a full replan
- [X] **Initialisation:** `g(s) = rhs(s) = ∞` for all cells except goal cells (`rhs(goal) = 0`); `ComputeShortestPath` produces the initial plan
- [X] **Replanning trigger:** after each `move_forward`, sense all four walls; for each newly confirmed wall `(u, v)`, set `c(u,v) = c(v,u) = ∞`, update the `rhs` of affected nodes, insert inconsistent nodes into the priority queue, then call `ComputeShortestPath` to repair the plan incrementally
- [X] **Computational cost per event:** counts only states extracted from the inconsistency queue whose key is non-stale at extraction time; stale-key re-insertions do not count as expansions
- [X] **Multi-goal:** all goal cells start with `rhs = 0`; when a goal is reached, its `rhs` is set to `∞`, neighbours' `rhs` values are updated, `km` is incremented, and `ComputeShortestPath` re-routes to the next goal
- [X] Every replanning event is logged via `MetricsLogger.log_replanning_event()`
- [X] **GUI display:** cell text reflects the current `g`/`rhs` values of every affected cell whenever either changes

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

  Cell text format: `g-XXXr-YYY` (`XXX`/`YYY` are the g-value/rhs-value, or `inf`; 10-character display budget)
- [X] Tested end-to-end on a 4×4 handcrafted maze via `SimAPI`; asserts all goals reached and logs correctness

### 3c – `MetricsLogger` Extension
- [X] `src/metrics/logger.py` tracks replanning events additively alongside the Phase 2 metrics: `start_plan_timer()` opens a timing window; `log_replanning_event(position, nodes_expanded, residual_distance, memory_occupancy)` appends one record —
  ```json
  {
    "event_id": 0,
    "position": [x, y],
    "planning_time_s": 0.0012,
    "nodes_expanded": 34,
    "residual_distance": 12,
    "cost_ratio": 2.83,
    "memory_occupancy": 57
  }
  ```
  `residual_distance` is `-1` when infinite (no known path yet); `cost_ratio` (`nodes_expanded / residual_distance`) is `null` whenever `residual_distance` is non-finite or zero
- [X] Derived properties: `total_replanning_events`, `cumulative_planning_time`, `cumulative_nodes_expanded`
- [X] `export_json` payload includes `total_replanning_events`, `cumulative_planning_time_s`, `cumulative_nodes_expanded`, `replanning_events`, and `scenario` (goal-placement metadata — see Phase 4)
- [X] `src/metrics/README.md` documents the full exported schema
- [X] `tests/test_metrics_logger.py` covers event logging, per-event payload correctness, and `export_json` schema validation

### 3d – Common Base (`BaseAlgorithm`, `src/algorithms/base_algorithm.py`)
- [X] **Constructor:**
  ```python
  BaseAlgorithm(
      api, maze_map, robot, logger,
      goals=None, n_random_goals=None, random_seed=None,
      heuristic="min_path", verbose=True,
  )
  ```
  Goal resolution at construction time: no `goals`/`n_random_goals` → the maze's 4-cell centre area; `goals=[...]` → an explicit list (single cell → single-goal; multiple → multi-goal, greedy nearest-goal visitation order); `n_random_goals=k, random_seed=s` → `k` random free cells via `random.Random(s)`
- [X] `heuristic` selects the planning heuristic dispatched by `_compute_heuristic(maze_map, goals)`: `"min_path"` (default, wall-aware BFS via `_compute_goal_heuristic`) or `"manhattan"` (straight-line distance). Consulted only by `AStarExplorer` — `DStarLiteExplorer` always uses its own Manhattan-to-`s_start` heuristic, a different mathematical role tied to its `Key`/`km` consistency invariant, so the flag has no effect there
- [X] `verbose` (default `True`) gates two stderr helpers, `_report_walls` (one consolidated `[WALL] (x, y) n e s w` line per sensing event, `_` marking an absent wall) and `_report_replan` (one `[REPLAN] ...` line per replanning event, with `cost_ratio`/`time_ms` rounded to 2 decimals); neither affects the exported JSON log
- [X] **Abstract method:** `run() → None` — sense → plan → act → log loop
- [X] **Reset handling:** once per iteration of the sense-plan-act loop, both explorers poll `was_reset()`; on `True`, they call `ack_reset()` (resets the API backend's own position tracking) and `robot.reset()` (re-syncs the algorithm's internal position/heading tracking) — both default to the maze's fixed origin cell `(0, 0)` facing North, matching MMS's own mouse-reset convention. D\*-Lite additionally clears its search state (`_g`, `_rhs`, `_U`, `_km`) on reset
- [X] **Protected utilities:** `_sense_and_update(maze_map, robot, api) → list[tuple[Direction, bool]]`; `_execute_path(path, robot, api, logger) → bool`; `_compute_goal_heuristic(maze_map, goals) → dict`; `_compute_start_heuristic(maze_map, start) → dict`; `_compute_heuristic(maze_map, goals) → dict`
- [X] Integration tests: run each algorithm on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached, replanning event counts consistent between A\* and D\*-Lite on the same maze, JSON log validates against the extended schema

> **Milestone M3 — Both algorithms pass:** `AStarExplorer` and `DStarLiteExplorer` each reach all goals on at least one maze from the corpus using the headless `SimAPI`. `python -m pytest tests/` passes including the metrics and integration tests.

---

## Phase 4 – Detour-Index Goal Placement as Exploration-Difficulty Proxy

**Objective:** Provide an automated, deterministic goal-placement scheme that manufactures exploration difficulty within any maze. Difficulty is a property of *where the goals sit*, not of the maze's own structure, so the same 55-maze corpus is reused at every difficulty level instead of being split into tiers — no maze in the corpus is set aside as "easy" or "hard".

- [X] `src/goal_placement.py` — `place_goals()` / `scenario_goals(wall_matrix, width, height, start, k)` place `k` goals by maximizing the **detour index** `detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)`: a cell with detour ≈ 1 is "honest" about its distance, while a high-detour cell looks close but is actually far — exactly the kind of cell that defeats a greedy planner working from a partial map. Goal 1 maximizes detour from `start`; each subsequent goal `k ≥ 2` maximizes the *minimum* detour over `start` and all previously placed goals, so later goals stay deceptive relative to the whole set placed so far. Deterministic (no randomness); ties break to the lowest `(y, x)` (y=0 at the bottom, the simulator's origin row); scenarios are nested across `k` (a `k`-goal run's first `L` goals equal the `L`-goal run's goals)
- [X] `tools/place_goals.py` — CLI over `scenario_goals()` for manual inspection: prints each goal's cell and detour score for a named maze in `mazes/txt/`; writes nothing to disk
- [X] `tools/gen_maze.py` (generates a perfect maze — no loops, no islands — in the MMS ASCII format) and `tools/filter_connected.py` (drops any maze that isn't fully connected or fails to parse; full connectivity is a hard precondition for `residual_distance` to be finite in every exported log)
- [X] Automated placement is wired into both headless batch runners (`experiments/01_experiment.py -k N`, `experiments/run_batch.py`) and into `run.py` via `--auto-goals MAZE [-k N]`, so a GUI run can name a maze instead of pasting hand-copied `--goal X Y` coordinates; `run.py` checks the named maze's parsed dimensions against what MMS reports, to catch a stale maze name after switching mazes in the GUI
- [X] `MetricsLogger.set_scenario(maze_file, k, goals)` records which cells were placed and their detour scores in each exported log (`"scenario"` is `null` for non-automated goal runs, e.g. the default centre-area goal or an explicit `--goal` list)
- [X] `tools/README.md` documents the placement algorithm, its I/O contract, and its known limitations
- [X] `detour_metric_limitations.md` documents a known bias of the metric: normalizing by Manhattan distance rewards small denominators, so goals tend to cluster near the start, and the ratio measures *relative* deception rather than *absolute* difficulty. Kept as-is and flagged for the final report, since correcting it would invalidate the already-collected experimental logs
- [X] `notebooks/goals_analysis.ipynb` renders, for a representative maze, the score map `place_goals` maximizes at each step (k = 1..4), saved to `docs/res/goal_heatmap_evolution.svg`
- [X] Unit tests: `tests/test_goal_placement.py` (determinism, nesting across `k`, tie-breaking) and `tests/test_auto_goals.py` (`run.py --auto-goals` maze-name resolution and its dimension-mismatch guard)

> **Milestone M4 — Goal placement ready:** `scenario_goals()` deterministically places goals of increasing exploration difficulty on any maze in the corpus; `run.py --auto-goals` and both headless batch runners consume it; `python -m pytest tests/` passes including the goal-placement and auto-goals tests.

---

## Phase 5 – End-to-End Integration

**Objective:** Wire all modules together into runnable entry points, and verify the complete pipeline. `run.py` is the MMS GUI entry point; all headless batch and analysis execution goes through `experiments/`.

- [X] `run.py` (repository root): parses `--algo {astar,dstar_lite}`, `--goal X Y` (repeatable, for multi-goal), `--n-goals N --seed S` (random goals), `--auto-goals MAZE [-k N]` (in-process detour-index placement, dimension-checked against the maze MMS reports), `--heuristic {min_path,manhattan}` (A\* only), `--maze-name` (log-filename override — MMS never reports which file the GUI has loaded), `--output-dir`, and `--no-log`; `--goal`, `--n-goals`, and `--auto-goals` are mutually exclusive, and with none given the algorithm defaults to the maze's 4-cell centre area. Instantiates `MmsAPI`, `MazeMap`, `Robot`, the chosen algorithm, and a `MetricsLogger`; calls `algorithm.run()`, then (unless `--no-log`) `logger.export_json(output_dir=...)`, so GUI runs produce a log directly comparable to headless `SimAPI` runs. Also spawns a Tkinter legend window in its own process (Tkinter must own its process's main thread on macOS) mapping the running algorithm's `LEGEND` colors/text codes to their meaning; a legend window left over from a previous MMS "Run" click is closed automatically
- [X] `docs/mms.md` documents `run.py`'s full CLI, the GUI configuration steps, and a troubleshooting table
- [X] A guard (`tests/test_no_stray_print.py`) asserts no `print(` call exists under `src/algorithms/` or `src/api/mms_api.py` outside the intended stdin/stdout protocol I/O — MMS communicates over raw stdin/stdout, so a stray `print()` would desync the protocol; this class of bug is invisible to the `SimAPI`-based test suite (which never touches real stdin/stdout) and would only surface when running in the real MMS GUI
- [X] Manual runs of both algorithms verified in MMS on mazes from the corpus, confirming walls, cell colours, and cell text render correctly
- [X] `experiments/run_batch.py`: iterates over every `(algorithm, maze_file, goal_count)` combination across the full `mazes/txt/` corpus, runs each via `SimAPI`, saves the extended JSON log, and reports a summary table (goal reached, total moves, replanning events, cumulative planning time)
- [X] The batch suite runs clean across the full corpus with no unhandled exceptions

> **Milestone M5 — Pipeline complete:** `python experiments/run_batch.py` produces one JSON log per run for all combinations (2 algorithms × 4 goal-count scenarios × 55 mazes = 440 runs) with no unhandled exceptions.

---

## Phase 6 – Experimental Evaluation

**Objective:** Execute the full evaluation campaign and collect all data needed for the report.

- [X] Run the complete batch suite; archive raw logs in `results/logs/<goal_dir>/<algorithm>` with filenames encoding `<algo>_<maze_name>_<timestamp>.json`
- [X] `notebooks/data_analysis.ipynb`: reads all JSON logs, aggregates per-algorithm statistics, and generates the following figures (saved to `docs/res/`):
  - Linear regression plot of *computational cost per replanning event* vs *residual distance to goal*, A\* and D\*-Lite overlaid on the same graph, per goal-count scenario and aggregated across all scenarios
  - Average cumulative planning time over all runs per goal-count scenario: stepped bars expected for A\*, near-flat for D\*-Lite
  - [ ] Memory occupancy of search structures over replanning events: bounded (resetting) for A\*, monotonically increasing for D\*-Lite
- [X] Manually verified at least two runs per algorithm in the MMS GUI, confirming the headless `SimAPI` behaviour matches the real simulator
- [ ] Analyse the aggregated data: compare A\* and D\*-Lite across planning metrics (cumulative planning time, per-event computational cost, memory occupancy) for increasing goal-count scenarios, over all mazes in `mazes/txt/`
- [ ] Document results and anomalies (e.g. D\*-Lite memory growth on high-complexity mazes, divergent performance across goal-count scenarios) in `notebooks/report.md`

> **Milestone M6 — Data collected:** All plots generated; all metrics tabulated for both algorithms across all four goal-count scenarios (k=1..4) over the full maze corpus.

---

## Phase 7 – Analysis and Final Report

**Objective:** Critically interpret the results and produce the final deliverable.

- [ ] Write `notebooks/report.md`: the results and discussion write-up drawing on Phase 6's aggregated data
- [ ] Write the final report in `Rob-26-MazeSolver_report.md`, following the structure defined in the proposal, incorporating `notebooks/report.md`'s findings and the goal-placement metric's known limitations (documented in `detour_metric_limitations.md`) as an explicit discussion point
- [ ] Review and finalise `README.md` with setup instructions, run commands, and a brief results summary
