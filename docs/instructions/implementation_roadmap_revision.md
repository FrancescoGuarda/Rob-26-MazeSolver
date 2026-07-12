# Revision of [`implementation_roadmap.md`](../implementation_roadmap.md)

> **Purpose of this document:** Operational blueprint for updating `implementation_roadmap.md`. Each section describes precisely what changes, what replaces it, and why. Phases not listed are unchanged. All claims have been cross-checked against the current implementation state in `implementation.md` and the relevant source files (`src/metrics/logger.py`, `src/api/base_api.py`, etc.).

---

## Source of Change

After a review with the course professor of the original project proposal described in [`Rob_26_proposal`](../Rob_26_proposal.md), we have been asked to modify the scope, according to the following extract of the project-update email:

> Unlike the original proposal, which envisaged a comparison of online exploration strategies (wall-following, Flood Fill, and A*) without adopting a freespace assumption—where node expansion essentially coincided with the agent's physical movement—the revised proposal introduces a clear separation between the planning and execution phases, according to the following modifications:
> 
> * **Freespace assumption:** The agent is assumed to know only the start and goal coordinates a priori, while the maze layout is initially unknown. However, all unexplored cells are temporarily assumed to be free of obstacles (freespace assumption) until exploration proves otherwise.
> 
> * **Planning and replanning:** At each planning cycle, the agent computes a path from its current position to the goal using the map available at that time (i.e., confirmed walls together with the freespace assumption for unexplored cells), and subsequently executes the corresponding sequence of actions.
> 
> * **Walls as replanning triggers:** Whenever a wall detected during exploration invalidates the current plan, a replanning event is triggered from the agent's current position.
> 
> * **D* Lite as the core of the analysis, A* as the baseline:** D* Lite is evaluated against an A* approach that replans from scratch after every map update, with the latter serving as the baseline for comparison.
> 
> * **Updated evaluation metrics:** In addition to the previously defined metrics, the evaluation will include measures of the computational cost associated with each replanning event, considering both execution time and the memory occupancy of the search data structures (e.g., open and closed lists).
> 
> * **Evaluation scenarios:** The evaluation is reduced to two difficulty levels by removing the simplest scenario, which contains neither islands nor dead ends and is now considered of limited relevance for the comparison. This allows the analysis to focus on scenarios where the two approaches exhibit the greatest differences.
> 
> * **Headless execution:** A headless execution mode will be introduced to enable automated batch simulations and systematic log collection without requiring a graphical user interface.
> 
> * **Multi-goal extension:** Extension to a sequence of *n* goals, with order of visitation not defined a priori, but optimally determined during exploration.

Hereafter are listed all required changes to `implementation_roadmap.md`, cross-checked against the current implementation state, with precise specifications for each affected phase.

---

## Phases Unchanged

**Phase 1** — complete; no changes.  
**Phase 2** — complete; no changes to existing deliverables. `MetricsLogger` extension is deferred to Phase 3 (part of algorithm implementation).  
**Phase 7** — Analysis and Final Report — objective unchanged; metrics and algorithms change, but the phase structure stands.

---

## Phase 3 — Algorithm Implementation (Complete Replacement)

**Replace the entire Phase 3 section** with the following.

---

### Phase 3 — Algorithm Implementation

**Objective:** Implement two planning/replanning exploration algorithms built on the Phase 2 infrastructure, a shared `BaseAlgorithm` abstract class, and the extended `MetricsLogger`. Both algorithms adopt the freespace assumption: all unexplored cells are treated as passable until a wall is confirmed.

**Wall sensing protocol (both algorithms):** After every `move_forward`, call `wall_front()`, `wall_back()`, `wall_left()`, `wall_right()` and update `MazeMap` with any newly discovered walls. A **replanning event** is triggered when a newly confirmed wall lies on the current plan.

#### 3a — A* (Replanning from Scratch, baseline)

- [ ] Implement `AStarExplorer` in `src/algorithms/astar.py` extending `BaseAlgorithm`
- [ ] **Freespace assumption:** treat all cells with wall bitmask 0 (unexplored) as passable. Movement cost = 1 for all traversable edges.
- [ ] **Heuristic:** before each A* search, run a multi-source BFS backward from the current goal set on the partial map (freespace assumption) to compute `h(s)` = exact shortest distance from `s` to the nearest goal under current knowledge. This heuristic is admissible (partial map cannot overestimate) and is recomputed at every replanning event.
- [ ] **Replanning trigger:** after each `move_forward`, sense all four walls; if any newly confirmed wall lies on the current plan, trigger replanning from the current position. If no wall invalidates the plan, continue executing.
- [ ] **Plan execution:** translate the planned path into a sequence of `turn_left` / `turn_right` / `move_forward` commands, using `Robot.heading` to compute required turns.
- [ ] **Multi-goal:** when a goal cell is reached, remove it from the remaining-goals set; recompute BFS heuristic over the updated goal set; plan to the next nearest goal. Repeat until all goals are reached.
- [ ] Log each replanning event via `MetricsLogger.log_replanning_event()` (see §8.1)
- [ ] Test end-to-end on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached and log correctness

#### 3b — D*-Lite

- [ ] Implement `DStarLiteExplorer` in `src/algorithms/dstar_lite.py` extending `BaseAlgorithm`
- [ ] **Freespace assumption:** same as A*. Edge cost = 1 for passable edges, `math.inf` for walls.
- [ ] **Heuristic:** compute once via BFS/Dijkstra from the robot's start position forward on the initial partial map (all cells free). `h(s)` = BFS distance from `s` to start. This heuristic is consistent and is used unchanged throughout the episode (the `km` accumulator in D*-Lite accounts for the agent moving away from start).
- [ ] **Initialisation:** `g(s) = rhs(s) = ∞` for all cells except goal cells, which have `rhs(goal) = 0`. Run `ComputeShortestPath` to produce the initial plan.
- [ ] **Replanning trigger:** after each `move_forward`, sense all four walls; for each newly confirmed wall `(u, v)`, set edge cost `c(u,v) = c(v,u) = ∞`, update `rhs` of affected nodes, insert inconsistent nodes into the priority queue, then call `ComputeShortestPath` to repair the plan incrementally.
- [ ] **Computational cost per event:** count only states *extracted from* the inconsistency queue that are processed (i.e., whose key does not become stale upon extraction — stale-key re-insertions do not count as expansions).
- [ ] **Multi-goal:** goal cells are initialised with `rhs = 0`. When a goal cell is reached, set `rhs(reached_goal) = ∞`; update its neighbours' rhs values and insert inconsistent nodes into the queue; call `ComputeShortestPath` to re-route to the next goal. The heuristic `h(s)` (distance to start) remains unchanged.
- [ ] Log each replanning event via `MetricsLogger.log_replanning_event()` (see §8.1)
- [ ] Test end-to-end on a 4×4 handcrafted maze via `SimAPI`; assert all goals reached and log correctness

#### 3c — `MetricsLogger` Extension

- [ ] Extend `src/metrics/logger.py` with replanning-event tracking (additive; all existing Phase 2 metrics retained). See §8.1 for the exact implementation spec.
- [ ] Update `src/metrics/README.md` accordingly
- [ ] Add `tests/test_metrics_logger.py` covering: event logging, per-event payload, `export_json` schema

#### 3d — `BaseAlgorithm` Abstract Class

- [ ] Define `BaseAlgorithm` in `src/algorithms/base_algorithm.py` (note: `src/algorithms/base.py` remains as the Phase 1 test wall-follower)
- [ ] **Constructor signature:**
  ```python
  BaseAlgorithm(
      api: BaseAPI,
      maze_map: MazeMap,
      robot: Robot,
      logger: MetricsLogger,
      goals: list[tuple[int,int]] | None = None,
      n_random_goals: int | None = None,
      random_seed: int | None = None,
  )
  ```
- [ ] **Goal handling logic** (resolved at construction time):
  - `goals=None, n_random_goals=None` → single goal at maze centre (cells `(W//2-1, H//2-1)`, `(W//2, H//2-1)`, `(W//2-1, H//2)`, `(W//2, H//2)` for a 16×16 maze)
  - `goals=[single cell]` → single-goal mode
  - `goals=[multiple cells]` → multi-goal mode; visitation order determined greedily during exploration (nearest remaining goal by BFS on current partial map at the time each goal is reached)
  - `n_random_goals=k, random_seed=s` → generate `k` random free cells as goals using `random.Random(s)`; fallback to maze centre if seed is `None`
- [ ] **Abstract method:** `run() → None` — sense → plan → act → log loop; handles `was_reset()` → `ack_reset()` → reset internal state
- [ ] **Shared utilities** (provided as protected methods): `_sense_and_update(maze_map, robot, api) → list[tuple[Direction, bool]]` (senses all 4 walls, returns list of `(direction, is_new_wall)` for walls discovered this step), `_execute_path(path, robot, api, logger)` (translates cell path to turn/move commands)
- [ ] Write integration tests: run each algorithm on a small (4×4) handcrafted wall matrix via `SimAPI`; assert all goals reached, replanning event count is consistent between A* and D*-Lite on the same maze, and JSON log validates against the extended schema

> **Milestone M3 — Both algorithms pass:** `AStarExplorer` and `DStarLiteExplorer` each reach all goals on at least one maze per difficulty level (headless `SimAPI`). `python -m pytest tests/` passes, including the new metrics and integration tests.

---

## Phase 4 — Offline Analysis and Maze Selection (Partial Revision)

**Replace** Phase 4 checklist item "Select and commit the final 9+ mazes…" and the `scripts/assess_difficulty.py` item as follows.

**Two difficulty levels** (rename existing levels 2 and 3 as new levels 1 and 2):

| New level | Old level | Characteristics |
|-----------|-----------|-----------------|
| Level 1   | Level 2   | 16×16, loops and dead ends, no islands |
| Level 2   | Level 2/3 | 16×16, islands, multiple loops; unsolvable by simple wall-following |

The original level 1 (tree-structured, no loops) is **removed** from scope as it is not relevant for the A* vs D*-Lite comparison.

**Updated Phase 4 checklist** (replace original):
- [ ] Implement `src/offline/pathfinder.py` — `Pathfinder` class: BFS on a wall matrix; exposes `shortest_path(wall_matrix, start, goal) → list[tuple]` and `closed_list_size(wall_matrix, start, goal) → int`
- [ ] Implement `experiments/assess_difficulty.py`: loads every maze in `mazes/`, runs BFS from start to goal on the full map, records closed-list size, prints a ranked table — use to assign mazes to **two levels** and select the final 6+ test mazes (≥ 3 per level)
- [ ] Select and commit the final 6+ mazes to `mazes/level1/` and `mazes/level2/`; record closed-list thresholds for each level in `docs/notes.md`
- [ ] Write unit tests for `Pathfinder` in `tests/test_pathfinder.py` on mazes with pre-computed solutions

> **Milestone M4 (updated):** All selected mazes parsed and classified. `Pathfinder` unit tests pass.

---

## Phase 5 — End-to-End Integration (Partial Revision)

**Replace all references to `scripts/`** with `experiments/`. The `run.py` entry point at the repo root remains unchanged (still the MMS GUI entry point).

**Updated Phase 5 checklist** (replace original):
- [ ] Implement `run.py` at repository root: parses `--algo {astar, dstar_lite}`, `--goal X Y` (repeatable for multi-goal), `--n-goals N`, `--seed S`; instantiates `MmsAPI`, the chosen algorithm, `MazeMap`, `Robot`, and `MetricsLogger`; calls `algorithm.run()`; this is the script path configured in MMS
- [ ] Verify manual runs of both algorithms in MMS on at least one maze per difficulty level; confirm that walls, cell colours, and cell text are rendered correctly per the GUI specs in §9
- [ ] Implement `experiments/run_batch.py`: iterates over all `(algorithm, maze_file)` combinations for both difficulty levels, runs each via `SimAPI`, saves the extended JSON log, reports a summary table (goal reached, total moves, replanning events, cumulative planning time)
- [ ] Run the batch suite on all selected mazes; fix any integration bugs
- [ ] Confirm that offline path lengths (partial map vs. full maze via `Pathfinder`) are computed and included in each JSON log

> **Milestone M5 (updated):** `python experiments/run_batch.py` produces one JSON log per run for all combinations (2 algorithms × 2 levels × ≥ 3 mazes = ≥ 12 runs) with no unhandled exceptions.

---

## Phase 6 — Experimental Evaluation (Partial Revision)

**Replace all references to `scripts/`** with `experiments/`. **Replace** the analysis outputs with the revised visualisations below.

**Updated Phase 6 checklist** (replace original):
- [ ] Run the complete batch suite; archive raw logs in `results/logs/`
- [ ] Implement `experiments/analyze.py`: reads all JSON logs, aggregates per-algorithm statistics, and generates the following figures (saved to `results/plots/`):
  - Scatter plot of *computational cost per replanning event* vs *residual distance to goal*, A* and D*-Lite overlaid on the same graph (one plot per maze, or one combined plot per difficulty level)
  - Cumulative planning time over replanning events along an episode: stepped curve expected for A*, near-flat for D*-Lite
  - Memory occupancy of search structures over replanning events: bounded (resetting) for A*, monotonically increasing for D*-Lite
  - Heatmaps of the visit-count matrix for each run
  - Bar charts comparing total moves, distinct cells visited, and total visits across algorithms and difficulty levels
- [ ] Manually verify at least two runs per algorithm in the MMS GUI to confirm headless `SimAPI` behaviour matches the real simulator
- [ ] Document anomalies (e.g., D*-Lite memory growth on high-complexity mazes) in `docs/notes.md`

> **Milestone M6 (updated):** All plots generated; all metrics tabulated for both algorithms across both difficulty levels.

---

## §8 — Supplementary Specifications

These sections provide the exact implementation detail required for Phase 3.

---

### §8.1 — `MetricsLogger` Extension (`src/metrics/logger.py`)

**Design principle:** additive only. All existing Phase 2 state and methods remain unchanged. Add the following.

#### New internal state

```python
_replanning_events: list[dict]   # per-event records; initially []
_plan_timer_start: float | None  # set at start of each replanning call
```

#### New recording methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `start_plan_timer` | `() → None` | Record `_plan_timer_start = time.monotonic()` at the start of a planning/replanning call |
| `log_replanning_event` | `(position, nodes_expanded, residual_distance, memory_occupancy) → None` | Append one event record; compute `planning_time_s` from `_plan_timer_start`; compute `cost_ratio = nodes_expanded / residual_distance` (or `None` if `residual_distance == 0`) |

#### Per-event record structure (appended to `_replanning_events`)

```python
{
    "event_id":           int,     # 0-indexed
    "position":           [x, y],  # robot position at time of replanning
    "planning_time_s":    float,   # wall-clock seconds for this replanning call
    "nodes_expanded":     int,     # A*: open+closed size; D*-Lite: extracted states (see §8.4)
    "residual_distance":  int,     # BFS distance from position to nearest goal on current partial map
    "cost_ratio":         float | None,  # nodes_expanded / residual_distance
    "memory_occupancy":   int,     # A*: open+closed size (resets each event); D*-Lite: total non-trivial nodes (monotone)
}
```

#### New computed properties

| Property | Type | Description |
|----------|------|-------------|
| `replanning_events` | `list[dict]` | Full per-event list |
| `total_replanning_events` | `int` | `len(_replanning_events)` |
| `cumulative_planning_time` | `float` | Sum of `planning_time_s` over all events |
| `cumulative_nodes_expanded` | `int` | Sum of `nodes_expanded` over all events |

#### Extended `export_json` payload

Add the following keys to the existing JSON payload (do not remove existing keys):

```json
"total_replanning_events": <int>,
"cumulative_planning_time_s": <float>,
"cumulative_nodes_expanded": <int>,
"replanning_events": [ { ...per-event record... }, ... ]
```

---

### §8.2 — `BaseAlgorithm` protected utilities

The following shared utilities should be implemented as `protected` methods on `BaseAlgorithm` and available to both subclasses.

**`_sense_and_update(maze_map, robot, api) → list[tuple[Direction, bool]]`**

Calls `api.wall_front()`, `api.wall_right()`, `api.wall_back()`, `api.wall_left()`, maps each reading to an absolute direction via `robot.wall_*_dir()`, sets walls in `maze_map` via `maze_map.set_wall()`, and returns a list of `(absolute_direction, is_new_wall)` for all four directions. `is_new_wall = True` if the wall was not previously recorded in `maze_map`.

**`_execute_path(path, robot, api, logger) → bool`**

Takes an ordered list of `(x, y)` cells representing the planned path from the current position. Translates each step to the required turns and one `move_forward`. Returns `True` if the step was executed without triggering a replanning event (no new wall on path), `False` if a newly sensed wall invalidates a future step in the current plan (caller should replan).

**`_compute_goal_heuristic(maze_map, goals) → dict[tuple[int,int], int]`**

Runs multi-source BFS backward from all cells in `goals` on the current partial map (treating wall bitmask 0 as passable). Returns `h[cell]` = BFS distance from `cell` to the nearest goal. Used by A* before each replanning event.

**`_compute_start_heuristic(maze_map, start) → dict[tuple[int,int], int]`**

Runs BFS forward from `start` on the current partial map (freespace assumption). Returns `h[cell]` = BFS distance from `cell` to `start`. Used by D*-Lite once at initialisation (the km-adjustment handles robot movement).

---

### §8.3 — A* Algorithm Spec (`src/algorithms/astar.py`)

**Class:** `AStarExplorer(BaseAlgorithm)`

**`run()` loop:**

```
1. logger.start()
2. Compute initial heuristic: h = _compute_goal_heuristic(maze_map, goals)
3. Plan: path = a_star(robot.position, goals, maze_map, h)
4. While remaining_goals not empty:
    a. For each step in path:
        i.  Execute one turn/move via _execute_path (one step)
        ii. logger.log_turn() / logger.log_move() as appropriate
        iii. new_walls = _sense_and_update(maze_map, robot, api)
        iv. If robot.position in remaining_goals:
               remove goal; if no more goals: break outer loop
        v.  If any new_wall lies on remaining path:
               logger.start_plan_timer()
               h = _compute_goal_heuristic(maze_map, remaining_goals)
               path = a_star(robot.position, remaining_goals, maze_map, h)
               residual = h[robot.position]
               memory = len(open_list) + len(closed_list)
               logger.log_replanning_event(robot.position, nodes_expanded, residual, memory)
               break  # restart step-loop with new path
5. logger.stop()
6. logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
```

**`a_star(start, goals, maze_map, h) → list[tuple]`**

Standard A* on the partial map. Edge cost = 1 for passable cells (wall bitmask 0 or no wall in that direction), ∞ for confirmed walls. `nodes_expanded` = total number of nodes removed from the open list during the search.

---

### §8.4 — D*-Lite Algorithm Spec (`src/algorithms/dstar_lite.py`)

**Class:** `DStarLiteExplorer(BaseAlgorithm)`

Implements the D*-Lite algorithm as described in Koenig & Likhachev (2002). Key definitions:

- `g(s)`: current best-known cost from `s` to the nearest goal (backward search)
- `rhs(s)`: one-step lookahead value; `rhs(goal) = 0` for all goal cells
- A node is **inconsistent** if `g(s) ≠ rhs(s)`; inconsistent nodes are managed in a priority queue keyed by `[min(g,rhs) + h(s) + km, min(g,rhs)]`
- `km`: accumulated offset added to all keys when the agent moves (to keep the heuristic consistent without recomputing it)

**`nodes_expanded` definition for logging:** count only states that are **extracted** from the inconsistency queue and whose stored key equals the current key at extraction time (i.e., non-stale extractions). Stale-key re-insertions do not count.

**`memory_occupancy` definition for logging:** count of cells `s` for which `g(s) ≠ ∞` **or** `rhs(s) ≠ ∞` after the call to `ComputeShortestPath`. This quantity is monotonically non-decreasing over the episode.

**Multi-goal handling:** initialise all goal cells with `rhs = 0`. When a goal cell `g_reached` is reached by the agent:
1. Set `rhs(g_reached) = ∞`; `g(g_reached) = ∞`
2. Update all successors of `g_reached` (their rhs values may now increase)
3. Insert newly inconsistent nodes into the priority queue
4. Increment `km` by `h(last_position)` (since the agent's reference point changes)
5. Call `ComputeShortestPath` to route to the next goal

---

### §9 — GUI Design Specifications

All GUI calls (`set_text`, `set_color`, `set_wall`, `clear_color`) must be **no-ops in headless mode** — this is already guaranteed by `SimAPI`'s pass-through implementations. No special branching is needed in algorithm code.

#### A* GUI

| Element | Condition | Action |
|---------|-----------|--------|
| Cell text | Initial state | Empty (`""`) for all cells except start: display `f = h(start)` |
| Cell text | After A* expansion | `set_text(x, y, str(f_value))` for each expanded cell |
| Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
| Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |
| Cell text | Replanning event | Clear all text with `clear_all_text()`; redisplay f-values from new search |

#### D*-Lite GUI

Cell text displays two values on two lines using the format `g:X r:Y`, where `X` and `Y` are the g-value and rhs-value respectively (use `inf` for `∞`). MMS cell text max length is 10 characters; use abbreviated notation if needed (e.g., `g:∞ r:0`).

| Element | Condition | Action |
|---------|-----------|--------|
| Cell text | Initial state | `set_text(x, y, "g:∞ r:∞")` for all cells; goal cells: `"g:∞ r:0"` |
| Cell text | After update | `set_text(x, y, f"g:{g} r:{rhs}")` whenever g or rhs changes |
| Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
| Inconsistent node | Inserted into queue | `set_color(x, y, 'R')` (Dark Red) |
| Consistent node | Removed from queue (expanded) | `set_color(x, y, 'b')` (Blue) |
| Trivial node | `g = rhs = ∞` | `clear_color(x, y)` |
| Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |

---

### §10 — `experiments/` Folder Structure

Replace `scripts/` with `experiments/` throughout the roadmap. Structure:

```
experiments/
├── run_batch.py          # iterates (algo, maze) combinations; headless SimAPI; saves logs
├── assess_difficulty.py  # BFS closed-list ranking; assigns mazes to levels
├── analyze.py            # reads logs from results/logs/; generates all plots
└── settings/
    └── default.py        # default algo list, maze directories, output paths, random seeds
```

All experiment files run in **headless mode** by instantiating `SimAPI` (not `MmsAPI`). `run.py` at the repository root is kept for MMS GUI integration only.

**`run_batch.py` summary output columns:**  
`algorithm | maze | level | goal_reached | total_moves | replanning_events | cumulative_planning_time_s | log_file`
