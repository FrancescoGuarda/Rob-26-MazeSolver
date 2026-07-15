# `src/algorithms` — Maze-Solving Algorithms

Online exploration algorithms that discover the maze one wall at a time (the **freespace assumption**: every unexplored cell is treated as passable until a wall is sensed) and replan whenever a sensed wall invalidates the current plan. All algorithms are coded against `BaseAPI` only, so the exact same code runs unmodified under the MMS GUI (`MmsAPI`) or headless (`SimAPI`).

## Module

| Module | Status | Role |
|--------|--------|------|
| [`base_algorithm.py`](#base_algorithmpy--basealgorithm) | ✓ implemented | Abstract base class shared by all explorers: goal resolution, sensing, movement, heuristics, reset handling |
| [`astar.py`](#astarpy--astarexplorer) | ✓ implemented | `AStarExplorer` — replanning-from-scratch A* |
| [`dstar_lite.py`](#dstar_litepy--dstarliteexplorer) | ✓ implemented | `DStarLiteExplorer` — incremental D*-Lite |
| [`base.py`](#basepy--phase-1-test-script) | ✓ implemented (frozen) | Phase 1 wall-follower smoke test; not part of the `BaseAlgorithm` hierarchy |

Both `AStarExplorer` and `DStarLiteExplorer` extend `BaseAlgorithm` and share the same constructor signature, `run()` entry point, and `MetricsLogger`/JSON-log schema, so they are interchangeable in `run.py` and `experiments/run_batch.py`.

---

## `base_algorithm.py` — `BaseAlgorithm`

Abstract base class (`abc.ABC`) providing everything that does not depend on the specific search algorithm.

```python
BaseAlgorithm(
    api: BaseAPI,
    maze_map: MazeMap,
    robot: Robot,
    logger: MetricsLogger,
    goals: list[tuple[int, int]] | None = None,
    n_random_goals: int | None = None,
    random_seed: int | None = None,
)
```

**Goal resolution** (performed once, at construction time, by `_resolve_goals`):

| Arguments | Behaviour |
|-----------|-----------|
| `goals=[...]` | Use exactly these cells, in the given order (single cell → single goal; multiple → multi-goal, greedy nearest-goal ordering emerges from the BFS heuristic) |
| `n_random_goals=k, random_seed=s` | Generate `k` random free cells (excluding the start) via `random.Random(s)` |
| Neither given | Default to the 4-cell centre area of the maze |

**Abstract method:**

| Method | Description |
|--------|-------------|
| `run() -> None` | Full sense → plan → act → log loop until all goals are reached. Implemented by each subclass. |

**Protected utilities available to subclasses:**

| Method | Description |
|--------|-------------|
| `_sense_and_update(maze_map, robot, api)` | Queries all four walls at the robot's current cell, updates `MazeMap` with any newly discovered wall, and returns `[(Direction, is_new_wall), ...]` |
| `_move_to(next_cell, robot, api, logger, maze_map)` | Turns the robot to face `next_cell` (minimum-turn strategy: 0, 1, or 2 turns), executes one `move_forward`, and updates the logger + visit matrix |
| `_compute_goal_heuristic(maze_map, goals)` | Multi-source BFS **backward** from `goals` on the current partial map; `h[cell]` = shortest known distance from `cell` to the nearest goal. Used by A* (recomputed every replan) |
| `_compute_start_heuristic(maze_map, start)` | BFS **forward** from `start`; used by D*-Lite at initialisation |
| `_check_reset(robot, api)` | Polls `api.was_reset()` once; if pressed, calls `api.ack_reset()` + `robot.reset()` (both default to the maze's fixed origin `(0, 0)` facing North — the same hardcoded convention used by the real MMS mouse and mirrored exactly by `SimAPI`) and returns `True`. Subclasses call this once per loop iteration and, on `True`, discard their in-progress plan/search state and resume from the robot's restored position |
| `_in_bounds(x, y)` | Bounds check against the maze's `(width, height)` |
| `_report_event(message)` | Print a diagnostic line to **stderr** (never stdout — the MMS protocol channel). Used by both explorers for `[WALL] …` / `[REPLAN] …` reporting |
| `_traversed_cells(maze_map)` | All cells with `maze_map.get_visit_count(x, y) > 0`; used to redraw the traversed path (cyan) in the GUI |
| `_display_maze_outline(api)` | Draw the maze's outer perimeter walls once at startup (cosmetic; a no-op under `SimAPI`). Does not affect `MazeMap`'s own wall knowledge |

**Other shared attributes:**

| Name | Description |
|------|-------------|
| `LEGEND` (class attribute) | `list[(symbol, meaning)]`, empty on the base and overridden per subclass. Read off the **class** by `run.py` to build the GUI legend window |
| `_is_default_goal` (instance) | `True` iff neither `goals` nor `n_random_goals` was passed. Gates the "stop at first centre cell" early-termination in each subclass's `run()` |

---

## `astar.py` — `AStarExplorer`

Online A* that **replans from scratch** every time a sensed wall invalidates the current path.

* **Heuristic:** before every plan/replan, `_compute_goal_heuristic` runs a multi-source BFS backward from the remaining goal set on the partial map. Admissible by construction (a partial map can only underestimate true distance), so A* is optimal on current knowledge.
* **Search (`_a_star`)**: standard A* with a binary-heap open list and lazy stale-entry deletion. Returns `(path, nodes_expanded, f_values, open_set, closed_set)` — `f_values` maps every expanded/open cell to its f-value, and `open_set`/`closed_set` are the full cell sets (used both for the GUI display and as the `memory_occupancy` metric, `len(open_set) + len(closed_set)`).
* **Replanning trigger:** after every move, `_sense_and_update` reports newly confirmed walls; if any lies on the remaining planned path (`_path_has_blocked_edge`), A* reruns from the current position and the event is logged via `MetricsLogger.log_replanning_event()`.
* **Tie-breaking:** the open-list heap breaks f-value ties on `h` (already computed to derive `f`), aligning exploration order with D*-Lite's `k2 = min(g, rhs)`. Admissibility — hence optimality — is tie-break-independent.
* **Multi-goal:** on reaching a goal cell it is dropped from the remaining set and the BFS heuristic is recomputed over what's left; nearest-goal ordering falls out of the BFS automatically.
* **Default goal:** when the constructor is given neither an explicit `goals` list nor `n_random_goals` (`_is_default_goal` is `True`), the goal is the maze's 4-cell centre *area* and the run stops as soon as the **first** of those cells is reached. An explicit list — even one identical to the centre cells — is still treated as a real multi-goal request (all cells required).
* **Reset handling:** checked once per step of the path-execution loop via `_check_reset`; on reset, the current path is abandoned and the outer loop replans from the robot's restored position.

**GUI display** (no-ops under `SimAPI`), driven by `_gui_show_search`:

| Cell state | Color | Text |
|------------|-------|------|
| Expanded (closed list) | `'b'` (Blue) | `f-XXXh-YYY` |
| Open list | `'R'` (Dark Red) | `f-XXXh-YYY` |
| Planned / traversed path | `'c'` (Cyan) | — |
| Remaining goal | `'G'` (Dark Green) | — |
| Reached goal | `'g'` (Green) | — |

Each value occupies a fixed 3-character slot (`f-42 `, `f-inf`), sized so 3-digit distances on a 16×16 maze still fit. Colors/text are cleared (`clear_all_color()`/`clear_all_text()`) and fully redrawn on every plan and replan; draw order is expanded/open → paths (cyan) → goals, so goal markers always win on overlap and previously-reached goals (`self._reached_goals`) survive the clear. New walls are shown via `set_wall` as they're sensed, including at the start cell. Wall discoveries and replanning events are also reported to **stderr** (`[WALL] …` / `[REPLAN] …`) via `_report_event`.

---

## `dstar_lite.py` — `DStarLiteExplorer`

Incremental D*-Lite (Koenig & Likhachev, AAAI 2002): only the nodes made inconsistent by a newly discovered wall are reprocessed, rather than rerunning search from scratch.

* **Key definitions:** `g(s)` = current best-known cost from `s` to a goal (backward search); `rhs(s)` = one-step lookahead, `min` over passable neighbours `n` of `1 + g(n)` (goal cells fixed at `rhs = 0`); a node is *inconsistent* when `g(s) ≠ rhs(s)`. `Key(s) = (min(g,rhs) + h(s, s_start) + km, min(g,rhs))`.
* **Heuristic:** Manhattan distance to the robot's current position, consistent and never recomputed; the `km` accumulator absorbs the cost of the robot having moved since the last `ComputeShortestPath`.
* **Priority queue (`_DStarQueue`):** a binary heap with lazy deletion — `insert`/`remove` are O(log n); stale entries are skipped on `top`/`top_key`/`pop` via a sequence-number validity check.
* **`_compute_shortest_path()`:** the core D*-Lite repair loop; processes over/under-consistent nodes until `s_start` is locally consistent and optimal. Returns the number of **non-stale** expansions (the `nodes_expanded` metric — stale-key re-insertions don't count).
* **Replanning trigger:** on each newly confirmed wall, edge cost is set to `+inf` in both directions, `rhs` is recomputed for the affected cells (`_update_rhs`), and `_compute_shortest_path` repairs only the affected region. An event is logged only when `nodes_expanded > 0` (i.e. the plan actually changed).
* **Multi-goal:** all goals start with `rhs = 0`; reaching one sets its `rhs`/`g` to `+inf`, repairs the neighbours that depended on it, bumps `km`, and reruns `ComputeShortestPath` toward the next goal (not counted as a replanning event).
* **Default goal:** same rule as A* — with neither `goals` nor `n_random_goals` given (`_is_default_goal`), the 4-cell centre area is a single goal *area* and the run stops at the first centre cell reached.
* **Reset handling:** checked once per navigation-loop iteration via `_check_reset`. On reset, the entire search state (`_g`, `_rhs`, `_U`, `_km`, and the `_previously_expanded` GUI history) is rebuilt from scratch at the restored start — D*-Lite's incremental repair is only valid relative to the `s_start` it was computed from, so a teleport back to the origin invalidates it outright rather than being a repairable edge-cost change.

**GUI display** (no-ops under `SimAPI`):

| Cell state | Color | Text |
|------------|-------|------|
| Inconsistent (in queue) | `'R'` (Dark Red) | |
| Expanded this planning cycle | `'b'` (Blue) | |
| Expanded in an earlier cycle | `'B'` (Dark Blue) | |
| Planned / traversed path | `'c'` (Cyan) | |
| Trivial (`g = rhs = ∞`) | cleared (`clear_color`) | `inf` |
| Remaining goal | `'G'` (Dark Green) | |
| Reached goal | `'g'` (Green) | |
| Every cell, whenever `g` or `rhs` changes | — | `g-XXXr-YYY` (`_gui_cell_text`) |

Cell text is kept live: `_update_rhs` and every `g`/`rhs` mutation site in `_compute_shortest_path` (plus the goal-reached handler) call `set_text` directly, so displayed values never go stale. A trivial cell (`g = rhs = ∞`) collapses to a bare `inf` instead of `g-infr-inf`. Each planning cycle first clears all colour and redraws the **persistent state** — goal markers, reached goals (`self._reached_goals`), the traversed path, and earlier-cycle expansions (`self._previously_expanded`, drawn `'B'`) — via `_gui_redraw_persistent_state`, then the current cycle paints its live `'b'`/`'R'` colours on top (the initial cycle skips the redraw, drawing from a blank canvas). The two `clear_color()` sites for trivial cells are guarded so a reached goal's green marker is never wiped. New walls are shown via `set_wall` as sensed; wall/replanning events are reported to **stderr** (`[WALL] …` / `[REPLAN] …`) via `_report_event`.

---

## `base.py` — Phase 1 test script

The original wall-following smoke test used in Phase 1 to verify the MMS protocol round-trip before `BaseAlgorithm` existed. It talks to `src/api/mms_api.py`'s module-level functions directly (not `BaseAPI`/`MmsAPI`) and is intentionally left as-is — it is not part of the `AStarExplorer`/`DStarLiteExplorer` hierarchy and is not used by `run.py` or the test suite.

---

## Usage

```python
from src.algorithms.astar import AStarExplorer
from src.algorithms.dstar_lite import DStarLiteExplorer
from src.api.sim_api import SimAPI          # or src.api.mms_api.MmsAPI for a real MMS run
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot

api = SimAPI(wall_matrix, width, height)
maze_map = MazeMap(width, height)
robot = Robot(0, 0)
logger = MetricsLogger("astar", "my_maze")

algo = AStarExplorer(api, maze_map, robot, logger, goals=[(3, 3)])
algo.run()

print(logger.total_replanning_events, logger.forward_moves)
logger.export_json(output_dir="results/logs/")
```

`DStarLiteExplorer` is a drop-in replacement — same constructor signature, same `run()` contract, same log schema — so algorithm comparisons only need to swap the class.

## Testing

`tests/test_integration.py` runs both explorers end-to-end via `SimAPI` on handcrafted mazes (open, single-blocked-path, multi-goal) and validates goal-reaching, log schema, and cross-algorithm replanning-count consistency. `tests/test_no_stray_print.py` statically guards `src/algorithms/*.py` (and `src/api/mms_api.py`) against stray writes to **stdout**, which would corrupt the MMS stdin/stdout protocol in a way invisible to `SimAPI`-based tests. `print(..., file=sys.stderr)` is explicitly allowed (that is the intended diagnostic channel for `_report_event`); any other `print(...)` is rejected.
