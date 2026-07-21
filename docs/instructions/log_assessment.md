# Metrics Tracking Subsystem — Implementation Blueprint

## 1. Purpose and scope

This document is the authoritative specification for the metrics-tracking subsystem that backs the algorithm-comparison analysis in `docs/implementation_roadmap.md` (Phases 6–7). It defines, for `src/metrics/logger.py`, `src/algorithms/base_algorithm.py`, `src/algorithms/astar.py`, and `src/algorithms/dstar_lite.py`:

* the exact JSON schema every run must export,
* what each field means and how it must be computed so that A\* and D\*-Lite records are comparable,
* which computations may and may not fall inside the timed `planning_time_s` window,
* the on-disk formatting contract for exported logs, and
* the preconditions the maze corpus must satisfy for the schema's guarantees to hold.

It is written to be implemented directly and to remain the reference for maintaining this subsystem afterward — not a record of a prior review. `experiments/01_experiment.py` runs `AStarExplorer` and `DStarLiteExplorer` headlessly (`SimAPI`) over a maze list and exports one JSON log per run via `MetricsLogger.export_json()` to `results/logs/<goal-count>/<algorithm>/`; this document governs everything that ends up in that file.

**Non-negotiable constraint:** the roadmap's Phase 6 analysis (`implementation_roadmap.md:166-169`) puts A\* and D\*-Lite records on the *same graph* — a scatter of per-event cost vs. residual distance, and overlaid cumulative-planning-time curves. Every rule below exists to make that overlay valid: the two algorithms' log records must measure the same quantities, under the same definitions, excluding the same classes of non-algorithmic overhead.

---

## 2. Components and data flow

| Component | Role |
|---|---|
| `src/metrics/logger.py` (`MetricsLogger`) | Accumulates counters/timers during a run; owns the export schema and file layout. Opaque to algorithm semantics — never inspects *why* a value is what it is. |
| `src/algorithms/base_algorithm.py` (`BaseAlgorithm`) | Shared sensing, movement, heuristic (`_compute_goal_heuristic`, `_compute_heuristic`), and diagnostic-reporting (`_report_walls`, `_report_replan`) infrastructure used by both explorers. |
| `src/algorithms/astar.py` / `dstar_lite.py` | Own the timed windows: decide exactly what runs between `logger.start_plan_timer()` and the point planning is considered "done" for a given replanning event. |
| `src/api/sim_api.py` (`SimAPI`) | Headless API used for every batch/analysis run. Every display method (`set_wall`, `set_color`, `set_text`, `clear_*`) is `pass` (`sim_api.py:107-129`) — the callee is free, but the call and any argument construction (e.g. string formatting) still execute in Python. This is the source of every "GUI overhead" concern in §5. |
| `experiments/01_experiment.py` | Batch driver. Currently the default `HEURISTIC = "manhattan"` (`experiments/01_experiment.py:60`) — this is the configuration under which §4.1's canonical `residual_distance` definition matters most, since A\*'s own planning heuristic is not wall-aware in this mode. |

---

## 3. Canonical log schema

Every field below is REQUIRED in every exported log (`logger.py:254-273`), regardless of algorithm.

### 3.1 Top-level fields

| Field | Type | Definition |
|---|---|---|
| `algorithm` | string | Logger's `algo` name, e.g. `"astar"`, `"dstar_lite"` — must match `run.py`'s `_ALGORITHMS` keys so headless and MMS-GUI runs of the same algorithm share a log bucket. |
| `maze` | string | Maze base name (no extension). |
| `goal_count` | int \| null | Number of goals requested for the run (`BaseAlgorithm.goal_count`); drives the export subdirectory (§3.4). `null` if `set_goal_count`/`set_scenario` was never called. |
| `timestamp` | string | `YYYYMMDD_HHMMSS`, wall-clock at export time. |
| `total_moves` | int | `forward_moves + turns`. |
| `forward_moves` | int | Count of `move_forward()` calls (`base_algorithm.py:301-303`). |
| `turns` | int | Count of `turn_left()`/`turn_right()` calls (`base_algorithm.py:293-298`). |
| `distinct_cells_visited` | int | Count of cells with `visit_matrix[y][x] > 0`. |
| `total_visits` | int | Sum of all `visit_matrix` entries (includes revisits). |
| `execution_time_s` | float | Wall-clock seconds between `logger.start()` and `logger.stop()`. **Not a clean algorithmic-cost proxy — see §4.5.** |
| `wall_matrix` | `list[list[int]]` | Final known wall bitmask per cell, `wall_matrix[y][x]`. |
| `visit_matrix` | `list[list[int]]` | Visit count per cell, `visit_matrix[y][x]`. |
| `total_replanning_events` | int | `len(replanning_events)`. Trigger criteria differ structurally between algorithms — see §4.4. |
| `cumulative_planning_time_s` | float | Sum of `planning_time_s` over all events. |
| `cumulative_nodes_expanded` | int | Sum of `nodes_expanded` over all events. |
| `replanning_events` | `list[object]` | See §3.2. |
| `scenario` | object \| null | See §3.3. `null` if `set_scenario()` was never called. |

### 3.2 `replanning_events[]` record

One entry per call to `logger.log_replanning_event(...)` (`logger.py:110-141`):

| Field | Type | Definition |
|---|---|---|
| `event_id` | int | 0-based, insertion order. |
| `position` | `[x, y]` | Robot position at the moment the event was logged. |
| `planning_time_s` | float | Elapsed time for the timed window — see §5 for exactly what may be inside it. |
| `nodes_expanded` | int | Count of search-graph expansions for this event. **Unit differs by algorithm — see §4.2.** |
| `residual_distance` | int | Wall-aware BFS distance from `position` to the nearest remaining goal, **for both algorithms, unconditionally** (§4.1). `-1` sentinel when the true value is `+∞` (unreachable under current knowledge). |
| `cost_ratio` | float \| null | `nodes_expanded / residual_distance`, or `null` when `residual_distance` is not a finite positive number (§7.1) — covers `residual_distance == 0` (at-goal replan) and `residual_distance == -1`/`+∞` (unreachable) alike. |
| `memory_occupancy` | int | Live search-structure size. **Semantics differ by algorithm by design — see §4.3.** |

### 3.3 `scenario` object

Set via `logger.set_scenario(maze_file, k, goals)` (`logger.py:77-96`):

```json
{
  "maze_file": "mazes/txt/88us.txt",
  "k": 2,
  "goals": [
    {"cell": [7, 7], "detour": 1.83},
    {"cell": [3, 12], "detour": 2.05}
  ]
}
```

`detour` is the detour index from `src/goal_placement.py` (`d_BFS / d_Manhattan` from a reference cell; see that module's docstring) — the logger stores it opaquely and must not interpret it.

### 3.4 Export path and file layout

`export_json(output_dir="results/logs/")` (`logger.py:226-278`) writes to:

```
<output_dir>/<goal-count-bucket>/<safe-algorithm-name>/<safe-algorithm-name>_<maze>_<timestamp>[_<n>].json
```

* **Goal-count bucket** (`_goal_dirname`, `logger.py:218-224`): `"one_goal"`, `"two_goals"`, … `"ten_goals"` for `goal_count` 1–10 (spelled out); `"<n>_goals"` for `goal_count` > 10; `"unknown_goals"` if `goal_count` is `None` or < 1.
* **Algorithm name**: sanitized to `[A-Za-z0-9_-]`, lowercased, falling back to `"unknown"` if empty.
* **Collision handling**: a numeric `_<n>` suffix is appended (starting at 1) if the timestamped filename already exists — second-granularity timestamps collide when a batch runs the same algorithm/maze/goal-count combination twice within one second; this must never silently overwrite a prior run.

This layout and its collision behavior are locked in by `tests/test_metrics_logger.py::test_export_json_goal_count_directory`, `test_export_json_without_goal_count_uses_unknown_bucket`, `test_set_scenario_sets_goal_count`, and `test_export_json_does_not_overwrite_same_second_run` — any change to the layout must update these tests in the same change.

---

## 4. Metric definitions and cross-algorithm comparability

### 4.1 `residual_distance` / `cost_ratio` — canonical definition

**Rule:** `residual_distance` is *always* the wall-aware BFS distance from the current robot position to the nearest remaining goal — i.e., what `BaseAlgorithm._compute_goal_heuristic()` (`base_algorithm.py:306-345`) computes — for both algorithms, in every run, regardless of the run's `heuristic` setting. There is exactly one definition; the schema does not record "which definition produced this value."

**Why this is the only acceptable rule:** `experiments/01_experiment.py`'s current default is `HEURISTIC = "manhattan"` (`experiments/01_experiment.py:60`). Under that setting, A\*'s own search heuristic (`_compute_heuristic`, dispatched at `base_algorithm.py:358-366`) is Manhattan-valued — a straight-line lower bound that ignores walls — while D\*-Lite always calls `_compute_goal_heuristic()` for its residual regardless of `self._heuristic` (D\*-Lite's *planning* heuristic is a separate, fixed Manhattan-to-`s_start` function, `_h()` in `dstar_lite.py:208-210`, per `base_algorithm.py:58-62`'s own comment). If A\*'s residual were left as its own search heuristic's value, `residual_distance` would silently be two different quantities across the two algorithms' log records whenever `heuristic="manhattan"` — the current default — corrupting the x-axis of the roadmap's planned overlay scatter plot (`implementation_roadmap.md:167`) and the denominator of `cost_ratio` for every A\* event.

**Required implementation, per algorithm:**

* **D\*-Lite** — do **not** call `_compute_goal_heuristic()` at all for this purpose. `self._g[self._s_start]` (equivalently `self._rhs[self._s_start]`, equal once `_compute_shortest_path()` converges, by its own termination condition at `dstar_lite.py:351`) is already the wall-aware shortest-known distance from the current position to the nearest goal — the exact quantity `_compute_goal_heuristic()` would recompute from scratch, but read in O(1) instead of an O(W×H) BFS. This substitution is valid because `self._s_start` is provably equal to `robot.position` at the point the replanning-event block runs: `dstar_lite.py:526` sets it immediately after the move, and nothing reassigns it before the `if has_new:` block (`dstar_lite.py:625`) — both paths that could (reset handling, goal-reached handling) `continue` first (`dstar_lite.py:548`, `dstar_lite.py:619`). Use:
  ```python
  residual = self._g[self._s_start]
  ```
* **A\*** — call `_compute_goal_heuristic()` explicitly for the log field whenever `self._heuristic != "min_path"` (i.e., decouple "heuristic guiding the search" from "heuristic used to report `residual_distance`"). When `self._heuristic == "min_path"`, `_compute_heuristic()` already dispatches to `_compute_goal_heuristic()` (`base_algorithm.py:367`), so the just-computed `h_new` *is* the wall-aware BFS map — reuse it instead of recomputing:
  ```python
  residual_map = h_new if self._heuristic == "min_path" else self._compute_goal_heuristic(maze_map, remaining_goals)
  residual = residual_map.get(robot.position, 0)
  ```
  This call belongs **after** `logger.stop_plan_timer()` (§5.3) — it is an analysis-only computation the search itself never consumes.

### 4.2 `nodes_expanded` — same field name, comparable unit, different mechanism

A\* performs a from-scratch replan each event: a node is settled (moved open→closed) at most once per `_a_star()` call, so its `nodes_expanded` counts *distinct* nodes. D\*-Lite can legitimately pop and process the *same* graph node twice within one `_compute_shortest_path()` call — once underconsistent (`g → ∞`), once later overconsistent (`g` lowered again) if its consistency flips within the same cycle (`dstar_lite.py:338-422`; both branches increment `nodes_expanded`, at `dstar_lite.py:369` and `dstar_lite.py:396`). This is correct, standard D\*-Lite behavior (documented in `dstar_lite.py:28-31`), not a bug.

**Rule:** keep counting operations (current behavior) on both sides — do not switch either algorithm to counting distinct nodes, and do not add a second field. A revisit costs the same shape of work as a first visit (mutate `g`/`rhs`, scan up to four neighbours, push vertex updates — `dstar_lite.py:365-421`), so settle-operation counting is the metric that faithfully tracks computational effort; distinct-node counting would *undercount* real work. `nodes_expanded` is comparable in spirit (both count units of search effort) but not identical in unit (distinct nodes vs. settle operations) — any analysis script comparing raw per-event `nodes_expanded` between algorithms must account for this, and this document is the reference for that caveat.

### 4.3 `memory_occupancy` — different semantics by design, not a defect

* **A\*** (`astar.py:360`): `len(open_set) + len(closed_set)` from just the current replanning search — resets every replan.
* **D\*-Lite** (`dstar_lite.py:424-428`): `|{s : g(s) ≠ ∞ ∨ rhs(s) ≠ ∞}|` — accumulates across the whole run, monotonically non-decreasing (documented in `dstar_lite.py:33-34`).

This asymmetry is intentional and matches the roadmap's own expected signature — "bounded (resetting) for A\*, monotonically increasing for D\*-Lite" (`implementation_roadmap.md:169`). No change required; recorded here so it is not mistaken for an inconsistency during implementation or analysis.

### 4.4 `total_replanning_events` — trigger criteria differ structurally

A\* logs an event only when a newly-discovered wall lies on the *currently planned path* — a path-based filter (`_path_has_blocked_edge`, `astar.py:181-193`, gated at `astar.py:350-352`). D\*-Lite runs `_compute_shortest_path()` whenever *any* new wall is discovered this step, but only logs an event when that replan performed at least one node expansion — a result-based filter (`n_exp > 0`, `dstar_lite.py:625,630`). Both are legitimate given each algorithm's own structure, but `total_replanning_events` is not guaranteed to fire on equivalent conditions across algorithms for the same sequence of wall discoveries. Analysis code must not assume event counts are directly comparable move-for-move; this is an interpretation note, not something to reconcile in the logger.

### 4.5 `execution_time_s` vs. `planning_time_s` — roles

* **`planning_time_s` / `cumulative_planning_time_s`** is the metric the Phase 6 analysis depends on (per-event cost vs. residual scatter, cumulative-time curve). It MUST measure genuine planning/search work only, symmetrically across algorithms. §5 is normative for this field.
* **`execution_time_s`** wraps the *entire* `run()` call: sensing, movement, stderr diagnostics (`_report_walls`/`_report_replan` — real `print()` I/O, a genuine cost but not an "algorithm" one), and every GUI/display API call made during the run, including calls that are symmetric between algorithms (`_display_maze_outline`, once per run in both: `astar.py:265`, `dstar_lite.py:444`; `_gui_show_termination`, once per run in both) and calls that are not: D\*-Lite runs an unconditional `O(W×H)` GUI-init sweep at `dstar_lite.py:464-469` (`set_text`/`set_color` over every cell before a single wall is sensed) that has no A\*-side equivalent (A\*'s init touches only `O(goals)` cells plus one `set_text` call, `astar.py:280-287`). **`execution_time_s` is therefore treated as a coarse, secondary/diagnostic wall-clock figure — never as a cross-algorithm-comparable proxy for algorithmic cost, and never as a substitute for `planning_time_s` in the Phase 6 analysis.** No code change is prescribed for this field; the requirement is documentational — do not build an analysis that treats `execution_time_s` as apples-to-apples between A\* and D\*-Lite.

---

## 5. Timing contract

### 5.1 Governing principle

> Any computation whose only purpose is producing a log field or a GUI display update — rather than being consumed by the algorithm's own state transitions (A\*'s `g_score`/`open_set`/`closed_set`; D\*-Lite's `self._g`/`self._rhs`/`self._U`) — MUST execute outside the `start_plan_timer()` → `log_replanning_event()` window, regardless of its individual cost.

This is the rule A\*'s existing structure already follows for `_gui_show_search()`/`_report_replan()` (both run after `log_replanning_event()`, `astar.py:364-369`). It is now made explicit and applied uniformly to D\*-Lite, where it currently is not followed (see §5.4).

### 5.2 `MetricsLogger` timer API

`start_plan_timer()`/`log_replanning_event()` currently share one mutable field (`self._plan_timer_start`, `logger.py:44`) with the timer read at the moment `log_replanning_event()` is called. That collapses "end of genuine search work" and "point at which the record is appended" into a single moment — which is exactly what D\*-Lite's residual/memory/display computations need to *not* be true (§4.1, §5.4). The API must be extended with an explicit stop step:

```python
class MetricsLogger:
    def __init__(self, ...):
        ...
        self._plan_timer_start: float | None = None
        self._pending_planning_time: float | None = None   # NEW

    def start_plan_timer(self) -> None:
        """Record the start time of a planning / replanning call."""
        self._plan_timer_start = time.monotonic()

    def stop_plan_timer(self) -> float:                     # NEW
        """End the timed window now. log_replanning_event() will use this
        elapsed time instead of measuring at call time, so any diagnostic or
        display work performed between this call and log_replanning_event()
        is excluded from planning_time_s. Returns the elapsed seconds.
        """
        elapsed = (
            time.monotonic() - self._plan_timer_start
            if self._plan_timer_start is not None else 0.0
        )
        self._pending_planning_time = elapsed
        self._plan_timer_start = None
        return elapsed

    def log_replanning_event(self, position, nodes_expanded, residual_distance, memory_occupancy) -> None:
        if self._pending_planning_time is not None:
            planning_time = self._pending_planning_time
        else:
            # Backward-compatible path: no explicit stop_plan_timer() call —
            # measure now, as before (used by existing unit tests that call
            # start_plan_timer() -> log_replanning_event() directly).
            planning_time = (
                time.monotonic() - self._plan_timer_start
                if self._plan_timer_start is not None else 0.0
            )
        cost_ratio = (...)  # see §7.1
        self._replanning_events.append({...})
        self._plan_timer_start = None
        self._pending_planning_time = None
```

**Canonical five-step sequence** for both algorithms' replanning-event call sites:

```
start_plan_timer() → [pure search] → stop_plan_timer() → [diagnostics/display] → log_replanning_event()
```

The older three-step form (`start_plan_timer()` → `[search]` → `log_replanning_event()`, with nothing to exclude) remains valid and is preserved for API robustness and the existing unit tests in `tests/test_metrics_logger.py` that exercise the timer directly — no test changes are required by this addition.

**Reentrancy note (informational, unchanged from prior behavior):** neither a second `start_plan_timer()` before a stop (silently overwrites) nor a `log_replanning_event()`/`stop_plan_timer()` call with no preceding `start_plan_timer()` (silently returns `0.0`) raises. Both call sites specified below always pair these calls correctly, so this remains a documented structural note, not a live bug.

### 5.3 A\* — required call structure

Current window (`astar.py:354-364`): `start_plan_timer()` → `_compute_heuristic()` → `_a_star()` → `log_replanning_event()`. Both `_compute_heuristic()`/`_a_star()` (`astar.py:93-179`) contain zero API/GUI calls, and `_report_replan()`/`_gui_show_search()` already run after logging (`astar.py:364-369`) — this window is already clean **except** that `residual`/`memory` are read at `astar.py:359-360` using data available at that point, which is fine, but `residual` must switch to the canonical definition from §4.1 when `self._heuristic != "min_path"`, and that extra BFS call must move outside the window:

```python
# ---- Replanning event ----
logger.start_plan_timer()
h_new = self._compute_heuristic(maze_map, remaining_goals)
new_path, n_exp, f_values, open_set, closed_set = self._a_star(
    robot.position, set(remaining_goals), maze_map, h_new
)
logger.stop_plan_timer()

memory = len(open_set) + len(closed_set)
residual_map = h_new if self._heuristic == "min_path" else self._compute_goal_heuristic(maze_map, remaining_goals)
residual = residual_map.get(robot.position, 0)
logger.log_replanning_event(robot.position, n_exp, residual, memory)
self._report_replan(logger.replanning_events[-1])
self._gui_show_search(
    f_values, h_new, open_set, closed_set,
    remaining_goals, self._reached_goals, new_path, api,
)
path = new_path
step_idx = 1
continue
```

### 5.4 D\*-Lite — required call structure

Current window (`dstar_lite.py:625-635`) bundles four components inside `start_plan_timer()` → `log_replanning_event()`:

1. **Embedded display text formatting inside the search itself** — `_compute_shortest_path()` (`dstar_lite.py:338-422`) and the `_update_rhs()` helper it calls (`dstar_lite.py:216-235`) each call `self._api.set_text(...)`, which calls `_gui_cell_text()` → `_fmt3()` (real string formatting), at every node touched (`dstar_lite.py:220, 235, 372, 388, 399`). This is pure display work baked *inside* the timed function, proportional to `nodes_expanded`.
2. **`_gui_show_search()`** (`dstar_lite.py:299-336`, called at `dstar_lite.py:628`) — pure display: full board recolor plus an `O(path length)` planned-path reconstruction, feeding nothing back into `self._g`/`self._rhs`/`self._U`.
3. **The residual-distance BFS** (`_compute_goal_heuristic()`, called at `dstar_lite.py:631`) — a full `O(W×H)` multi-source BFS run on every single replanning event, independent of how much actually changed, solely to populate a log field.
4. **`_memory_occupancy()`** (`dstar_lite.py:424-428`, called at `dstar_lite.py:633`) — scans the live `self._g`/`self._rhs` dicts, `O(state size)`, diagnostic-only.

Component 3 is the most severe: D\*-Lite's entire value proposition is that an incremental repair after one new wall is *cheap*; a roughly maze-size-constant `O(W×H)` cost added to every event, however small the real repair, can dominate the measurement and flatten out the "near-flat cumulative planning time" signature the roadmap expects to observe (`implementation_roadmap.md:168`).

**Required changes**, applied together:

* **Component 3 is eliminated, not merely relocated** — per §4.1, use `self._g[self._s_start]` (O(1)) instead of calling `_compute_goal_heuristic()` at all.
* **Component 4** moves after `stop_plan_timer()` — a pure call-ordering change; its cost (O(state size), diagnostic-only, never queried by D\*-Lite's own planning) is otherwise unaffected. An O(1) incrementally-maintained counter is a valid future optimization but is not required for correctness — only its placement outside the window matters.
* **Component 2** moves after `stop_plan_timer()` — pure call-ordering change, mirroring where `_gui_show_search()`/`_report_replan()` already sit in A\*'s call order.
* **Component 1** requires restructuring `_compute_shortest_path()`/`_update_rhs()` so the (cheap) fact "this cell's displayed text is stale" is recorded during the search, but the (real-cost) string formatting and `set_text()` call happen only once, after the window closes:
  * Add `self._dirty_text_cells: set[tuple[int, int]] = set()` (reset at the top of `_compute_shortest_path()`, alongside `self._expanded_this_cycle`).
  * Replace every `self._api.set_text(u[0], u[1], self._gui_cell_text(u[0], u[1]))` inside `_update_rhs()` (`dstar_lite.py:220, 235`) and inside `_compute_shortest_path()`'s two branches (`dstar_lite.py:372, 399`) with `self._dirty_text_cells.add(u)` (an O(1) set insert — no string formatting, no API call, during the search).
  * Add `_flush_gui_text(self, api)`: iterates `self._dirty_text_cells`, calls `api.set_text(x, y, self._gui_cell_text(x, y))` for each, clears the set.
  * `_update_rhs()` is also invoked directly from `run()` outside any timed block (initial wall handling `dstar_lite.py:484,487`; per-move wall handling `dstar_lite.py:570,575`; goal-reached repair `dstar_lite.py:612`). Each of those call sites, and every existing call site of `_compute_shortest_path()` **other than** the replanning-event path (initial plan `dstar_lite.py:494`, reset handling `dstar_lite.py:545`, goal-reached handling `dstar_lite.py:616`), must call `self._flush_gui_text(api)` immediately afterward so visual behavior outside the timed path is unchanged.

Resulting replanning-event block:

```python
# ---- Replanning event (only when new walls actually changed the map) ----
if has_new:
    logger.start_plan_timer()
    n_exp = self._compute_shortest_path()      # now only self._dirty_text_cells.add(u) — no string formatting
    logger.stop_plan_timer()

    self._flush_gui_text(api)                   # component 1 — outside window
    self._gui_show_search(api)                  # component 2 — outside window
    self._previously_expanded |= self._expanded_this_cycle
    if n_exp > 0:
        residual = self._g[self._s_start]        # component 3 — eliminated (O(1), §4.1)
        memory = self._memory_occupancy()         # component 4 — outside window
        logger.log_replanning_event(robot.position, n_exp, int(residual), memory)
        self._report_replan(logger.replanning_events[-1])
```

With this structure, D\*-Lite's `planning_time_s` measures `_compute_shortest_path()` alone — its in-loop cost now reduced to O(1) set-inserts for text bookkeeping — the same class of quantity A\*'s window measures via `_a_star()` alone (§5.3). This is the structural symmetry the Phase 6 analysis requires.

### 5.5 Non-goals for this section

`BaseAPI`/`SimAPI` are not modified by any of the above — every display and diagnostic call still executes exactly as before, only its position relative to the timer moves (or, for component 3, is replaced by an O(1)-equivalent read). `execution_time_s` is explicitly out of scope for timing correction (§4.5) — the GUI-init asymmetry documented there is left as a known, documented property of that field, not something this section's relocations address.

---

## 6. Output formatting spec

### 6.1 Problem

`export_json()` writes with `json.dump(payload, fh, indent=2)` (`logger.py:276`). Python's stdlib `json` applies `indent` uniformly across the whole object graph — there is no keyword to indent the outer object while keeping specific leaf arrays on one line. Every element of `wall_matrix`, `visit_matrix`, and every 2-element coordinate pair (`replanning_events[].position`, `scenario.goals[].cell`) is exploded one value per line. On a real 16×16/10-event/2-goal sample log this is 718 lines / 7,983 bytes versus 144 lines / 4,479 bytes row/pair-compacted (79.9% / 43.9% reduction) — purely cosmetic (no value is corrupted), but it scales with maze area and event/goal count, and matters at the batch scale `experiments/01_experiment.py` runs at (5 mazes × 2 algorithms = 10 files per invocation, multiplied further by any `-k/--k-goals` sweep).

### 6.2 Required implementation

A structural post-processing pass over the payload's object graph — not a text-level regex sweep, which could in principle match a digit sequence inside an unrelated string value introduced by future schema growth (there is none today, but nothing prevents it). The implementation replaces target arrays with unique placeholder tokens *before* serialization (so matching is by actual object identity/position in the structure, never by scanning dumped text for patterns), dumps normally, then substitutes the placeholders for their compact form:

```python
def _compact_payload_arrays(payload: dict) -> tuple[dict, dict[str, str]]:
    """Replace every wall_matrix/visit_matrix row and every position/cell
    coordinate pair with a unique placeholder token. Returns (payload_copy,
    tokens), where tokens maps each placeholder to the compact JSON text that
    must replace it after json.dumps(payload_copy, indent=2).

    Structural: driven by (parent key, value shape) during a walk of the
    actual payload object, not by scanning already-serialized text — cannot
    misfire on an unrelated digit sequence inside some future string field.
    """
    tokens: dict[str, str] = {}

    def token_for(value) -> str:
        tok = f"@@COMPACT_{len(tokens)}@@"
        tokens[tok] = json.dumps(value)
        return tok

    def walk(node, key):
        if key in ("wall_matrix", "visit_matrix") and isinstance(node, list):
            return [token_for(row) for row in node]
        if key in ("position", "cell") and isinstance(node, list):
            return token_for(node)
        if isinstance(node, dict):
            return {k: walk(v, k) for k, v in node.items()}
        if isinstance(node, list):
            return [walk(v, key) for v in node]
        return node

    return walk(payload, None), tokens
```

Used in `export_json()` in place of the current `with open(...) as fh: json.dump(payload, fh, indent=2)`:

```python
compacted, tokens = _compact_payload_arrays(payload)
text = json.dumps(compacted, indent=2)
for tok, compact_json in tokens.items():
    text = text.replace(f'"{tok}"', compact_json)
with open(filepath, 'w') as fh:
    fh.write(text)
```

Result: `wall_matrix`/`visit_matrix` keep one row per line, each row a single-line array; `position`/`cell` collapse to single-line 2-element arrays; every other field keeps normal `indent=2` formatting. This runs entirely after `logger.stop()`/`set_matrices()`/all `log_replanning_event()` calls have already captured their data — it cannot affect any timed window in §5, and must never be moved earlier in `export_json()` for that reason.

`@@COMPACT_<n>@@` tokens are synthetic and monotonically generated per call, so they cannot collide with real payload content (maze names, scenario data, etc.).

---

## 7. Correctness guards

### 7.1 `cost_ratio` must be `null`, not `0.0`, when the true distance is unknown

`log_replanning_event()` (`logger.py:110-141`) special-cases `residual_distance == float('inf')` to the `-1` storage sentinel (avoiding `int(inf)`'s `OverflowError`), but the `cost_ratio` guard, `residual_distance and residual_distance > 0`, does **not** exclude `inf` — `inf` is truthy and `> 0`, so `cost_ratio` computes as `nodes_expanded / float('inf') == 0.0`. A `0.0` ratio reads as "essentially free, at the goal" — the opposite of the true situation, "distance unknown/unreachable." `tests/test_metrics_logger.py::test_cost_ratio_zero_residual` already locks in the design intent that a degenerate residual must produce `cost_ratio is None`; the `inf` case falls through that intent by omission.

**Required fix:**

```python
cost_ratio: float | None = (
    nodes_expanded / residual_distance
    if isinstance(residual_distance, (int, float))
    and 0 < residual_distance < float('inf')
    else None
)
```

This mirrors the already-correct `residual_distance == 0` handling and is an O(1) comparison — its placement has no bearing on §5.

---

## 8. Preconditions and invariants

### 8.1 Maze connectivity guarantees `residual_distance` is always finite

§7.1's guard is a defensive fix for a case that is real as a matter of code correctness but is **not currently reachable** on this project's maze corpus, verified directly:

* `tools/filter_connected.py` flood-fills from `(0, 0)` over the ground-truth wall matrix and rejects any maze where not every cell is reachable, and (in `--strict` mode) any maze with wall-reciprocity mismatches.
* Running `python3 tools/filter_connected.py --dry-run --strict` against the live corpus confirms all 55 files under `mazes/txt/` pass both checks (0 rejected).
* The partial map used by `_compute_goal_heuristic()`/`_compute_heuristic()`/`self._g` only ever *removes* passable edges as walls are confirmed during a run — its edge set is a superset of the ground-truth maze's at every point. Full ground-truth connectivity therefore guarantees the partial map is also fully connected at every point during exploration.

Consequently, for the current maze corpus, `residual_distance` provably cannot be `inf` for a reachable robot position at any point in a run. `mazes/README.md:45` already lists "No inaccessible locations (all cells reachable from start)" under "Micromouse competition requirements," but does not connect that requirement to the fact that the metrics code silently depends on it.

**Required change to `mazes/README.md`:** append to the existing requirement line (`mazes/README.md:45`) that this precondition is what keeps `residual_distance` finite in the exported metrics — e.g.:

> - No inaccessible locations (all cells reachable from start) — this is also a hard dependency of the metrics subsystem: `residual_distance` in exported logs is only guaranteed finite because every maze in the corpus satisfies full connectivity (enforced by `tools/filter_connected.py`).

**Any maze added to the corpus outside `tools/filter_connected.py` voids this guarantee** and must be run through that tool (or an equivalent connectivity check) before use. §7.1's guard exists precisely as defense against that case and must be kept regardless of the corpus's current guarantee.

---

## 9. Implementation checklist

In dependency order:

1. **`src/metrics/logger.py`**
   - [ ] Add `self._pending_planning_time` field; add `stop_plan_timer()` (§5.2).
   - [ ] Update `log_replanning_event()` to prefer `self._pending_planning_time` when set, falling back to the existing at-call-time measurement otherwise (§5.2).
   - [ ] Fix the `cost_ratio` guard to exclude `inf` (§7.1).
   - [ ] Add `_compact_payload_arrays()` and route `export_json()`'s write through it (§6.2).
2. **`src/algorithms/astar.py`**
   - [ ] Insert `logger.stop_plan_timer()` immediately after `_a_star()` returns in the replanning-event block; move `residual`/`memory` computation after it, switching `residual` to the canonical definition (§4.1, §5.3).
3. **`src/algorithms/dstar_lite.py`**
   - [ ] Add `self._dirty_text_cells` set; replace the four embedded `set_text` calls in `_update_rhs()`/`_compute_shortest_path()` with `self._dirty_text_cells.add(u)`; add `_flush_gui_text(api)` (§5.4).
   - [ ] Call `_flush_gui_text(api)` after every existing `_compute_shortest_path()`/`_update_rhs()` call site that is *not* the replanning-event path (initial plan, reset handling, goal-reached handling, initial/per-move wall-update sites) so non-timed visual behavior is unchanged.
   - [ ] Restructure the replanning-event block per §5.4: `stop_plan_timer()` right after `_compute_shortest_path()`; flush text and redraw after; replace the `_compute_goal_heuristic()` residual call with `self._g[self._s_start]`; move `_memory_occupancy()` after the stop.
4. **`mazes/README.md`**
   - [ ] Append the metrics-dependency note to the connectivity requirement (§8.1).
5. **No changes required** to `src/api/base_api.py`, `src/api/sim_api.py`, `src/api/mms_api.py`, or `base_algorithm.py`'s shared utilities.

---

## 10. Verification plan

* **Existing tests must keep passing unmodified**: `tests/test_metrics_logger.py` — in particular `test_cost_ratio_zero_residual` (still `None` for `residual_distance == 0`), `test_plan_timer_reset_after_event` (three-step form still valid), `test_export_json_goal_count_directory` / `_without_goal_count_uses_unknown_bucket` / `test_set_scenario_sets_goal_count` (export path unchanged), `test_export_json_does_not_overwrite_same_second_run`.
* **New unit tests to add** to `tests/test_metrics_logger.py`:
  - `stop_plan_timer()` followed by `log_replanning_event()` produces the same `planning_time_s` as the elapsed time at the `stop_plan_timer()` call (not later, even if `log_replanning_event()` is delayed by intervening work).
  - `cost_ratio is None` when `residual_distance == float('inf')` (regression test for §7.1) and when `residual_distance == -1` is passed directly (defensive: the guard must key off the value's finiteness, not the sentinel).
  - `export_json()` output: `wall_matrix`/`visit_matrix` rows and `position`/`cell` pairs are single-line in the written file; the parsed JSON is unchanged in value from before compacting (round-trip equality against the pre-compacting payload).
* **Algorithm-level checks** (manual or scripted, against a real maze via `SimAPI`):
  - D\*-Lite: `residual_distance` recorded via `self._g[self._s_start]` matches `_compute_goal_heuristic()`'s value for `robot.position` at the same point in the run (equivalence check, then remove the BFS call from the hot path once confirmed).
  - A\*: with `heuristic="manhattan"`, confirm `residual_distance` in the exported log is the wall-aware BFS distance, not the Manhattan value used for search.
  - Both algorithms: confirm `_gui_show_search`/`_report_replan`/text-flush output is bit-for-bit identical to pre-change behavior when run against `MmsAPI` (or a display-call-recording stub), since §5's relocations must not alter *what* is displayed — only *when*, relative to the timer.
* **Corpus check**: re-run `python3 tools/filter_connected.py --dry-run --strict` whenever a maze is added to `mazes/txt/` before including it in a batch run (§8.1).
* **End-to-end**: run `experiments/01_experiment.py` on the full `MAZES` list after all changes land; confirm no unhandled exceptions, all runs still reach their goals, and spot-check one exported D\*-Lite log's `cumulative_planning_time_s` against a pre-change log for the same maze/seed to confirm it dropped (expected, since components 1/3/4 no longer contaminate it) without any value field changing.
