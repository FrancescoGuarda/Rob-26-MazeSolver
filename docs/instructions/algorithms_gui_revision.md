# Algorithms & GUI Simulator Integration — Revision Notes

Observations collected from manual testing of `AStarExplorer` and `DStarLiteExplorer` in the real MMS GUI (via `run.py`), revised for clarity and cross-checked against the current implementation (`src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, `src/algorithms/base_algorithm.py`, `src/api/mms_api.py`, `src/maze_map.py`, `src/constants.py`). This revision does not change the intent of any observation; it clarifies wording, fixes internal inconsistencies, and grounds each remark in the concrete implementation details needed to act on it later. It is a starting point for a subsequent pass that will turn these notes into a concrete implementation blueprint — no code changes are proposed here.

## 1. Behavioral updates

### 1.1 Early termination for the default (centre-area) goal

When no explicit goal is given, `BaseAlgorithm._resolve_goals` defaults to the 4-cell centre area of the maze (e.g. `(7,7)`, `(7,8)`, `(8,7)`, `(8,8)` on a 16×16 maze). Both `AStarExplorer` and `DStarLiteExplorer` currently treat this exactly like any other multi-goal list: they keep exploring until every one of the 4 cells has been visited.

**Requested change:** when — and only when — the default centre-area goal is in effect, the run should terminate as soon as the robot reaches *any one* of the 4 cells, since they represent a single goal *area* rather than 4 separate waypoints to visit in sequence. This must not change behavior when the user supplies an explicit goal or goal list (including if that list happens to be the same 4 cells) — in that case all specified goals must still be visited, as today.

> Note: `_resolve_goals` currently returns a flat list of coordinates regardless of whether they came from the default centre-area logic or from an explicit `goals=[...]` argument — there is presently no way for `run()` to tell the two cases apart after construction. Preserving that distinction (so the "stop at first hit" rule can be scoped to the default case only) is a prerequisite for this change and will need to be addressed in the implementation blueprint.

### 1.2 Execution-event reporting

MMS provides a per-mouse debug/output panel in its GUI, which displays the diagnostic (`stderr`) stream of the running algorithm process. To make algorithm execution easier to follow when watching a run, print the main events to that stream: replanning events (using the data already collected by `MetricsLogger`) and new-wall discoveries (cell, direction).

Example:
```
NEW WALL DISCOVERED: (3, 4, 'n')
REPLANNING EVENT: ...
```
(Illustrative only — do not use this exact wording as the final template.) The goal is output that is simple to read at a glance but still informative.

> **Correction:** the original note specified `stdout`. That would corrupt the MMS protocol — every command exchanged with the simulator is written to `sys.stdout` and read from `sys.stdin` (see `command()` in `src/api/mms_api.py`), so any other write to `stdout` desyncs the connection exactly the way a stray `print()` would. Diagnostic output must go to `sys.stderr` instead (e.g. `print(..., file=sys.stderr)`), which is what the MMS GUI's debug panel actually displays. Note that `tests/test_no_stray_print.py` currently forbids *any* `print(` call under `src/algorithms/` and `src/api/mms_api.py` — that blanket rule will need to be relaxed to allow intentional `stderr`-directed output while still catching accidental `stdout` writes, once this item is implemented.
>
> **Metrics already available** on every replanning event, via `MetricsLogger.log_replanning_event`: `position`, `nodes_expanded`, `residual_distance`, `memory_occupancy`, `planning_time_s`, and the derived `cost_ratio`. Some of the additional figures suggested (open-list size, closed-list size, inconsistent-node count, newly-discovered-wall count) are not uniformly available from both algorithms today:
> - **A*** already computes `open_set`/`closed_set` for every plan/replan (`_a_star`'s return value, added alongside its GUI live-search display), so open-/closed-list sizes are directly available at the point a replanning event is logged.
> - **D\*-Lite** has no equivalent explicit closed list — `_U` is a priority queue of *inconsistent* nodes only, and `_memory_occupancy()` reports the combined count of cells with non-trivial `g` or `rhs`. An "inconsistent-node count" (`_U.size()`) is available, but "closed-list size" has no direct D*-Lite equivalent and would need its own definition.
> - **Newly discovered walls** are already computed on every sensing step — `_sense_and_update` returns an `(direction, is_new)` list — so this is just a matter of surfacing an existing value, for both algorithms.
>
> Because the two algorithms don't expose identical internal structures, a single uniform log-line format may not fit both without either allowing algorithm-specific fields or defining a well-scoped common subset.

## 2. GUI cosmetic enhancements

### 2.1 Common to A* and D*-Lite

1. **Value formatting.** The current implementation zero-pads numeric fields to a fixed width to fit the 10-character cell-text budget (e.g. `g-005r-010`, `f-023`). This is confirmed to be harder to read at a glance than necessary. Replace zero-padding with trailing-space padding instead (e.g. `g-5  r-10 ` rather than `g-005r-010`), while still filling the fixed 10-character budget so the display stays aligned across cells. This convention should apply to whichever text format a given algorithm uses: `f-XXX` for A* (or `f-XXX h-YYY` if the h-value display in §2.2 is implemented), and `g-XXXr-YYY` for D*-Lite.
   > Open detail: the exact per-field width isn't fully settled yet. With the maze sizes used in this project (≤ 16×16), `f`/`g`/`rhs`/`h` values need at most 2 digits, but the `inf` sentinel needs 3 characters (`inf`) — the padding scheme must accommodate both within the same fixed-width field without breaking cell-to-cell alignment. To be resolved in the implementation blueprint.

2. **Static legend window.** Add a legend to `run.py` — e.g. built with `Tkinter` (Python's built-in GUI toolkit) — explaining what each color and text pattern means for the algorithm currently running. It should be a static window (fixed content, no dynamic updates while the run is in progress) that opens automatically whenever `run.py` is launched from the MMS GUI (per the run-command convention documented in `docs/mms.md`), with content that depends on which algorithm (`--algo`) was selected.
   > Open design question, to be resolved in the implementation blueprint: where the legend content should live — a `legend` dict inside each algorithm module, a dedicated `legend.py`, or some other structure — chosen for scalability and to keep color usage consistent across algorithms, rather than for expedience.
   >
   > Note: `src/constants.py` already defines `COLORS: dict[str, str]`, mapping every valid MMS color character to its display name (all 15 supported colors). Every color referenced in this document (`'b'`, `'B'`, `'c'`, `'g'`, `'G'`, `'R'`) is confirmed to be a valid, distinct entry in it, so no new color codes need to be invented. `COLORS` does not, however, carry any algorithm-specific *meaning* yet (e.g. there is no constant tying `'b'` to "expanded cell") — supplying that meaning, consistently, is the actual gap this item needs to close.

3. **Planned-path and traversed-path highlighting.** Color cells on the currently planned path, and cells the robot has already physically visited, both in cyan (`'c'`). This both highlights how a replanning event changes the upcoming path and keeps the robot's past trajectory visible. Neither is displayed by either algorithm today.
   > Note: `MazeMap` already tracks every visited cell via its visit-count matrix (`mark_visit` / `get_visit_count`), so the "traversed path" set requires no new state — it can be read directly from the existing visit matrix rather than tracked separately.

### 2.2 A*

| Element | Condition | Action |
|---------|-----------|--------|
| Cell color | Cell is on the current planned path | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Cell has been traversed by the robot | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Replanning event | `clear_all_color()`, then re-apply `'G'`/`'g'` to all goal cells and `'c'` to all traversed cells |
| Goal cell color | Not yet reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | Reached | `set_color(x, y, 'g')` (Green); must persist through every subsequent `clear_all_color()` call, including at replanning events that happen after this particular goal was reached |

> **Correction:** the original note stated goal color should be "dark if reached, bright if not reached" — the opposite of both the table rows above and the current implementation (`api.set_color(gx, gy, 'G')` for remaining goals, `api.set_color(robot.x, robot.y, 'g')` on arrival, in both `astar.py` and `dstar_lite.py`). The table above reflects the correct, code-consistent convention: Dark Green (`'G'`) until reached, Green (`'g'`) once reached.
>
> **Verified against the current implementation:** `AStarExplorer._gui_show_search` (added together with the GUI live-search display) already re-applies `'G'` for every cell still in `remaining_goals` after each `clear_all_color()`, but does **not** re-apply `'g'` for goals reached in an earlier iteration of a multi-goal run — so a previously-reached goal's green marker is already lost at the next replanning event, independent of anything proposed in this document. This confirms the requirement above is a real, currently-reproducible gap, not just a cosmetic nicety.
>
> The API itself has no selective-clear operation — `clear_all_color()` wipes every cell unconditionally — so preserving goal and traversed-path colors across it always requires the algorithm to explicitly redraw them afterward.

**If A*'s tie-breaking mechanism is changed (see §3.1),** additionally display each expanded/open cell's h-value alongside its f-value, using the same trailing-space-padded style from §2.1 (e.g. `f-12 h-34`), to help visualize the new tie-breaking behavior and the resulting exploration pattern.

### 2.3 D*-Lite

| Element | Condition | Action |
|---------|-----------|--------|
| Cell color | Expanded in an earlier planning/replanning cycle (not the current one) | `set_color(x, y, 'B')` (Dark Blue) |
| Cell color | Expanded in the current planning/replanning cycle | `set_color(x, y, 'b')` (Blue) — already implemented |
| Cell color | On the current planned path | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Traversed by the robot | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Replanning event | `clear_all_color()`, then re-apply `'G'`/`'g'` (goals), `'c'` (traversed path), and `'B'` (previously-expanded history) |
| Goal cell color | Not yet reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | Reached | `set_color(x, y, 'g')` (Green); must persist through every later `clear_all_color()` |

(The planned-path/traversed-path rationale and the goal-color correction from §2.1/§2.2 apply identically here.)

> Distinguishing "previously expanded" (`'B'`, Dark Blue) from "expanded in the current cycle" (`'b'`, Blue) requires the algorithm to remember, across replanning events, which cells were ever colored `'b'` — nothing in the current `_compute_shortest_path` tracks *when* a cell was expanded, only whether `g == rhs` at a given moment.
>
> Unlike A*, `DStarLiteExplorer` currently never calls `clear_all_color()` — its display is built up incrementally, one `set_color` call at a time, inside `_compute_shortest_path`. Introducing a full clear on every replanning event, as this table requires, is a new behavior rather than a fix to existing clearing logic, and — per the point above — means the "previously-expanded", "reached-goal", and "traversed-path" cell sets must all be explicitly tracked and redrawn after every clear, since none of them can be recovered from `_g`/`_rhs` alone once a full clear is introduced.
>
> Goal-color persistence has the same underlying risk noted in §2.2, and D*-Lite has a concrete mechanism that can trigger it: a reached goal has its `g` and `rhs` set to infinity on arrival (see the goal-reached handling in `run()`), which satisfies the "trivial node" condition (`g == rhs == ∞`) that `_compute_shortest_path` elsewhere uses to justify `clear_color()` on ordinary cells. A reached goal later revisited as some other cell's neighbour during a subsequent replanning event therefore risks having its green marker cleared by that unrelated logic — a pre-existing gap, not one introduced by this document's requests.

## 3. Open issues requiring verification

### 3.1 A* — open-list tie-breaking

The open-list tie-break when multiple cells share the same f-value is **not random**, contrary to the original note — Python's `heapq` on the open-heap entries `(f, g, (x, y))` is fully deterministic: ties in `f` are broken by `g` (lower `g` first), and any remaining tie by lexicographic `(x, y)` order. So the exploration order among equal-`f` cells is deterministic, but arbitrary with respect to the maze layout — it depends on cell coordinates, not on proximity to the goal — and this is what makes A*'s expansion order diverge from D*-Lite's, complicating direct comparison between the two.

D*-Lite's tie-break, by contrast, uses `k2 = min(g, rhs)` (see `_calc_key`). D*-Lite's `g(s)` is a *backward*-search quantity — "shortest known cost from `s` to the nearest goal" (per `dstar_lite.py`'s own module docstring) — which is conceptually the same quantity as A*'s heuristic `h(s)` (estimated cost from `s` to the goal), not A*'s `g(s)` (cost from the actual start to `s`). So D*-Lite implicitly tie-breaks by something equivalent to "closer to the goal", while A* currently tie-breaks by cell coordinates. Aligning the two algorithms' exploration behavior would mean giving A* an explicit secondary sort key of `h(s)` (ascending) before falling back to coordinates, so both algorithms prefer expanding cells nearer the goal when `f` ties.

### 3.2 D*-Lite — inconsistent-node display

A cell that is inconsistent (present in the priority queue `_U`) should, by definition, have `g(s) ≠ rhs(s)` — this is exactly the condition `_update_vertex` uses to decide queue membership (`if self._g[u] != self._rhs[u]: insert; elif u in self._U: remove`), so "in queue" and "`g == rhs`" should never coexist once `_update_vertex` has run for that cell. The observed display bug — a queued/inconsistent cell showing `g-infr-inf`, i.e. apparently consistent — is therefore most likely a *timing* artifact between when a cell's color is set (based on queue membership) and when its text was last refreshed (based on the `g`/`rhs` values at that specific point within `_compute_shortest_path`'s multi-step update sequence), rather than the underlying search state itself being incorrect. This should be verified empirically — e.g. by logging `g`/`rhs` immediately before each `set_text`/`set_color` call for the affected cell during a reproducing run — before concluding whether it's a display-only bug or a genuine algorithmic one.
