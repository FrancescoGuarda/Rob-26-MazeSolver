# Algorithms & GUI Simulator Integration — Revision Notes

Observations collected from manual testing of `AStarExplorer` and `DStarLiteExplorer` in the real MMS GUI (via `run.py`), revised for clarity and cross-checked against the current implementation (`src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, `src/algorithms/base_algorithm.py`, `src/api/mms_api.py`, `src/maze_map.py`, `src/constants.py`). This revision does not change the intent of any observation; it clarifies wording, fixes internal inconsistencies, and grounds each remark in the concrete implementation details needed to act on it later. It is a starting point for a subsequent pass that will turn these notes into a concrete implementation blueprint — no code changes are proposed here.

## 1. Behavioral updates

### 1.1 Early termination for the default (centre-area) goal

When no explicit goal is given, `BaseAlgorithm._resolve_goals` defaults to the 4-cell centre area of the maze (e.g. `(7,7)`, `(7,8)`, `(8,7)`, `(8,8)` on a 16×16 maze). Both `AStarExplorer` and `DStarLiteExplorer` currently treat this exactly like any other multi-goal list: they keep exploring until every one of the 4 cells has been visited.

**Requested change:** when — and only when — the default centre-area goal is in effect, the run should terminate as soon as the robot reaches *any one* of the 4 cells, since they represent a single goal *area* rather than 4 separate waypoints to visit in sequence. This must not change behavior when the user supplies an explicit goal or goal list (including if that list happens to be the same 4 cells) — in that case all specified goals must still be visited, as today.

> Note: `_resolve_goals` currently returns a flat list of coordinates regardless of whether they came from the default centre-area logic or from an explicit `goals=[...]` argument — there is presently no way for `run()` to tell the two cases apart after construction. Preserving that distinction (so the "stop at first hit" rule can be scoped to the default case only) is a prerequisite for this change and will need to be addressed in the implementation blueprint.
>
> **Resolved:** detecting the default case by re-deriving the centre-area coordinates and comparing them against `self._goals` (i.e. "if the goal list equals what the default logic would have produced, treat it as the default") does not work — it cannot be distinguished from a user explicitly passing `goals=[(7,7),(7,8),(8,7),(8,8)]`, which per the requirement above must still require all 4 cells to be visited, not just one. The two cases produce an identical resulting coordinate list and can only be told apart by *which constructor argument was used*, not by the list's contents. The correct, unambiguous approach is to record a flag at construction time — e.g. `self._is_default_goal = goals is None and n_random_goals is None` — captured in `__init__`/`_resolve_goals` before the centre-area fallback runs, so it reflects which code path was taken rather than what the resulting coordinates happen to be.

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
>
> **Resolved:** rather than exposing additional, algorithm-specific internals (open-/closed-list sizes, inconsistent-node count), print exactly the fields both algorithms already record identically in each `replanning_events` entry via `MetricsLogger.log_replanning_event` — `position`, `nodes_expanded`, `residual_distance`, `cost_ratio`, `memory_occupancy`, `planning_time_s` — for example:
> ```json
> {
>   "event_id": 4,
>   "position": [5, 4],
>   "planning_time_s": 0.0023954999924171716,
>   "nodes_expanded": 73,
>   "residual_distance": 16,
>   "cost_ratio": 4.5625,
>   "memory_occupancy": 105
> }
> ```
> This sidesteps the "not uniformly available" concern above entirely, since these fields are already common to both algorithms today — no new instrumentation is needed. The JSON above is a reminder of which values already exist, not the intended print format: dumping the raw dict to `stderr` would not meet the "simple and clear at a glance" goal stated at the top of this section, so the actual line printed per event should be a compact, human-readable rendering of some or all of these fields (e.g. one line per event, plain key–value pairs or a short sentence) rather than a JSON blob — the exact template is still open and explicitly not fixed by the illustrative example earlier in this section.

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

4. **Maze outline display.** Before exploration starts, display the maze's outer perimeter walls with `set_wall(x, y, direction)` (south on every `y=0` cell, north on `y=height-1`, west on `x=0`, east on `x=width-1`, using `self._width`/`self._height`, already available on `BaseAlgorithm`). Mechanically this is straightforward — a fixed loop over the four edges, independent of any sensing.
   > This is worth flagging against a convention used everywhere else in this document: every other display rule (walls, colors, text) is driven strictly by what the robot has actually sensed, mirroring the freespace assumption central to both algorithms (`astar.py`'s own docstring: "every cell whose wall bitmask is 0 is treated as passable... until a wall is confirmed") — and, as implemented today, neither algorithm's *internal* knowledge (`maze_map`) is pre-seeded with the perimeter either; it gets discovered by `wall_front()`/etc. exactly like any interior wall, the first time the robot faces outward at the boundary. Pre-drawing the perimeter is a reasonable convenience regardless — unlike an interior wall, the outer boundary's existence is a structural guarantee of every valid maze, not maze-specific information, so displaying it upfront doesn't leak anything the algorithm doesn't already assume — but it is worth being explicit that this would be a **display-only** convenience:
   > - **Cosmetic only** (what this item appears to ask for, and what fits under "GUI cosmetic enhancements"): draw the perimeter walls on the GUI once at start; `maze_map` and both algorithms' planning continue to discover the border via sensing exactly as today, unchanged. The only downside is a purely visual one — the border appears "solid" on screen slightly before the robot has literally driven along it.

### 2.2 A*

| Element | Condition | Action |
|---------|-----------|--------|
| Cell color | Cell is on the current planned path | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Cell has been traversed by the robot | `set_color(x, y, 'c')` (Cyan) |
| Cell color | Replanning event | `clear_all_color()`, then re-apply `'G'`/`'g'` to all goal cells and `'c'` to all traversed cells |
| Goal cell color | Not yet reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | Reached | `set_color(x, y, 'g')` (Green); must persist through every subsequent `clear_all_color()` call, including at replanning events that happen after this particular goal was reached |

> **Verified against the current implementation:** `AStarExplorer._gui_show_search` (added together with the GUI live-search display) already re-applies `'G'` for every cell still in `remaining_goals` after each `clear_all_color()`, but does **not** re-apply `'g'` for goals reached in an earlier iteration of a multi-goal run — so a previously-reached goal's green marker is already lost at the next replanning event, independent of anything proposed in this document. This confirms the requirement above is a real, currently-reproducible gap, not just a cosmetic nicety.
>
> The API itself has no selective-clear operation — `clear_all_color()` wipes every cell unconditionally — so preserving goal and traversed-path colors across it always requires the algorithm to explicitly redraw them afterward.

**If A*'s tie-breaking mechanism is changed (see §3.1),** additionally display each expanded/open cell's h-value alongside its f-value, using the same trailing-space-padded style from §2.1 (e.g. `f-12 h-34`), to help visualize the new tie-breaking behavior and the resulting exploration pattern.

**Initial-position wall-sensing not displayed.** The initial-position walls are sensed but not shown in the GUI.

> **Confirmed by direct comparison of the two `run()` methods — this is a real, reproducible gap, not a perception issue.** `AStarExplorer.run()` senses the start cell (`self._sense_and_update(maze_map, robot, api)`, called right after `logger.start()`) but discards the returned `(direction, is_new)` list entirely — the walls it just discovered are recorded into `maze_map` for planning purposes, but never passed to `api.set_wall(...)`, so they never appear on screen. `DStarLiteExplorer.run()` does not have this gap: it captures the same call's return value (`initial_walls = self._sense_and_update(maze_map, robot, api)`) and loops over it, calling `api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])` for every newly-confirmed wall — which is exactly the same display pattern `AStarExplorer` already uses for *every wall sensed after the first move* (its main loop's `for direction, is_new in new_wall_events: if is_new: api.set_wall(...)`). So the fix is simply to apply A*'s own existing pattern to its one remaining un-displayed sensing call, matching what D*-Lite already does.

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

> Distinguishing "previously expanded" (`'B'`, Dark Blue) from "expanded in the current cycle" (`'b'`, Blue) requires the algorithm to remember, across replanning events, which cells were ever colored `'b'` — nothing in the current `_compute_shortest_path` tracks *when* a cell was expanded, only whether `g == rhs` at a given moment. Concretely, this means maintaining an explicit "previously expanded" set (every cell colored `'b'` in an earlier cycle) so it can be redrawn as `'B'` right after each replanning event's `clear_all_color()`, before the current cycle's own `'b'` expansions are drawn on top — the same "clear, then explicitly redraw what should persist" pattern already required for goals and the traversed path.
>
> Unlike A*, `DStarLiteExplorer` currently never calls `clear_all_color()` — its display is built up incrementally, one `set_color` call at a time, inside `_compute_shortest_path`. Introducing a full clear on every replanning event, as this table requires, is a new behavior rather than a fix to existing clearing logic, and — per the point above — means the "previously-expanded", "reached-goal", and "traversed-path" cell sets must all be explicitly tracked and redrawn after every clear, since none of them can be recovered from `_g`/`_rhs` alone once a full clear is introduced.
>
> Goal-color persistence has the same underlying risk noted in §2.2, and D*-Lite has a concrete mechanism that can trigger it: a reached goal has its `g` and `rhs` set to infinity on arrival (see the goal-reached handling in `run()`), which satisfies the "trivial node" condition (`g == rhs == ∞`) that `_compute_shortest_path` elsewhere uses to justify `clear_color()` on ordinary cells. A reached goal later revisited as some other cell's neighbour during a subsequent replanning event therefore risks having its green marker cleared by that unrelated logic — a pre-existing gap, not one introduced by this document's requests.

**Compact display for the trivial (untouched) state.** When a cell's `g` *and* `rhs` are both still infinite, display a single compact `inf` marker rather than the redundant two-field `g-infr-inf`, to avoid cluttering the display with a pair of values that are known to be equal and uninformative. Cosmetic only — no functional/planning change — and should be documented in the algorithm's legend (§2.1, item 2) alongside the normal `g-XXXr-YYY` pattern.

> **Scope clarification:** read literally, "when `g` *or* `rhs` is infinite" would also compact cells where only *one* of the two is infinite (e.g. `g` finite, `rhs` still infinite) — a common, informative in-progress state during `ComputeShortestPath`, distinct from the untouched trivial state. Collapsing that case to a bare `inf` would discard real information (which of the two is known and which isn't) and should **not** be included; the compaction should trigger only on `g == rhs == ∞` — i.e. exactly the same condition the D*-Lite cosmetic table (§2.3) already calls the "trivial node" for coloring purposes (`clear_color(x, y)`). This reuses an existing, named condition rather than introducing a new one, and pairs naturally with it: a trivial node gets both its color cleared and its text compacted.
>
> Independent of, and compatible with, the §3.2 fix: that item is about a *missing* `set_text` call on one mutation site; this item is about *what string* `_gui_cell_text` produces once it is called for a cell in the `g == rhs == ∞` state. Both touch the same rendering path but don't conflict.

## 3. Open issues requiring verification

### 3.1 A* — open-list tie-breaking

The open-list tie-break when multiple cells share the same f-value is **not random**, contrary to the original note — Python's `heapq` on the open-heap entries `(f, g, (x, y))` is fully deterministic: ties in `f` are broken by `g` (lower `g` first), and any remaining tie by lexicographic `(x, y)` order. So the exploration order among equal-`f` cells is deterministic, but arbitrary with respect to the maze layout — it depends on cell coordinates, not on proximity to the goal — and this is what makes A*'s expansion order diverge from D*-Lite's, complicating direct comparison between the two.

D*-Lite's tie-break, by contrast, uses `k2 = min(g, rhs)` (see `_calc_key`). D*-Lite's `g(s)` is a *backward*-search quantity — "shortest known cost from `s` to the nearest goal" (per `dstar_lite.py`'s own module docstring) — which is conceptually the same quantity as A*'s heuristic `h(s)` (estimated cost from `s` to the goal), not A*'s `g(s)` (cost from the actual start to `s`). So D*-Lite implicitly tie-breaks by something equivalent to "closer to the goal", while A* currently tie-breaks by cell coordinates. Aligning the two algorithms' exploration behavior would mean giving A* an explicit secondary sort key of `h(s)` (ascending) before falling back to coordinates, so both algorithms prefer expanding cells nearer the goal when `f` ties.

**Resolved:** this is achievable by replacing the heap entries' existing secondary field — `tentative_g` — with the neighbour's already-computed `h` value, i.e. `(f, h, (x, y))` instead of today's `(f, g, (x, y))`; no third key needs to be added, since `h.get(neighbour, INF)` is already computed once per neighbour to derive `f` itself, so this is a same-cost substitution, not new work.

This does **not** turn A* into a different algorithm, and does not need to be left alone to preserve "canonical" behavior. The standard definition of A* leaves tie-breaking among equal-`f` nodes completely unspecified — any deterministic secondary criterion is a legitimate implementation choice, and choosing one does not affect optimality: since the heuristic is admissible on the partial (freespace-assumption) map by construction (a partial map can only underestimate true distance — see `astar.py`'s own module docstring), every node popped in `f`-order is still guaranteed optimal regardless of how ties among equal-`f` nodes are broken. Preferring lower `h` (i.e., cells nearer the goal) on a tie is, in fact, a well-known and commonly used A* tie-breaking convention — it does not create a new algorithm variant, it resolves an implementation detail the algorithm's definition leaves open, and it happens to be the choice that aligns A*'s exploration order with D*-Lite's.

One practical follow-up, not a reason to avoid the change: because this alters *which* of several equal-cost paths gets returned on a tie (not whether a shortest path is found), any existing test that asserts an exact step count, node-expansion count, or replanning-event count on a maze with symmetric/tied paths should be re-checked once this is implemented, in case it happened to depend on the old coordinate-order tie-break.

### 3.2 D*-Lite — inconsistent-node display

A cell that is inconsistent (present in the priority queue `_U`) should, by definition, have `g(s) ≠ rhs(s)` — this is exactly the condition `_update_vertex` uses to decide queue membership (`if self._g[u] != self._rhs[u]: insert; elif u in self._U: remove`), so "in queue" and "`g == rhs`" should never coexist once `_update_vertex` has run for that cell.

**Resolved, by code inspection (no empirical/log-based check needed to confirm this):** this is a genuine, deterministically-reproducible display bug — not a timing race — with an exact, locatable cause. Every place `_compute_shortest_path` mutates a cell's `g` or `rhs` is expected to immediately refresh that cell's displayed text, and every site does so correctly **except one**: in the *overconsistent* branch's neighbour-repair loop,

```python
new_val = 1.0 + self._g[u]
if new_val < self._rhs[v]:
    self._rhs[v] = new_val
    self._update_vertex(v)
    if v in self._U:
        self._api.set_color(v[0], v[1], 'R')  # GUI: inconsistent
```

`self._rhs[v]` is mutated directly — bypassing `_update_rhs()` (which does call `set_text` after recomputing `rhs`, per §1.2's related fix) — and no `set_text(v, ...)` call follows: only `set_color` runs, and only when `v` actually entered the queue. Contrast this with the *underconsistent* branch's equivalent neighbour loop, which calls `_update_rhs(v)` (text-refreshing) before `_update_vertex(v)`, or with `_update_rhs()`'s own two branches and the goal-reached handler, which each refresh text as part of the same mutation. This one branch is the outlier.

The consequence: when this branch fires for a neighbour `v` that hasn't had its text refreshed since the run started, `v` gets correctly colored `'R'` (its `rhs` really did just change and really is now inconsistent with `g`), while its on-screen text still shows whatever it displayed last — which, for a cell touched for the first time, is its untouched initial-state text, `g-infr-inf`. So the cell *is* genuinely inconsistent internally (the search state is correct); only its **displayed text** is stale, because this one mutation site never asked for it to be redrawn. Fixing it is a matter of adding the same `set_text(v[0], v[1], self._gui_cell_text(v[0], v[1]))` call used everywhere else in this function, right after `self._rhs[v] = new_val`.