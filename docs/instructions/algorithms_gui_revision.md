# Algorithms & GUI Simulator Integration — Implementation Blueprint

This is the implementation-ready specification for the GUI/behavioral changes identified from manual MMS-GUI testing of `AStarExplorer` and `DStarLiteExplorer`, and reviewed against the current codebase (`src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, `src/algorithms/base_algorithm.py`, `src/api/mms_api.py`, `src/api/base_api.py`, `src/maze_map.py`, `src/constants.py`, `run.py`, `tests/test_no_stray_print.py`, `tests/test_integration.py`). Every item below settles on one concrete design (file, method, call site) rather than leaving it open — where the prior review left a design question unresolved, this pass makes the call and states the reasoning.

Read section **A** first: it's shared infrastructure that sections **B**–**F** depend on.

---

## A. Shared infrastructure — `src/algorithms/base_algorithm.py`

### A.1 Default-goal flag

Add, in `__init__`, captured from *which argument was supplied* rather than derived from the resulting coordinates (see §B.1 for why):

```python
self._is_default_goal: bool = goals is None and n_random_goals is None
```

Set this **before** `self._goals = self._resolve_goals(...)` runs (order doesn't functionally matter since it only reads the two arguments, but keep it adjacent to that line for readability). No change to `_resolve_goals` itself.

### A.2 Shared stderr event-reporting helper

Add `import sys` at the top of the module, and:

```python
def _report_event(self, message: str) -> None:
    """Print a diagnostic line to stderr (never stdout — stdout is the MMS protocol channel)."""
    print(message, file=sys.stderr)
```

Both `AStarExplorer` and `DStarLiteExplorer` call this; see §E for call sites and message format.

### A.3 Shared traversed-path lookup

No new state needed — `MazeMap` already tracks every visited cell. Add a helper so both algorithms query it the same way:

```python
def _traversed_cells(self, maze_map: MazeMap) -> list[tuple[int, int]]:
    """All cells visited at least once, per MazeMap's visit-count matrix."""
    return [
        (x, y)
        for y in range(maze_map.height)
        for x in range(maze_map.width)
        if maze_map.get_visit_count(x, y) > 0
    ]
```

### A.4 Shared maze-outline display

```python
def _display_maze_outline(self, api: BaseAPI) -> None:
    """Draw the maze's outer perimeter walls. Cosmetic only — see §C.5/§D.6:
    maze_map's own knowledge of the border is still built up by sensing,
    unchanged; this only affects what's drawn on screen before exploration starts."""
    W, H = self._width, self._height
    for x in range(W):
        api.set_wall(x, 0, 's')
        api.set_wall(x, H - 1, 'n')
    for y in range(H):
        api.set_wall(0, y, 'w')
        api.set_wall(W - 1, y, 'e')
```

Call once, as the very first GUI action in both `AStarExplorer.run()` and `DStarLiteExplorer.run()` (before `logger.start()` or right after — order relative to the logger doesn't matter, it must just precede the first `_sense_and_update` call so the perimeter is visible from frame one).

### A.5 Legend content — `LEGEND` class attribute

Design decision (resolving the open question from the prior review): the legend content lives as a class attribute on each algorithm class, not a separate `legend.py` or per-instance dict. Rationale: it's tightly coupled to that class's own display logic (`_gui_show_search`, `_gui_cell_text`, etc.) and should change in lockstep with it; a separate file invites drift. `run.py` only needs the class (not an instance) to read it, since `--algo` selects the class before construction.

Add to `BaseAlgorithm`:

```python
LEGEND: list[tuple[str, str]] = []  # overridden per subclass; each entry is (symbol, meaning)
```

`AStarExplorer.LEGEND` (class attribute, alongside its other class-level definitions):

```python
LEGEND = [
    ("b  (Blue)",        "Expanded (closed list)"),
    ("R  (Dark Red)",     "Open list"),
    ("c  (Cyan)",         "Planned path / traversed path"),
    ("G  (Dark Green)",   "Goal cell, not yet reached"),
    ("g  (Green)",        "Goal cell, reached"),
    ("f-XXX",             "f-value of an expanded/open cell"),
    ("f-XXXh-YYY",        "f-value and h-value (shown only if tie-breaking uses h — see §B.2)"),
]
```

`DStarLiteExplorer.LEGEND`:

```python
LEGEND = [
    ("b  (Blue)",        "Expanded in the current planning cycle"),
    ("B  (Dark Blue)",   "Expanded in an earlier planning cycle"),
    ("R  (Dark Red)",    "Inconsistent (in the priority queue)"),
    ("c  (Cyan)",        "Planned path / traversed path"),
    ("G  (Dark Green)",  "Goal cell, not yet reached"),
    ("g  (Green)",       "Goal cell, reached"),
    ("g-XXXr-YYY",       "g-value and rhs-value of a cell"),
    ("inf",              "Trivial cell: g = rhs = infinity"),
]
```

Every color code above is a valid entry in `src/constants.py`'s `COLORS` dict — no new color codes are introduced anywhere in this blueprint.

### A.6 `run.py` — legend window wiring

In `run.py`, after argument parsing and before (or concurrently with) `algorithm.run()`:

```python
import threading
import tkinter as tk

def _show_legend(algo_name: str, legend: list[tuple[str, str]]) -> None:
    root = tk.Tk()
    root.title(f"{algo_name} — GUI legend")
    for symbol, meaning in legend:
        tk.Label(root, text=symbol, anchor="w", width=14).grid(row=..., column=0, sticky="w")
        tk.Label(root, text=meaning, anchor="w").grid(row=..., column=1, sticky="w")
    root.mainloop()

threading.Thread(
    target=_show_legend, args=(args.algo, _ALGORITHMS[args.algo].LEGEND), daemon=True
).start()
```

`_ALGORITHMS[args.algo].LEGEND` is read off the **class**, so this can run before `algorithm` is constructed. `daemon=True` so the window doesn't block process exit when the MMS run ends.

> **Platform risk to verify empirically, not assumed away:** running Tkinter's `mainloop()` on a background thread while the main thread runs the blocking MMS stdin/stdout loop is the standard pattern for "one blocking loop + one GUI popup", but Tkinter (via Cocoa on macOS) is documented to work most reliably when driven from the main thread. Start with the threaded approach above; if the popup fails to render or misbehaves on macOS during manual testing, fall back to a separate process (`multiprocessing.Process` or a small standalone script launched via `subprocess.Popen`, passing the `LEGEND` list through argv/a temp file) instead of a thread. Since the legend is genuinely static (no updates after creation), either approach is otherwise equivalent.
>
> **Important**: I've tested the threaded approach on macOS and fails, it needs to be a separate process (`multiprocessing.Process`).

---

## B. Behavioral changes

### B.1 Early termination for the default (centre-area) goal

**Files:** `astar.py::run()`, `dstar_lite.py::run()`. **Depends on:** §A.1 (`self._is_default_goal`).

*Why a flag, not a coordinate comparison:* comparing the resolved goal list against a freshly-recomputed centre-area list cannot distinguish "default kicked in" from a user explicitly passing `goals=[(7,7),(7,8),(8,7),(8,8)]` — both produce an identical list, and the latter must still require all 4 reached. Only *which constructor argument was given* disambiguates them, hence the flag captured at construction (§A.1).

**A* (`astar.py::run()`):** in the "goal reached?" block inside the inner path-execution loop —

```python
if robot.position in goal_set:
    remaining_goals.remove(robot.position)
    api.set_color(robot.x, robot.y, 'g')
    if self._is_default_goal:
        remaining_goals.clear()   # single goal *area*: stop at the first cell reached
    break  # replan in next outer iteration (loop exits naturally if remaining_goals is empty)
```

Also apply the same rule to the pre-loop "already at a goal" edge case (`while robot.position in goal_set: ...`) for consistency, even though it's only reachable on implausibly small mazes:

```python
while robot.position in goal_set:
    remaining_goals.remove(robot.position)
    api.set_color(robot.x, robot.y, 'g')
    if self._is_default_goal:
        remaining_goals.clear()
    goal_set = set(remaining_goals)
```

**D\*-Lite (`dstar_lite.py::run()`):** in the goal-reached block —

```python
if robot.position in self._remaining_goal_set:
    self._remaining_goal_set.discard(robot.position)
    api.set_color(robot.x, robot.y, 'g')

    if self._is_default_goal:
        self._remaining_goal_set.clear()

    if not self._remaining_goal_set:
        break
    ...  # existing "remove reached goal / replan to next goal" logic, unchanged
```

No changes needed to `_resolve_goals`, goal-count logging, or `MetricsLogger` — this only changes the loop's own termination condition.

**Test impact:** `TestMultiGoal` in `tests/test_integration.py` uses explicit `GOALS = [(3, 3), (0, 3)]` (not the default), so it's unaffected. `TestDefaultGoal` currently only asserts `forward_moves > 0`; it does not assert all 4 centre cells are visited, so it stays valid either way — but it's worth adding an explicit assertion there that the run stops after exactly one of the 4 centre cells is reached, to actually cover this behavior.

### B.2 A* tie-breaking

**File:** `astar.py::_a_star()`. **Depends on:** nothing; independent of all other items except that its outcome determines whether A*'s h-value display (§C.4) is implemented.

Change the heap's secondary sort key from the neighbour's tentative `g` to its `h`, since `h` is already computed to derive `f` — this is a same-cost substitution:

```python
# Initial entry (was: (h.get(start, INF), 0, start))
open_heap: list[...] = [(h.get(start, INF), h.get(start, INF), start)]
...
# Inside the neighbour relaxation loop (was: heapq.heappush(open_heap, (f_new, tentative_g, neighbour)))
h_neighbour = h.get(neighbour, INF)
heapq.heappush(open_heap, (f_new, h_neighbour, neighbour))
```

Rename the unpacked second element at pop time for clarity, since it's no longer `g`:

```python
# was: f, g, current = heapq.heappop(open_heap)
f, _tie, current = heapq.heappop(open_heap)
```

No other logic changes — `g_score`, `came_from`, and the freespace-assumption relaxation are all driven by the real `tentative_g` variable already local to the loop, untouched by this change.

This does not change A*'s optimality (the heuristic remains admissible on the partial map regardless of tie-break — see `astar.py`'s own docstring) and does not create a new algorithm variant; it resolves an implementation detail the algorithm's definition leaves open, choosing the convention that aligns exploration order with D*-Lite's own `k2 = min(g, rhs)` tie-break (D*-Lite's backward-search `g(s)` is the same quantity as A*'s `h(s)`).

**Test impact:** `test_replanning_count_same_maze` (in `TestReplanningConsistency`) runs on `_single_path_maze_with_wall()`, which has a *unique* shortest path by construction (per its own docstring) — no ties exist on it, so this test is unaffected. Any future test asserting an exact node-expansion or step count on a maze with symmetric/tied paths should be rechecked, since this changes *which* of several equal-cost paths is returned (not whether a shortest path is found).

---

## C. GUI — A\* (`src/algorithms/astar.py`)

### C.1 Value formatting

Replace `_format_f`'s zero-padding with a fixed 3-character slot per value, left-justified, trailing-space-filled — this covers `inf` (already exactly 3 characters) and any numeric value up to 3 digits (999) without special-casing, and 3 digits is the correct budget to keep: on a 16×16 maze (256 cells) a pathological maze can force a shortest path over 100 cells long, so 2 digits is not always enough, despite typical values being much smaller.

```python
@staticmethod
def _format_f(value: int | float) -> str:
    """Format an f-value into a 3-char left-justified slot ('inf' or up to 999)."""
    text = "inf" if value == float('inf') else str(int(value))
    return f"f-{text:<3}"
```

This produces `f-5  `, `f-42 `, `f-123`, `f-inf` (5 chars total; no need to fill further since A* alone only uses one value).

### C.2 h-value display (only if §B.2 is implemented — it is recommended, so treat this as required, not optional)

Extend the format to two 3-char slots (10 chars total, mirroring D*-Lite's `g-XXXr-YYY`):

```python
@staticmethod
def _format_fh(f_value: int | float, h_value: int | float) -> str:
    f_text = "inf" if f_value == float('inf') else str(int(f_value))
    h_text = "inf" if h_value == float('inf') else str(int(h_value))
    return f"f-{f_text:<3}h-{h_text:<3}"
```

`_a_star` already computes `h.get(cell, INF)` for every cell in `f_values`'s domain — extend `_gui_show_search` to also accept the `h` dict (already available at every call site: `h`/`h_new`) and call `_format_fh(f_values[cell], h[cell])` instead of `_format_f(f_values[cell])` for the closed/open cell loops. Update the initial-state display (`f = h(start)`, before the first plan) to use `_format_fh` too, once this is wired in.

### C.3 Goal-color persistence across `clear_all_color()`

**Confirmed gap:** `_gui_show_search` re-applies `'G'` for every cell still in `remaining_goals` after `clear_all_color()`, but never re-applies `'g'` for goals reached in an earlier iteration of the same multi-goal run — so a previously-reached goal's marker is lost at the very next replanning event, today, independent of anything else in this blueprint.

Fix: track reached goals explicitly and redraw them too.

```python
# In __init__ (or lazily at first use in run()):
self._reached_goals: list[tuple[int, int]] = []

# In run(), wherever a goal is currently marked reached:
#   remaining_goals.remove(robot.position); api.set_color(robot.x, robot.y, 'g')
# add immediately after:
self._reached_goals.append(robot.position)
```

Update `_gui_show_search`'s signature to also take `reached_goals: list[tuple[int, int]]` and, after the existing `for gx, gy in remaining_goals: api.set_color(gx, gy, 'G')` loop, add:

```python
for gx, gy in reached_goals:
    api.set_color(gx, gy, 'g')
```

Update both call sites in `run()` to pass `self._reached_goals`.

### C.4 Planned-path and traversed-path highlighting (new)

Neither is displayed today. Extend `_gui_show_search` (called on every plan and replan) to also color:

```python
# planned path: every cell in the just-computed `path`, excluding the robot's current cell
# (already implicitly shown by its own state color) — straightforward addition, e.g.:
for cx, cy in path[1:]:
    api.set_color(cx, cy, 'c')

# traversed path: read from MazeMap via the shared helper (§A.3)
for cx, cy in self._traversed_cells(maze_map):
    api.set_color(cx, cy, 'c')
```

Pass `path` and `maze_map` into `_gui_show_search` (both already available at every call site). Order matters: draw these *after* the closed/open-list colors so cyan wins on any cell that's both traversed/planned and expanded/open, and *before* the goal re-application in §C.3 so goal colors (which must always be visually distinguishable) are drawn last and win over cyan on goal cells.

### C.5 Maze outline (see §A.4)

Call `self._display_maze_outline(api)` once, at the very start of `run()`, before `self._sense_and_update(...)`.

### C.6 Initial-position wall-sensing not displayed (bug fix)

**Confirmed gap:** `run()`'s very first sensing call discards its return value —

```python
# current:
self._sense_and_update(maze_map, robot, api)
```

`DStarLiteExplorer.run()` does not have this gap (it captures and displays `initial_walls`). Fix by applying A*'s own later-loop pattern (`for direction, is_new in new_wall_events: if is_new: api.set_wall(...)`) to this call too:

```python
initial_walls = self._sense_and_update(maze_map, robot, api)
for direction, is_new in initial_walls:
    if is_new:
        api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])
```

(If §E's event reporting is implemented first, fold the `_report_event(...)` call for wall discovery into this same loop — see §E.1.)

---

## D. GUI — D\*-Lite (`src/algorithms/dstar_lite.py`)

### D.1 Value formatting

Same 3-char-slot rule as §C.1, applied to `_fmt3`:

```python
@staticmethod
def _fmt3(value: float) -> str:
    text = "inf" if value == _INF else str(int(value))
    return f"{text:<3}"
```

`_gui_cell_text`'s `f"g-{self._fmt3(g_val)}r-{self._fmt3(r_val)}"` is unchanged (still exactly 10 chars: `g-XXXr-YYY`), except see §D.5 for the trivial-state special case.

### D.2 Introduce `clear_all_color()` + persistent-state redraw on every replanning cycle

**This is new behavior**, not a fix — `DStarLiteExplorer` currently never calls `clear_all_color()` at all; its display is built up incrementally inside `_compute_shortest_path`. Three things must persist through every full clear: goal markers (§below), the traversed path (§D.7), and the "previously expanded" history (§D.3) — none of these are recoverable from `_g`/`_rhs` alone once a clear is introduced, so all three must be tracked explicitly and redrawn.

Add a helper:

```python
def _gui_redraw_persistent_state(self, maze_map: MazeMap, api: BaseAPI) -> None:
    api.clear_all_color()
    for gx, gy in self._remaining_goal_set:
        api.set_color(gx, gy, 'G')
    for gx, gy in self._reached_goals:          # see §D.4
        api.set_color(gx, gy, 'g')
    for cx, cy in self._traversed_cells(maze_map):  # §A.3
        api.set_color(cx, cy, 'c')
    for bx, by in self._previously_expanded:    # §D.3
        api.set_color(bx, by, 'B')
```

Call `self._gui_redraw_persistent_state(maze_map, api)` at the start of **every** `_compute_shortest_path()` call *except the very first one* (nothing to redraw yet — the initial GUI setup loop already handles goal colors directly from a blank canvas). Concretely, add the call immediately before each of the three later call sites: the wall-discovery replanning branch, and the goal-reached "replan to next goal" branch. (These two call sites are a GUI "cycle" boundary regardless of whether the wall-discovery one also counts as a *replanning event* for `MetricsLogger` purposes — that distinction, used for metrics/logging, is unrelated to this cosmetic redraw rule.)

The current cycle's own expansions will then draw `'b'`/`'R'` over this redraw live, as `_compute_shortest_path` runs — no ordering conflict, since that happens after this call returns.

### D.3 "Previously expanded" (`'B'`) vs "expanded this cycle" (`'b'`)

Nothing today tracks *when* a cell was expanded — only whether `g == rhs`. Add:

```python
# In __init__:
self._previously_expanded: set[tuple[int, int]] = set()
self._expanded_this_cycle: set[tuple[int, int]] = set()
```

In `_compute_shortest_path`, reset `self._expanded_this_cycle = set()` at the top of the method, and add `self._expanded_this_cycle.add(u)` in both branches that increment `nodes_expanded` (the overconsistent branch, right after `self._g[u] = self._rhs[u]`, and the underconsistent branch, right after `self._g[u] = _INF`).

After `_compute_shortest_path()` returns (at every call site — initial, wall-discovery replanning, goal-reached), merge into history:

```python
self._previously_expanded |= self._expanded_this_cycle
```

This must happen *after* `_compute_shortest_path` returns and *before* the next call's `_gui_redraw_persistent_state` (§D.2) runs, so next cycle's redraw includes this cycle's cells.

### D.4 Reached-goal color persistence + the trivial-node collision

**Confirmed risk:** a reached goal has `g`/`rhs` set to infinity on arrival, which matches the "trivial node" condition (`g == rhs == ∞`) that `_compute_shortest_path` elsewhere uses to justify `clear_color()` on ordinary cells — so a reached goal, revisited as some other cell's neighbour in a later cycle, risks having its green marker cleared by that unrelated logic.

Fix with the same tracking list needed for §D.2's redraw:

```python
# In __init__:
self._reached_goals: list[tuple[int, int]] = []

# In run()'s goal-reached handling, immediately after api.set_color(robot.x, robot.y, 'g'):
self._reached_goals.append(robot.position)
```

Then guard every `clear_color()` call in `_compute_shortest_path` (there are two: the underconsistent branch's own-cell update, and its neighbour-loop) so it never touches a reached goal:

```python
# was: elif self._g[u] == self._rhs[u] == _INF: self._api.clear_color(ux, uy)
elif self._g[u] == self._rhs[u] == _INF and u not in self._reached_goals:
    self._api.clear_color(ux, uy)
# same guard on the analogous `v` branch
```

(`self._reached_goals` is a short list in practice — one membership check per trivial-node event is negligible; convert to a `set` if profiling ever shows otherwise.)

### D.5 Compact display for the trivial (untouched) state

Only when **both** `g` and `rhs` are infinite (not just one — a cell with one finite and one infinite value is a genuinely informative in-progress state and must keep showing both). This is exactly the same condition already named "trivial node" elsewhere in this file:

```python
def _gui_cell_text(self, x: int, y: int) -> str:
    g_val = self._g[(x, y)]
    r_val = self._rhs[(x, y)]
    if g_val == _INF and r_val == _INF:
        return "inf"
    return f"g-{self._fmt3(g_val)}r-{self._fmt3(r_val)}"
```

### D.6 Missing `set_text` call (display bug fix — root cause already located)

In `_compute_shortest_path`'s **overconsistent** branch, the neighbour-repair step mutates `rhs` directly and never refreshes that cell's text, unlike every other `g`/`rhs` mutation site in the file:

```python
# current (bug): no set_text call after the mutation
new_val = 1.0 + self._g[u]
if new_val < self._rhs[v]:
    self._rhs[v] = new_val
    self._update_vertex(v)
    if v in self._U:
        self._api.set_color(v[0], v[1], 'R')

# fixed:
new_val = 1.0 + self._g[u]
if new_val < self._rhs[v]:
    self._rhs[v] = new_val
    self._api.set_text(v[0], v[1], self._gui_cell_text(v[0], v[1]))
    self._update_vertex(v)
    if v in self._U:
        self._api.set_color(v[0], v[1], 'R')
```

This is the confirmed root cause of a queued/inconsistent cell displaying stale `g-infr-inf`-style text (its search state was always correct; only this one cell's on-screen text was never asked to refresh).

### D.7 Planned-path and traversed-path highlighting (new)

Same rationale as §C.4. D*-Lite's "planned path" is the sequence of `best_next` cells the main loop's argmin step would walk if run to completion from the current `s_start` — since D*-Lite doesn't materialize a full path object (it decides one step at a time from `g`), reconstruct it on demand for display by repeatedly following `argmin over passable neighbours n of (1 + g(n))` from `s_start` until a goal or a dead end (bounded by e.g. `width*height` iterations to guard against an inconsistent state producing a cycle). Color each cell in that reconstructed sequence `'c'`, and traversed cells via `self._traversed_cells(maze_map)` (§A.3) — fold both into `_gui_redraw_persistent_state` (§D.2), same ordering rule as §C.4 (goal colors drawn last, win on goal cells).

### D.8 Maze outline (see §A.4)

Call `self._display_maze_outline(api)` once, at the very start of `run()`, before the initial GUI setup loop.

---

## E. Execution-event reporting (stderr) — cross-cutting

**Files:** `astar.py`, `dstar_lite.py`, `tests/test_no_stray_print.py`. **Depends on:** §A.2.

Concrete line format (adjust wording freely — the data source and destination are what matter):

```
[WALL] (x, y) <n|e|s|w>
[REPLAN] pos=(x, y) expanded=N residual=D cost_ratio=R time_ms=T
```

### E.1 Wall-discovery reporting

At every site in both algorithms that already loops `for direction, is_new in ...: if is_new: api.set_wall(...)` (A*'s initial-sense fix from §C.6, A*'s main-loop wall display, D*-Lite's initial-sense loop, D*-Lite's main-loop wall handling), add alongside the existing `api.set_wall(...)` call:

```python
x, y = robot.position
self._report_event(f"[WALL] ({x}, {y}) {DIR_TO_STR[direction]}")
```

### E.2 Replanning-event reporting

Immediately after each `logger.log_replanning_event(...)` call (both algorithms), read back the fully-computed record via the logger's existing `replanning_events` property rather than recomputing `cost_ratio`/`planning_time_s` independently:

```python
logger.log_replanning_event(robot.position, n_exp, residual, memory)
record = logger.replanning_events[-1]
self._report_event(
    f"[REPLAN] pos=({record['position'][0]}, {record['position'][1]}) "
    f"expanded={record['nodes_expanded']} residual={record['residual_distance']} "
    f"cost_ratio={record['cost_ratio']} time_ms={record['planning_time_s'] * 1000:.2f}"
)
```

### E.3 `tests/test_no_stray_print.py` update

The current AST guard rejects *any* `print(` call. It must be relaxed to allow exactly `print(..., file=sys.stderr)` while still catching everything else (bare `print(...)`, `file=sys.stdout`, or any other stream). Update `_find_print_calls` to inspect each matched `Call` node's keywords:

```python
def _is_stderr_only(node: ast.Call) -> bool:
    for kw in node.keywords:
        if kw.arg == "file":
            return (
                isinstance(kw.value, ast.Attribute)
                and kw.value.attr == "stderr"
                and isinstance(kw.value.value, ast.Name)
                and kw.value.value.id == "sys"
            )
    return False  # no `file=` kwarg → defaults to stdout → still forbidden
```

Only flag a `print(` call as offending when `_is_stderr_only(node)` is `False`. This keeps the guard's core purpose (nothing may write to the MMS `stdout` protocol stream) while allowing the intentional diagnostic output from §E.1/§E.2.

---

## F. Cross-cutting testing checklist

| Change | Existing tests affected | New coverage worth adding |
|---|---|---|
| §B.1 default-goal termination | `TestDefaultGoal` (unaffected, but weak) | Assert the run stops after exactly one of the 4 centre cells is reached |
| §B.2 A* tie-break | `test_replanning_count_same_maze` (unaffected — maze has a unique path) | None required; consider a symmetric-maze test if precise tie-break behavior needs locking down |
| §C.3 / §D.4 goal-color persistence | none (cosmetic, no existing test touches GUI color calls) | Could assert via a fake `BaseAPI` recording `set_color` calls, if GUI-behavior testing is introduced |
| §C.6 initial-wall display | none | Same — GUI-call recording, if introduced |
| §E.1–E.2 stderr reporting | `tests/test_no_stray_print.py` — **must be updated (§E.3) or it will fail immediately** | A test asserting `_report_event` output goes to `stderr` and never `stdout` |
| §D.2/§D.3/§D.6 D*-Lite GUI/state changes | `TestSingleGoal`/`TestMultiGoal`/`TestReplanningConsistency` (unaffected — none assert GUI-call content, only logged metrics and final position) | none required |

No item in this blueprint changes `MetricsLogger`'s schema, `SimAPI`'s behavior, or any planning outcome other than §B.1 (goal-set termination) and §B.2 (tie-break, which only affects *which* equal-cost path is chosen, never path length or goal-reachability) — so the bulk of `tests/test_integration.py` is expected to keep passing unmodified.
