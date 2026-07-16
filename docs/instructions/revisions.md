# Revisions — Implementation Blueprint

This is the implementation-ready specification for the second round of remarks gathered from manual MMS-GUI testing of `AStarExplorer` and `DStarLiteExplorer`, building on top of `algorithms_gui_revision.md` (fully implemented) and reviewed against the current codebase (`src/algorithms/base_algorithm.py`, `src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, `src/api/base_api.py`, `src/api/sim_api.py`, `src/constants.py`, `src/metrics/logger.py`, `run.py`, `docs/mms.md`, `src/algorithms/README.md`). Every item settles on one concrete design (file, method, call site) and states the reasoning behind it; the one item left genuinely open (§A.1's D*-Lite scope) is flagged explicitly rather than guessed at, since resolving it requires re-verifying D*-Lite's incremental-correctness proof, not just reading the current code.

Organized into **A. Operative** (CLI flags, stderr diagnostics) and **B. GUI** (MMS display). Read §B.2 before §B.3 — the D*-Lite `gui_show_search` rewrite in §B.3 subsumes §B.2's D*-Lite half.

---

## A. Operative changes

### A.1 `--heuristic` flag (`min_path` | `manhattan`)

**Files:** `src/algorithms/base_algorithm.py`, `src/algorithms/astar.py`, `run.py`, `docs/mms.md`, `src/algorithms/README.md`.

**Current state:**
- `AStarExplorer` uses `BaseAlgorithm._compute_goal_heuristic(maze_map, goals)` exclusively as its planning heuristic (`astar.py:281,301,353`) — a multi-source BFS *backward* from the goal set, wall-aware (respects confirmed walls; freespace assumption elsewhere). This is what `min_path` refers to.
- `DStarLiteExplorer` computes its own heuristic inline via `_h(s)` (`dstar_lite.py:200-202`): a hardcoded Manhattan distance from `s` to `self._s_start`, ignoring walls entirely. It also calls `_compute_goal_heuristic` once (`dstar_lite.py:617`), but only to derive the `residual_distance` metric for `[REPLAN]` logging — **not** as its planning heuristic. That call site must not be disturbed by this change.
- `BaseAlgorithm._compute_start_heuristic(maze_map, start)` (BFS *forward* from a start cell, also wall-aware) exists but is dead code: a repo-wide search confirms it is never called anywhere. Its docstring — and `src/algorithms/README.md`'s copy of the claim — states it is "used by D*-Lite at initialisation." That is stale and must be corrected regardless of the decision below.

**Decision — scope of the flag:** implement `--heuristic` for **A* only** in this pass. D*-Lite's heuristic is not a stylistic choice: `Key(s) = (min(g,rhs) + h(s, s_start) + km, min(g,rhs))` requires `h` to be consistent with respect to the fixed reference point `s_start` (incrementally corrected via `km` as the robot moves), which is a different mathematical role from A*'s admissible-to-goal `h`. Swapping in a wall-aware `min_path`-to-`s_start` heuristic (built on the otherwise-unused `_compute_start_heuristic`) is plausible in principle, but recomputing a wall-aware heuristic on a schedule that doesn't match D*-Lite's incremental-repair assumptions risks breaking its consistency proof — that needs to be verified against the algorithm's correctness argument before it's wired in, not assumed safe by analogy with A*. Treat D*-Lite support as an explicit follow-up, not part of this pass; `--heuristic manhattan` passed with `--algo dstar_lite` is accepted (D*-Lite already behaves this way) but has no effect, and `--heuristic min_path` with `--algo dstar_lite` is likewise accepted and is also a no-op (document this explicitly rather than leaving it to be discovered).

**Implementation:**

1. `BaseAlgorithm.__init__` gains a new parameter, stored unvalidated (validity is enforced by `run.py`'s `argparse choices=`, and `BaseAlgorithm` is trusted-internal-code otherwise):
   ```python
   heuristic: str = "min_path",
   ...
   self._heuristic: str = heuristic
   ```
2. Add a new dispatch method to `BaseAlgorithm` (does not replace `_compute_goal_heuristic`, which stays as-is — it's still needed verbatim for D*-Lite's `residual_distance` logging call):
   ```python
   def _compute_heuristic(
       self, maze_map: MazeMap, goals: list[tuple[int, int]],
   ) -> dict[tuple[int, int], int | float]:
       """Dispatch on self._heuristic: wall-aware BFS ('min_path', default)
       or straight-line Manhattan distance to the nearest goal ('manhattan')."""
       if self._heuristic == "manhattan":
           return {
               (x, y): min(
                   (abs(x - gx) + abs(y - gy) for gx, gy in goals),
                   default=float('inf'),
               )
               for y in range(maze_map.height)
               for x in range(maze_map.width)
           }
       return self._compute_goal_heuristic(maze_map, goals)
   ```
3. `AStarExplorer.run()`: replace the three call sites at `astar.py:281,301,353` (`self._compute_goal_heuristic(maze_map, remaining_goals)`) with `self._compute_heuristic(maze_map, remaining_goals)`.
4. `run.py::_parse_args`: add
   ```python
   parser.add_argument(
       "--heuristic", choices=["min_path", "manhattan"], default="min_path",
       help="Heuristic for planning (astar only; ignored by dstar_lite).",
   )
   ```
   and pass `heuristic=args.heuristic` in the algorithm constructor call in `main()`.
5. Fix the stale `_compute_start_heuristic` docstring in `base_algorithm.py` (remove the "used by D*-Lite at initialisation" claim; state it is currently unused) and the corresponding row in `src/algorithms/README.md`.
6. Update `docs/mms.md`'s Run Command Format template and bullet list to include `[--heuristic min_path|manhattan]`, noting it only affects `--algo astar`.
7. Update `src/algorithms/README.md`'s `BaseAlgorithm` constructor signature table (new `heuristic` row) and add a `_compute_heuristic` row next to the existing `_compute_goal_heuristic`/`_compute_start_heuristic` rows.

**Validation:**
- Existing `tests/test_integration.py` must keep passing unmodified — default `heuristic="min_path"` reproduces current behavior exactly (no call site is exercised differently).
- New test: run `AStarExplorer` with `heuristic="manhattan"` on `_single_path_maze_with_wall()` (already used by `TestReplanningConsistency`, has internal walls where BFS and Manhattan distance diverge) and assert the goal is still reached (Manhattan is always ≤ true wall-aware distance, so admissibility — hence optimality — is preserved) and, optionally, that `nodes_expanded` is `>=` the `min_path` run's on the same maze (a less-informed heuristic should never expand fewer nodes).
- Manual: `python run.py --algo astar --heuristic manhattan` against the MMS GUI; confirm the `f-XXXh-YYY` cell text now shows straight-line `h` values.

### A.2 `--no-log` flag

**Files:** `run.py`.

**Current state:** `run.py::main()` unconditionally calls `logger.export_json(output_dir=args.output_dir)` after `algorithm.run()` returns (`run.py:112`); there is no way to suppress it.

**Implementation:**
```python
parser.add_argument("--no-log", action="store_true", help="Skip writing the JSON metrics log.")
```
```python
if not args.no_log:
    logger.export_json(output_dir=args.output_dir)
```
A polished `help=` string is low priority — MMS invokes `run.py` as a fixed shell command, so the user never sees `argparse`'s `--help` output in normal use. The authoritative documentation channel is `docs/mms.md`'s Run Command Format section, which **must** list `--no-log` and note that `--output-dir` is irrelevant when it's set. stderr diagnostics (`_report_event`, `base_algorithm.py:119-126`) are untouched by this flag: they write directly to `sys.stderr` and have no dependency on `MetricsLogger`/`export_json`.

**Validation:**
- New lightweight test (`tests/test_run_cli.py` or similar): monkeypatch `sys.argv` and call `run._parse_args()` directly (no `MmsAPI`/GUI dependency) to assert `--no-log` defaults to `False` and sets `True` when passed, alongside the existing `--goal`/`--n-goals` mutual-exclusivity check already implied by `_parse_args`.
- Manual: `python run.py --algo astar --no-log` against MMS or a stub; confirm no new file appears under `results/logs/` while `[WALL]`/`[REPLAN]` stderr lines still appear.

### A.3 Stderr diagnostics: consolidate `[WALL]`, round `[REPLAN]`

**Files:** `src/algorithms/base_algorithm.py`, `src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`. **Depends on:** `_report_event` (already implemented, `algorithms_gui_revision.md` §E).

**a. One `[WALL]` line per sensing event, not per wall.**

Today, each newly-discovered wall produces its own line (`astar.py:268-272,330-335`; `dstar_lite.py:461-478,588-595`). Replace with a single consolidated line reporting all four directions' current status at that cell, using the existing `DIR_TO_STR` letters (`n`/`e`/`s`/`w`) — chosen over a directional-glyph alternative because it reuses `constants.py`'s existing direction-to-string mapping instead of introducing a second lookup table — in `Direction`'s canonical `N, E, S, W` order (matching `_sense_and_update`'s own read order, `base_algorithm.py:183-188`), with `_` marking an absent wall:
```
[WALL] (0, 0) _ e s w
```
Add a shared helper to `BaseAlgorithm`:
```python
def _report_walls(self, maze_map: MazeMap, x: int, y: int) -> None:
    """One consolidated stderr line for all four walls at (x, y), reporting
    the cell's complete now-known wall status (not just this pass's deltas)."""
    parts = [
        DIR_TO_STR[direction] if maze_map.has_wall(x, y, direction) else '_'
        for direction in Direction
    ]
    self._report_event(f"[WALL] ({x}, {y}) {' '.join(parts)}")
```
Call it, guarded by the existing "at least one new wall this pass" condition (so verbosity actually decreases rather than growing — a line is still skipped when nothing new was found), at all four existing call sites, replacing their per-direction reporting loop while leaving each direction's `api.set_wall(...)` display call untouched (only the stderr line consolidates):
```python
if any(is_new for _, is_new in wall_events):
    self._report_walls(maze_map, x, y)
```

**b. Round `[REPLAN]` fields to two decimals; fix the `None` edge case.**

`time_ms` is **already** formatted with `:.2f` at both `_report_event` call sites (`astar.py:368`, `dstar_lite.py:627`) — no change needed there. The only gap is `cost_ratio`, printed today at full `float` precision. `MetricsLogger.log_replanning_event` sets `cost_ratio` to `None` whenever `residual_distance` is `0` or falsy (`src/metrics/logger.py:95-98`, e.g. a replan triggered while already at zero residual distance from the goal) — a bare `f"{value:.2f}"` on `None` raises `TypeError`, so this must be special-cased.

Both explorers currently duplicate the same `[REPLAN]` f-string block; consolidate into one shared `BaseAlgorithm` helper (removes the duplication and guarantees both algorithms format identically):
```python
def _report_replan(self, record: dict) -> None:
    cost_ratio = record['cost_ratio']
    cost_ratio_str = f"{cost_ratio:.2f}" if cost_ratio is not None else "None"
    self._report_event(
        f"[REPLAN] pos=({record['position'][0]}, {record['position'][1]}) "
        f"expanded={record['nodes_expanded']} "
        f"residual={record['residual_distance']} "
        f"cost_ratio={cost_ratio_str} "
        f"time_ms={record['planning_time_s'] * 1000:.2f}"
    )
```
Replace both existing inline blocks (`astar.py:362-369`, `dstar_lite.py:621-628`) with `self._report_replan(logger.replanning_events[-1])`.

**Validation:**
- `tests/test_no_stray_print.py` requires no change — both new helpers still route through `_report_event`, i.e. `print(..., file=sys.stderr)`.
- New unit-level check (can be a small `SimAPI`-backed integration test or a direct call): construct a `MetricsLogger`, log a replanning event with `residual_distance=0` (→ `cost_ratio=None`), and assert `_report_replan` produces `cost_ratio=None` in the output instead of raising.
- Manual: run either explorer under MMS or `SimAPI` on a maze with multiple walls around one cell; confirm exactly one `[WALL]` line appears per sensing event with all four directions shown, and `[REPLAN]` lines show 2-decimal `cost_ratio`.

---

## B. GUI changes

### B.1 Legend color palette, swatch rendering, and window de-duplication

**Files:** `src/constants.py`, `run.py`.

**Palette — resolved to match `constants.py`'s existing 15 supported colors** (the previously-proposed 17-code palette included `O`/Dark-Orange and `V`/Dark-Violet, neither of which exists in `COLORS` today, and neither of which is referenced by any current `LEGEND` entry in `AStarExplorer`/`DStarLiteExplorer` — so there is nothing to reconcile against the real simulator; simply add hex values for the 15 codes already defined):

```python
# src/constants.py, alongside COLORS
COLOR_HEX: dict[str, str] = {
    'k': '#000000', 'w': '#FFFFFF',
    'b': '#0000BB', 'B': '#000036',
    'a': '#B3B3B3', 'A': '#1A1A1A',
    'c': '#006867', 'C': '#003434',
    'g': '#00B600', 'G': '#004F00',
    'o': '#BF6100',
    'r': '#DF0000', 'R': '#550000',
    'y': '#B3B300', 'Y': '#333300',
}
```
(15 entries, one per `COLORS` key — `'O'`/`'V'` intentionally omitted; add them only if a future revision actually introduces a `LEGEND` entry that needs them.)

**Rendering** — replace `run.py::_show_legend`'s plain-text `tk.Label` rendering with color swatches, styled like `src/algorithms/base.py::show_color_legend()` (`tk.Canvas` rectangle filled with the entry's hex color, `("Arial", 12, "bold")` title, `("Arial", 10)` entries):

```python
def _show_legend(algo_name: str, legend: list[tuple[str, str]]) -> None:
    import tkinter as tk
    from src.constants import COLOR_HEX

    _write_legend_lock()
    try:
        root = tk.Tk()
        root.title(f"{algo_name} — GUI legend")
        tk.Label(root, text="GUI Legend", font=("Arial", 12, "bold")).grid(
            row=0, column=0, columnspan=3, sticky="w", padx=6, pady=(6, 10))
        for row, (symbol, meaning) in enumerate(legend, start=1):
            code = symbol.split()[0]           # leading token, e.g. 'b' or 'f-XXX'
            hex_color = COLOR_HEX.get(code)     # None for text-format rows (no swatch)
            canvas = tk.Canvas(root, width=22, height=22,
                                highlightthickness=1, highlightbackground="black")
            if hex_color:
                canvas.create_rectangle(0, 0, 22, 22, fill=hex_color, outline=hex_color)
            canvas.grid(row=row, column=0, padx=6, pady=2)
            tk.Label(root, text=symbol, anchor="w", font=("Arial", 10)).grid(
                row=row, column=1, sticky="w", padx=6, pady=2)
            tk.Label(root, text=meaning, anchor="w", font=("Arial", 10)).grid(
                row=row, column=2, sticky="w", padx=6, pady=2)
        root.mainloop()
    finally:
        _LEGEND_LOCK.unlink(missing_ok=True)
```
**Note:** `LEGEND` entries mix genuine color rows (e.g. `"b  (Blue)"`) with text-format rows that describe cell-text patterns, not colors (e.g. `"f-XXX"`, `"inf"`). `symbol.split()[0]` reliably extracts the leading token for both — a real color letter for color rows, or the whole text-pattern string (which won't be a `COLOR_HEX` key) for text-format rows — so `COLOR_HEX.get(code)` naturally returns `None` and the swatch is skipped for those rows without any special-casing. This has been checked against every current entry in both `AStarExplorer.LEGEND` and `DStarLiteExplorer.LEGEND`.

**Window de-duplication (resolved — implement in this pass):** `run.py` is invoked as a fresh OS process by MMS on every "Run" click (`docs/mms.md` Step 5; `run.py` already spawns the legend via `multiprocessing.Process(..., daemon=True)`, `run.py:93-95`), so there is no long-lived parent process across runs that could hold an in-memory handle to a previous run's legend window — any previous legend `Process` object is already gone by the time a new `run.py` starts. Use a PID lock file in the system temp directory instead:
```python
import os
import signal
import tempfile
from pathlib import Path

_LEGEND_LOCK = Path(tempfile.gettempdir()) / "mazesolver_legend.pid"

def _close_existing_legend() -> None:
    """Terminate a legend window left over from a previous run, if any."""
    if not _LEGEND_LOCK.exists():
        return
    try:
        pid = int(_LEGEND_LOCK.read_text().strip())
        os.kill(pid, signal.SIGTERM)
    except (ValueError, ProcessLookupError, PermissionError):
        pass
    _LEGEND_LOCK.unlink(missing_ok=True)

def _write_legend_lock() -> None:
    _LEGEND_LOCK.write_text(str(os.getpid()))
```
In `main()`, call `_close_existing_legend()` **before** spawning the new `multiprocessing.Process` for `_show_legend`; `_show_legend` itself writes its own PID via `_write_legend_lock()` right after entering (see snippet above) and removes the lock file in a `finally` block on normal exit (window closed manually). A stale lock left behind by a crash is harmless: `os.kill(pid, 0)`-style failure (`ProcessLookupError`) is caught and the stale file is cleaned up on the next run.

**Validation:**
- Manual: launch `run.py` twice in a row (simulating two consecutive MMS "Run" clicks) without closing the first legend window; confirm the first window closes automatically when the second run starts, and that a single leftover lock file from a killed process doesn't prevent a later run from opening its own legend.
- Manual: visually confirm swatch colors render correctly for every `LEGEND` row in both `AStarExplorer` and `DStarLiteExplorer`, and that `"f-XXX"`/`"f-XXXh-YYY"`/`"g-XXXr-YYY"`/`"inf"` rows render with no swatch (blank first column) rather than an error or a wrong color.

### B.2 Traversed-path removal

**Files:** `src/algorithms/astar.py` (standalone fix), `src/algorithms/dstar_lite.py` (superseded by §B.3 — no separate patch needed).

The traversed-path highlight (every cell the robot has physically visited, drawn cyan via the shared `_traversed_cells()` helper, `base_algorithm.py:128-135`) adds clutter without information beyond the planned-path highlight, and is removed from both displays:

- **A*:** remove the `for cx, cy in self._traversed_cells(maze_map): api.set_color(cx, cy, 'c')` loop from `AStarExplorer._gui_show_search` (`astar.py:241-242`). This is the only change needed for A* in this item.
- **D\*-Lite:** the equivalent loop in `_gui_redraw_persistent_state` (`dstar_lite.py:308-309`) disappears automatically as part of §B.3's rewrite — the new `_gui_show_search` has no traversed-path drawing at all, so implementing §B.3 already satisfies this item for D*-Lite. No separate patch is required.

**Scope note:** this removes only the *display* call. `_traversed_cells()` and the underlying `MazeMap` visit-count tracking are unrelated to `MetricsLogger`'s `distinct_cells_visited`/`total_visits` metrics and `MazeMap.mark_visit`, and must keep working exactly as today.

**Validation:** manual — run either algorithm and confirm no cyan appears on cells that are neither the planned path nor a goal.

### B.3 D*-Lite: unified `gui_show_search` with priority-ordered coloring

**Files:** `src/algorithms/dstar_lite.py`. **Depends on:** §B.2 (subsumes it).

**Current state:** D*-Lite's display logic is split across `_gui_redraw_persistent_state` (called *before* each `_compute_shortest_path()`, redrawing goals/traversed-path/`_previously_expanded` from the *previous* cycle's state) and inline `set_color`/`clear_color` calls scattered inside `_compute_shortest_path`'s over-/under-consistent branches (`dstar_lite.py:349,369,387,389,406,408`), which paint live as the search runs. A* has no such split: `_gui_show_search` is one function, called once per plan/replan *after* the search completes, driven by final state only.

**Target design:** give D*-Lite the same shape — one entry point, called once *after* each `_compute_shortest_path()` completes, that recolors the board from final live state under a strict, first-match-wins priority order:

| Priority | Rule | Color | Live condition |
|---|---|---|---|
| 1 (highest) | Goal | `G` (Dark Green) / `g` (Green) | `s ∈ _remaining_goal_set` / `s ∈ _reached_goals` |
| 2 | Inconsistent | `R` (Dark Red) | `g[s] ≠ rhs[s]` (equivalently `s ∈ _U`) |
| 3 | Trivial | cleared background | `g[s] = rhs[s] = ∞` |
| 4 | Last expanded/updated | `b` (Blue) | `s ∈ _expanded_this_cycle` |
| 5 | Planned path | `c` (Cyan) | `s` on the reconstructed current-position-to-goal path |
| 6 (lowest) | Previously expanded/updated | `B` (Dark Blue) | `s ∈ _previously_expanded` |

Text is unaffected by this change: `set_text` calls stay exactly where they are today (`_update_rhs`, and both branches of `_compute_shortest_path`), which already keep every touched cell's `g-XXXr-YYY` text live and non-stale — there is no full-board text sweep to add, since nothing in the current implementation ever lets displayed text go stale.

**Resolution — no explicit pruning of `_previously_expanded`/`_expanded_this_cycle` is needed.** Because priority is evaluated top-down against *live* `g`/`rhs`/queue state at draw time (rules 1–3 always checked before rules 4/6), a stale set entry is harmless: if a cell in `_previously_expanded` has since become inconsistent or trivial, rule 2 or 3 preempts rule 6 automatically and it is drawn correctly without ever being removed from the set. This holds even *within* one cycle (a cell expanded earlier in the same `_compute_shortest_path()` call can be re-perturbed and land back in `_U` before the call returns — rule 2 still wins at draw time). The sets are bounded by maze size (≤ width×height ≈ 256 cells on a 16×16 maze) regardless, so unbounded growth is not a concern. This replaces both previously-open "pruning mechanics" questions with a single mechanism: **recompute color from live state at draw time; never trust set membership above rules 1–3.**

**Implementation:**

1. Add a `states()` accessor to `_DStarQueue` (`dstar_lite.py:87-148`) so inconsistent cells can be enumerated directly instead of re-deriving them from a full `g`/`rhs` scan:
   ```python
   def states(self) -> Iterable[tuple[int, int]]:
       """All states currently queued (i.e., all inconsistent nodes)."""
       return self._valid.keys()
   ```
2. Replace `_gui_redraw_persistent_state` with:
   ```python
   def _gui_show_search(self, api: BaseAPI) -> None:
       """Single entry point: recolor the board from live D*-Lite state.
       Call once after each _compute_shortest_path() cycle completes.
       Priority (first match wins): goal > inconsistent > trivial >
       last-expanded (this cycle) > planned path > previously-expanded.
       """
       api.clear_all_color()
       planned = set(self._reconstruct_planned_path()[1:])  # exclude current cell
       candidates = (
           self._remaining_goal_set
           | set(self._reached_goals)
           | set(self._U.states())
           | self._expanded_this_cycle
           | self._previously_expanded
           | planned
       )
       for (x, y) in candidates:
           if (x, y) in self._remaining_goal_set:
               api.set_color(x, y, 'G')
           elif (x, y) in self._reached_goals:
               api.set_color(x, y, 'g')
           elif self._g[(x, y)] != self._rhs[(x, y)]:
               api.set_color(x, y, 'R')
           elif self._g[(x, y)] == self._rhs[(x, y)] == _INF:
               api.clear_color(x, y)
           elif (x, y) in self._expanded_this_cycle:
               api.set_color(x, y, 'b')
           elif (x, y) in planned:
               api.set_color(x, y, 'c')
           elif (x, y) in self._previously_expanded:
               api.set_color(x, y, 'B')
   ```
   Only cells that could plausibly need a non-default color are iterated — trivial cells not otherwise in `candidates` are already blank after `clear_all_color()`, so no full-maze scan is needed (unlike `_memory_occupancy`, which does need one for a different reason — that method is unaffected by this change).
3. Remove the inline `set_color`/`clear_color` calls inside `_compute_shortest_path`'s over-/under-consistent branches (`dstar_lite.py:349,369,387,389,406,408`); keep every `set_text` call in that method exactly as-is.
4. At all four `_compute_shortest_path()` call sites in `run()` (initial setup, reset-recovery, goal-reached, wall-discovery), change the sequencing to: **compute → draw → merge**, in that order — drawing *before* merging `_expanded_this_cycle` into `_previously_expanded` is what keeps rules 4 and 6 correctly disjoint for that cycle's draw call:
   ```python
   self._compute_shortest_path()          # (or: n_exp = self._compute_shortest_path())
   self._gui_show_search(api)
   self._previously_expanded |= self._expanded_this_cycle
   ```
   This replaces the old before-the-cycle `_gui_redraw_persistent_state(...)` call at three of the four sites, and adds the pairing to the initial call site, which previously had no draw call paired with it at all. The old special-casing of "skip the redraw on the very first call" is removed — `_gui_show_search` runs unconditionally after every cycle, including the first, with no special case needed. Keep the existing initial per-cell text/goal-color setup loop (`dstar_lite.py:451-457`) unchanged: it still provides the only on-screen state during the brief window between process start and the first `_compute_shortest_path()` call (before which `_gui_show_search` has not yet run).
5. `_reconstruct_planned_path()` is unchanged in implementation, only in *when* it's called — now after `_compute_shortest_path()` (step 4) rather than before, so it naturally reconstructs from the just-computed `g` values instead of the previous cycle's. This is an intentional improvement (the displayed planned path becomes current, not one cycle stale), not a regression.
6. `LEGEND` (`dstar_lite.py:158-167`) is unaffected — its color/meaning entries already match this priority table exactly.

**Unaffected:** replanning-event logging (`MetricsLogger.log_replanning_event`, gated on `n_exp > 0`) is out of scope for this change and continues to log only plan-changing cycles, independent of how the GUI redraw is structured.

**Validation:**
- `tests/test_integration.py`'s `TestSingleGoal`/`TestMultiGoal`/`TestReplanningConsistency` must keep passing unmodified — none assert GUI-call content, only logged metrics and final robot position, and this change does not touch `_compute_shortest_path`'s `g`/`rhs`/`nodes_expanded` computation, only its side-effecting `set_color` calls.
- New test worth adding (requires a fake/recording `BaseAPI`, not currently present in the test suite): drive `DStarLiteExplorer` on a small maze via a `set_color`-recording stub and assert, after a replanning cycle, that no cell is ever assigned two conflicting colors and that a cell known to be inconsistent at cycle-end is never shown `'b'`/`'B'`.
- Manual: run D*-Lite under MMS on a maze that forces at least two replanning cycles; visually confirm dark-blue (`B`) cells from an earlier cycle correctly flip to red or clear if a later wall discovery makes them inconsistent or trivial, without ever showing stale blue.

### B.4 Terminated-execution display behavior

**Files:** `src/algorithms/base_algorithm.py` (shared helper), `src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`.

When a run reaches its stopping condition, the final GUI frame is simplified: non-goal cells are cleared, and goal cells alone remain marked, per this settled reading (confirmed: "remaining cells" means *non-goal* cells — goal cells, reached or not, are always exempt from the clear and always show at least `G`):

| Scenario | Unreached goal cells | Reached goal cells | Order-of-reach text |
|---|---|---|---|
| Explicit multi-goal (`goals` list ≥ 2 cells) or `n_random_goals ≥ 2` | none remain | `g` (Green) | `1`, `2`, `3`, … in reach order |
| Default centre-area goal (`_is_default_goal`; run stops after the first of the 4 cells) | remain `G` (Dark Green) | the one reached cell: `g` (Green) | none |
| Explicit single goal (`goals` list of exactly 1 cell) | none remain | `g` (Green) | none |

Only goal cells reached in the current run carry the `g` color/text; `self._reached_goals` (already maintained as an append-ordered list by both explorers for the goal-color-persistence behavior in `algorithms_gui_revision.md` §C.3/§D.4) directly gives the 1-based reach order with no new bookkeeping.

**Out of scope — confirmed:** a run terminating *without* its goal condition satisfied (e.g. `AStarExplorer`'s `if len(path) < 2: break`, or `DStarLiteExplorer`'s `if self._rhs[self._s_start] == _INF: break` / `if best_next is None: break`) does not need dedicated final-frame handling — the maze corpus used by this project has been verified solvable by both algorithms, so these branches are not expected to execute in practice. The termination helper below is called unconditionally regardless of which `break`/loop-exit path was taken, which is sufficient: on the (unexercised) stuck path it will simply display whatever `self._reached_goals` holds at that point.

**Implementation** — add to `BaseAlgorithm` (both `self._goals` and `self._is_default_goal` are already set at construction; both subclasses already maintain `self._reached_goals` with the same name and shape):
```python
def _gui_show_termination(self, api: BaseAPI) -> None:
    """Final GUI frame: clear all non-goal decoration, mark goal cells only.
    Called once, unconditionally, at the end of run() in both explorers."""
    api.clear_all_color()
    api.clear_all_text()
    multi_goal = not self._is_default_goal and len(self._goals) >= 2
    for i, (gx, gy) in enumerate(self._reached_goals, start=1):
        api.set_color(gx, gy, 'g')
        if multi_goal:
            api.set_text(gx, gy, str(i))
    if self._is_default_goal:
        reached = set(self._reached_goals)
        for gx, gy in self._goals:
            if (gx, gy) not in reached:
                api.set_color(gx, gy, 'G')
```
`multi_goal` uniformly covers both the explicit-`goals`-list and `n_random_goals` construction paths via `len(self._goals) >= 2`, without needing to know which constructor argument was originally supplied (only `_is_default_goal` needs distinguishing, to skip text for the default centre-area case even though it also involves 4 candidate cells).

Call `self._gui_show_termination(api)` at the end of `run()` in both `AStarExplorer` and `DStarLiteExplorer`, immediately before `logger.stop()`. No headless-mode special-casing is needed: `SimAPI.set_color`/`clear_all_color`/`set_text`/`clear_all_text` are all confirmed no-ops (`src/api/sim_api.py:107-129`), matching every other GUI call in the codebase.

**Validation:**
- Manual, per scenario: run `--algo astar`/`--algo dstar_lite` with (a) `--goal 3 3 --goal 0 3` (explicit multi-goal), (b) no goal flags (default centre area), (c) `--goal 3 3` (explicit single goal); confirm the final frame in each matches its row in the table above.
- `tests/test_integration.py`'s existing goal-reaching assertions are unaffected (this only changes GUI calls, not final robot position or logged metrics), but consider adding a `set_color`/`set_text`-recording `BaseAPI` stub test asserting the exact final-frame color/text set for the multi-goal case (order-of-reach text `"1"`, `"2"`, … matches `_reached_goals` order).

---

## C. Cross-cutting validation checklist

| Change | Files | Existing tests affected | New coverage to add |
|---|---|---|---|
| A.1 `--heuristic` | `base_algorithm.py`, `astar.py`, `run.py` | none (default preserves current behavior) | Manhattan-heuristic admissibility/expansion-count test on `_single_path_maze_with_wall()` |
| A.2 `--no-log` | `run.py` | none | `_parse_args()` unit test; manual no-file-written check |
| A.3 stderr formatting | `base_algorithm.py`, `astar.py`, `dstar_lite.py` | `tests/test_no_stray_print.py` (must keep passing unmodified — still routes through `_report_event`) | `cost_ratio=None` formatting test |
| B.1 legend palette/dedup | `constants.py`, `run.py` | none | manual: swatch rendering per `LEGEND` row; manual: second-run auto-closes first legend |
| B.2 traversed-path removal | `astar.py` (+ `dstar_lite.py` via B.3) | none (cosmetic) | manual visual check |
| B.3 D*-Lite `gui_show_search` | `dstar_lite.py` | `TestSingleGoal`/`TestMultiGoal`/`TestReplanningConsistency` (unaffected — no GUI-call assertions) | recording-`BaseAPI` test: no conflicting colors, no stale `B`/`b` on now-inconsistent cells |
| B.4 termination display | `base_algorithm.py`, `astar.py`, `dstar_lite.py` | goal-reaching/log-schema assertions unaffected | manual per-scenario final-frame check; optional recording-`BaseAPI` test for order-of-reach text |

No item in this blueprint changes `MetricsLogger`'s schema or any planning outcome other than A.1 (heuristic, opt-in via flag, admissibility-preserving) — so `tests/test_integration.py` is expected to keep passing unmodified throughout.
