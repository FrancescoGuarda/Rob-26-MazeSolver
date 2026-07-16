# Revisions

This document records a second round of remarks, gathered from further manual MMS-GUI testing of `AStarExplorer` and `DStarLiteExplorer`, on top of the behavior already delivered by `algorithms_gui_revision.md` (that blueprint is now fully implemented in `src/algorithms/base_algorithm.py`, `src/algorithms/astar.py`, and `src/algorithms/dstar_lite.py`). Each remark below has been checked against the current implementation and against `algorithms_gui_revision.md` where relevant, and annotated with the concrete file/function it targets, the precise gap it closes, and any ambiguity that still needs a decision.

As with its predecessor, this file is a **requirements** document, not yet an implementation blueprint: it records *what* should change and *why*, cross-checked for accuracy, but intentionally leaves *how* (data structures, call sites, exact code) to a later pass. Where an item's current wording admits more than one reasonable implementation, that is flagged as an open question rather than resolved here.

It is organized into two parts: **Operative** (CLI flags and stderr/log behavior in `run.py` and the two explorers) and **GUI** (MMS display behavior).

---

## Operative

### 1. Heuristic-selection flag (`--heuristic`)

Extend `BaseAlgorithm` (`src/algorithms/base_algorithm.py`) so the heuristic used during planning is selectable, and expose the choice as an optional `run.py` flag:

```
--heuristic [min_path|manhattan]
```

Default: `min_path` (matches current behavior, so omitting the flag changes nothing).

**Current state:** `BaseAlgorithm` already provides two BFS-based heuristic helpers:
- `_compute_goal_heuristic(maze_map, goals)` — multi-source BFS *backward* from the goal set, wall-aware (respects confirmed walls, freespace assumption elsewhere). This is what "`min_path`" refers to: the shortest distance to the nearest goal under the *currently known* wall layout, not a straight-line distance. `AStarExplorer` calls this exclusively as its heuristic today (`astar.py:281,301,353`).
- `_compute_start_heuristic(maze_map, start)` — BFS *forward* from a start cell, also wall-aware. Its docstring states it is "used by D*-Lite at initialisation," and `src/algorithms/README.md` repeats that claim — but this is **stale**: a repo-wide search confirms `_compute_start_heuristic` is never called anywhere in `dstar_lite.py` (or elsewhere). `DStarLiteExplorer` instead computes its own heuristic inline via `_h(s)` (`dstar_lite.py:200-202`), a hardcoded Manhattan distance from `s` to `self._s_start`, ignoring walls entirely.

**"manhattan" should be understood precisely as:** straight-line `|dx| + |dy|` distance, ignoring all wall knowledge — i.e., exactly the formula D*-Lite's `_h()` already uses (for a different target: the current start cell, not the goal).

**Open question — scope of the flag:** A*'s heuristic is goal-directed (distance to nearest goal), while D*-Lite's is start-directed (distance to `s_start`), and the latter is not a stylistic choice: D*-Lite's `Key(s)` consistency invariant depends on the heuristic being consistent with respect to a fixed reference point (`s_start`, adjusted incrementally via `km`), which is a different mathematical role than A*'s admissible-to-goal heuristic. It is not yet decided whether `--heuristic` is meant to:
- apply to A* only (leaving D*-Lite's Manhattan-to-start heuristic untouched), or
- apply to both, in which case D*-Lite's `min_path` option would need a wall-aware distance *to `s_start`* — presumably built on the otherwise-unused `_compute_start_heuristic` — and its interaction with the `km` incremental-update mechanism and D*-Lite's correctness guarantees would need explicit verification, since naively recomputing a wall-aware heuristic on a schedule that mismatches D*-Lite's incremental-repair assumptions could break its consistency proof.

**Wiring implied (for the later blueprint):** a new `BaseAlgorithm.__init__` parameter, a new `argparse` choice in `run.py::_parse_args`, and passing the resolved heuristic through to whichever call site(s) the scope decision above lands on. Once implemented, update `docs/mms.md` (Run Command Format section) and `src/algorithms/README.md` (constructor signature table and the `_compute_goal_heuristic`/`_compute_start_heuristic` rows) to document the new flag and correct the stale `_compute_start_heuristic` docstring/README claim either way.

### 2. `--no-log` flag

Add an optional boolean flag to `run.py` that skips writing the JSON metrics log to `results/logs/`, while leaving stderr diagnostics untouched:

```
--no-log
```

Default: off (logs are saved, matching current behavior).

**Current state:** `run.py::main()` unconditionally calls `logger.export_json(output_dir=args.output_dir)` after `algorithm.run()` returns (`run.py:112`). There is currently no way to suppress this short of not passing `--output-dir` correctly or deleting the file afterward.

**Why stderr is unaffected regardless:** `_report_event` (`base_algorithm.py:119-126`, used for `[WALL]`/`[REPLAN]` reporting) writes directly to `sys.stderr` and has no dependency on `MetricsLogger` or `export_json` — the two are already independent subsystems, so `--no-log` only needs to guard the `export_json` call itself.

**Note:** with `--no-log` set, `--output-dir` becomes irrelevant for that run; worth a one-line mention in `run.py --help` / `docs/mms.md` so the two flags' relationship isn't left implicit.

### 3. Stderr log formatting

Two independent refinements to the diagnostics introduced by `algorithms_gui_revision.md` §E (already implemented in both explorers via `_report_event`):

**a. Consolidate `[WALL]` lines per cell.** Today, each of the (up to four) newly-discovered walls at a cell produces its own line, e.g.:
```
[WALL] (0, 0) e
[WALL] (0, 0) s
[WALL] (0, 0) w
```
Replace this with a single line per sensing event, reporting the status of all four directions at that cell in one place. Two equivalent presentations are given as alternatives (a decision between them is left open):
```
[WALL] (0, 0) _ e s w
```
or, using directional glyphs instead of letters:
```
[WALL] (0, 0) _ → ↓ ←
```
In both, the four positions represent a fixed direction order and `_` marks a direction with no wall.

Missing details worth settling before implementation (not resolved here):
- **Direction order.** The example lists north (absent, `_`), then east, south, west — matching `Direction`'s canonical enum order (`N, E, S, W`; `constants.py:12-21`) and the order `_sense_and_update` already reads walls in (`base_algorithm.py:183-188`). Adopting that existing order avoids introducing a second convention.
- **What the line reports.** Because `_sense_and_update` senses all four walls of the current cell on every call (not just the newly-discovered ones), a consolidated line can cheaply show the cell's complete now-known wall status, not merely the deltas from this pass — matching the given example, where `e s w` are shown even though (in the original per-wall version) they might have been discovered across separate earlier visits to the same cell. This should be stated explicitly, since "log ... all discovered walls" is ambiguous between "all walls discovered in this pass" and "all walls known at this cell so far."
- **Emission gate.** To actually *reduce* verbosity (the stated goal) rather than increase it, the line should still be emitted only when at least one new wall was confirmed during that sensing pass — i.e., preserve the existing `is_new` gate that currently suppresses output when nothing changed — rather than logging a line on every sense.
- **Symbol choice.** Letters vs. arrows is a presentation choice with no functional difference; either is fine as long as it's applied consistently at all four call sites (A*'s initial sense and main-loop sense; D*-Lite's initial sense and main-loop sense).

**b. Round `[REPLAN]` numeric fields to two decimals.** Illustrative example given:
```
[REPLAN] pos=(0, 8) expanded=12 residual=9 cost_ratio=1.3333333333333333 time_ms=2.734
```
→
```
[REPLAN] pos=(0, 8) expanded=12 residual=9 cost_ratio=1.33 time_ms=2.73
```

**Correction to the "current" example:** both `astar.py` and `dstar_lite.py` already format `time_ms` with `:.2f` at their `_report_event` call sites (e.g. `astar.py:368`), so `time_ms` is **already** rounded to two decimals today — the `time_ms=2.734` shown above as "current" does not actually occur. The only remaining gap is `cost_ratio`, which is interpolated at full `float` precision from `record['cost_ratio']` (`MetricsLogger.log_replanning_event`, `src/metrics/logger.py:95-98`).

**Missing detail:** `cost_ratio` can be `None` — `MetricsLogger` sets it to `None` whenever `residual_distance` is `0` or falsy (`logger.py:95-98`, e.g. when a replan is triggered while the robot is already at, or adjacent with zero residual to, a goal). A bare `f"{value:.2f}"` on `None` raises `TypeError`, so the rounding must special-case `cost_ratio is None` (e.g., print `None`/`n/a` unrounded) rather than assume it is always a float.

---

## GUI

### 1. Legend color palette and font

Update `src/constants.py` to add a hex-color lookup table for the MMS GUI legend window (currently `constants.py`'s `COLORS` dict maps each color letter only to a human-readable name, e.g. `'b': 'Blue'` — there is no hex value anywhere in the codebase to actually paint a swatch).

Proposed palette (dark/uppercase and light/lowercase variants):

| Code | Meaning | Hex | Code | Meaning | Hex |
|------|---------|---------|------|---------|---------|
| `A` | Dark Gray | `#1A1A1A` | `a` | Gray | `#B3B3B3` |
| `B` | Dark Blue | `#000036` | `b` | Blue | `#0000BB` |
| `C` | Dark Cyan | `#003434` | `c` | Cyan | `#006867` |
| `G` | Dark Green | `#004F00` | `g` | Green | `#00B600` |
| `O` | Dark Orange | `#371800` | `o` | Orange | `#BF6100` |
| `R` | Dark Red | `#550000` | `r` | Red | `#DF0000` |
| `V` | Dark Violet | `#390035` | — | — | — |
| `Y` | Dark Yellow | `#333300` | `y` | Yellow | `#B3B300` |
| — | — | — | `k` | Black | `#000000` |
| — | — | — | `w` | White | `#FFFFFF` |

**Discrepancy to reconcile:** `constants.py`'s existing `COLORS` dict is explicitly commented "all 15 supported colors" and currently defines 15 entries (`k, b, a, c, g, o, r, w, y` and `B, C, A, G, R, Y`). This palette lists 17 codes — it additionally includes `O` (Dark Orange) and `V` (Dark Violet), neither of which exists in `COLORS` today. Since neither `docs/mms.md` nor `src/api/mms_api.py` documents the MMS protocol's full supported color set, this discrepancy should be verified against the actual MMS simulator (`mackorone/mms`) before implementation — either `COLORS` is currently incomplete (in which case add `O`/`V`), or this palette over-specifies relative to what the real GUI accepts (in which case trim it). This is a factual question with a single right answer, not a design choice, so it is called out rather than guessed at here.

Render the legend using this palette, styled like `src/algorithms/base.py::show_color_legend()` — which draws each entry as a `tk.Canvas` rectangle filled with the entry's actual hex color next to a text label, using `("Arial", 12, "bold")` for the window title and `("Arial", 10)` for each entry — replacing `run.py::_show_legend`'s current plain-text rendering (`tk.Label` only, no color swatch, `("Courier", 12)` for the symbol column).

**Open question (unchanged from the prior pass — left for the implementation phase, not resolved here):** should a previously-opened legend window be closed automatically when a new simulation starts, to avoid multiple stacked windows? One relevant constraint worth carrying into that decision: `run.py` is invoked as a **fresh OS process** by MMS on every "Run" click (confirmed in `docs/mms.md` Step 5 and by `run.py`'s use of `multiprocessing.Process(..., daemon=True)`, `run.py:93-95`) — there is no long-lived parent process across runs that could hold an in-memory handle to a previous run's legend window. Any "close the existing one first" behavior would therefore need an out-of-process signal (e.g. a lock/PID file, or OS-level window enumeration by title), not just tracking a `Process` object.

### 2. Traversed-path removal (A* and D*-Lite)

The traversed-path highlight (every cell the robot has physically visited, drawn cyan via the shared `_traversed_cells()` helper, `base_algorithm.py:128-135`) clutters the GUI without adding information beyond what the planned-path highlight already conveys, and should be removed from both explorers' displays:

- `AStarExplorer._gui_show_search` (`astar.py:241-242`) — remove the `for cx, cy in self._traversed_cells(maze_map): api.set_color(cx, cy, 'c')` loop.
- `DStarLiteExplorer._gui_redraw_persistent_state` (`dstar_lite.py:308-309`) — remove the equivalent loop.

**Scope note:** this only removes the *display* call. `_traversed_cells()` and the underlying `MazeMap` visit-count tracking are unaffected and still needed — `MetricsLogger`'s `distinct_cells_visited`/`total_visits` metrics and `MazeMap.mark_visit` are unrelated to this GUI highlight and must keep working exactly as they do today.

### 3. D*-Lite: consolidate into a single `gui_show_search` entry point

Today, D*-Lite's display logic is split across two places: `_gui_redraw_persistent_state` (called *before* each `_compute_shortest_path()`, redraws goals/traversed path/`_previously_expanded`) and inline `set_color`/`clear_color`/`set_text` calls scattered inside `_compute_shortest_path`'s over-/under-consistent branches (`dstar_lite.py:349-408`), which paint live as the search runs. A* has no such split — its `_gui_show_search` is a single function called once per plan/replan, after the search completes, driven purely by the final `f_values`/`open_set`/`closed_set`.

The request is to give D*-Lite the same shape: one `gui_show_search`-equivalent entry point, called once after each `_compute_shortest_path()` completes, that recolors the board from the final post-cycle state according to a strict priority order (highest wins when a cell matches more than one rule):

| Priority | Rule | Color | Condition |
|---|---|---|---|
| 1 (highest) | Goal | `G` (Dark Green) / `g` (Green) | cell is a remaining / reached goal |
| 2 | Inconsistent | `R` (Dark Red) | `g(s) ≠ rhs(s)` (equivalently: in the priority queue `U`) |
| 3 | Trivial | cleared background | `g(s) = rhs(s) = ∞` |
| 4 | Last expanded/updated | `b` (Blue) | expanded/updated (and left consistent) in the most recent `_compute_shortest_path()` call |
| 5 | Planned path | `c` (Cyan) | on the current-position-to-goal path, and not already colored by rule 4 |
| 6 (lowest) | Previously expanded/updated | `B` (Dark Blue) | expanded/updated (and left consistent) in an earlier call, not yet reclassified by rules 2–3 |

Text rule: on every `_compute_shortest_path()` call, whether or not it changed the plan (`n_exp` may be `0`), the displayed `g`/`rhs` text should reflect current values for the affected cells.

**Architectural implication (not prescribed, but implied by "single entry point"):** painting `'b'`/`'R'` colors *inline*, mid-search, is incompatible with a priority-ordered redraw computed from final state — a cell's rule-4-vs-rule-2 outcome can only be known once the cycle has finished. Consolidating therefore implies moving the inline `set_color` calls in `_compute_shortest_path` out of the search loop entirely, replacing them with a single post-cycle pass; `_gui_redraw_persistent_state`'s current before-the-cycle call would similarly need to move to after.

**Redefinition of "last" vs. "previously" expanded, relative to `algorithms_gui_revision.md` §D.3:** §D.3 defined `_previously_expanded` as a monotonically-growing, write-only history of every cell ever expanded in the run, merged from `_expanded_this_cycle` after *every* `_compute_shortest_path()` call regardless of whether that call changed the plan — and the current implementation already does exactly this (`dstar_lite.py:482-483,533-534,579-580,614-615` all merge unconditionally; only the *metrics-logging* call is gated on `n_exp > 0`, not the merge). So the "even if `n_exp=0`" rule is **already satisfied** by the existing code — that is not a new requirement.

What **is** new, and not yet implemented, is **pruning**: today `_previously_expanded` only grows and is never pruned. The rule above requires actively removing a cell from it (recoloring to `R` or clearing) once that cell's live `g`/`rhs` state changes to inconsistent or trivial in a *later* cycle, in order to preserve the stated invariant that both the "last" and "previously" sets contain only currently-consistent, non-trivial cells and are disjoint from each other and from rules 1–3.

**Open question on pruning mechanics:** a cell can become inconsistent as a side effect of a *different* cell's update (via `_update_rhs`/`_update_vertex` on a neighbour) without itself being re-expanded that cycle — so pruning cannot rely solely on watching which cells get expanded each cycle; it requires checking each `_previously_expanded` member's *current* `g`/`rhs`/queue-membership state at (or before) each redraw. Whether that's done by scanning the set every cycle, or by hooking every `g`/`rhs` mutation site to update set membership as a side effect, is left open for the implementation pass.

**Open question on the text rule:** "the `g` and `rhs` values of all cells should be updated" could mean a full width×height text refresh every cycle, or (matching current behavior, where `set_text` is already called live at every `g`/`rhs` mutation site — `_update_rhs`, and both branches of `_compute_shortest_path`) simply confirms that no cell touched by a cycle should have stale displayed text. The former is a behavior change (full-board redraw every cycle); the latter is a restatement of existing behavior. Left open here.

**Path-reconstruction ordering:** `_reconstruct_planned_path()` (used for rule 5) is currently invoked *before* `_compute_shortest_path()` runs (inside `_gui_redraw_persistent_state`, using the *previous* cycle's `g` values). Moving the redraw to after the cycle (per the architectural implication above) means it would naturally use the *just-computed* `g` values instead — worth flagging as an intentional behavior change, not an oversight, once this is implemented.

**Unaffected:** replanning-event logging (`MetricsLogger.log_replanning_event`, gated on `n_exp > 0`, i.e. only cycles that actually changed the plan) is explicitly out of scope for this change and must continue to log only plan-changing cycles.

### 4. Terminated-execution display behavior

When a run reaches its stopping condition, the final GUI frame should be simplified: clear per-search decoration (colors/text on non-goal cells) and leave only goal cells marked, with the following per-scenario rules:

| Scenario | Unreached goal cells | Reached goal cells | Order-of-reach text |
|---|---|---|---|
| Explicit multi-goal (`goals` list, ≥2 cells) or `n_random_goals ≥ 2`, all reached | — (none remain) | `g` (Green) | shown: `1`, `2`, `3`, … in reach order |
| Default centre-area goal (`_is_default_goal`, run stops after the first of the 4 cells) | remain `G` (Dark Green) | the one reached cell: `g` (Green) | none |
| Explicit single goal (`goals` list of exactly 1 cell) | — (none remain) | `g` (Green) | none |

**Resolving an apparent tension in the original wording:** "clear all the remaining cells' color and text ... only display the reached goal/s in green" (general rule) appears to conflict with "in the default case ... all goal cells should be displayed in green (`G`)" (the default-case exception), since a literal "clear all remaining cells" would also wipe the three unreached centre-area cells. The coherent reading — adopted above — is that "remaining cells" means *non-goal* cells: goal cells (reached or not) are always exempt from the clear and always show at least `G`; only the default-area case ever has unreached goal cells left over at termination, since it's the only scenario where the run stops without every listed goal being reached by design (`algorithms_gui_revision.md` §B.1). This reading should be confirmed, not assumed, before implementation.

**Order-of-reach text is cheap to compute:** both explorers already maintain `self._reached_goals` as an append-ordered list for the goal-color-persistence fix in `algorithms_gui_revision.md` §C.3/§D.4 — its existing index order already *is* the reach order, so no new bookkeeping is needed to know "1st, 2nd, 3rd."

**Gap not addressed by the original wording:** none of the three rows above cover a run that terminates *without* satisfying its goal condition — e.g. `AStarExplorer.run()`'s `if len(path) < 2: break` (no path exists) or `DStarLiteExplorer.run()`'s `if self._rhs[self._s_start] == _INF: break` / `if best_next is None: break` (robot stuck). What the final frame should look like in that case is currently unspecified and should be decided explicitly rather than left to fall out of whatever the last live frame happened to show.

**Cross-cutting scope:** this behavior is identical for both explorers and is triggered purely by how each `run()` exits its main loop — a natural candidate for a shared `BaseAlgorithm` helper (paralleling `_display_maze_outline`/`_traversed_cells`) rather than duplicated logic in `astar.py` and `dstar_lite.py`, though the exact mechanism is left for the implementation pass.
