# Assessment request: is the Phase 3 algorithm implementation consistent with running in the MMS GUI once `run.py` (Phase 5) exists?

## Background

This project studies planning/replanning algorithms for maze solving and renders their execution on the [`mms`](https://github.com/mackorone/mms) simulator GUI (by Mackorone), whose Python API (`mackorone/mms-python`'s `API.py`) is cloned into `src/api/mms_api.py`. For batch experiments and metrics unaffected by the GUI, a headless backend also exists: `src/api/sim_api.py` (`SimAPI`). Both implement the shared contract in `src/api/base_api.py` (`BaseAPI`). Full API documentation: [`src/api/README.md`](../../src/api/README.md).

Per `docs/mms.md` ("Step 3: Configure an Algorithm"), MMS runs an algorithm via a user-configured **run command** — an arbitrary shell command (executable + optional CLI args) bound to a button in the GUI. Per "Step 4: Select a Maze", the maze file is chosen separately in the GUI and is **not** passed to the run command as an argument. This is not a problem in principle: `MmsAPI` gets everything it needs (wall sensing, maze dimensions via `mazeWidth`/`mazeHeight`) over stdin/stdout, and the maze layout itself is never needed upfront because of the **freespace assumption** (unexplored cells are treated as passable until a wall is sensed) — unlike `SimAPI`, which requires the full wall matrix upfront since it simulates the maze itself rather than querying a running GUI process.

`docs/implementation_roadmap.md` Phase 5 assigns this integration to a not-yet-implemented `run.py` at the repo root, which is meant to be the exact script path configured in MMS. **Cross-checking `implementation_roadmap.md` Phase 5 against its own companion doc `docs/instructions/implementation_roadmap_revision.md` (§5, §10) shows the design is already decided, not open**: `run.py` parses `--algo`, `--goal`, `--n-goals`, `--seed`, always instantiates `MmsAPI` (never `SimAPI`), and is documented as "the MMS GUI entry point only" — headless/batch runs go exclusively through `experiments/run_batch.py`, which instantiates `SimAPI` directly and never touches `run.py`. So there is no mode-detection ambiguity left to resolve at the `run.py` level; what remains open is whether that already-decided design is actually consistent with the rest of the codebase as implemented. That is the subject of this assessment.

## What to assess

Cross-check `src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, and `src/algorithms/base_algorithm.py` (`BaseAlgorithm`) against:
1. `docs/implementation_roadmap.md` Phase 3 (marked complete) and Phase 5 ("End-to-End Integration", not yet implemented),
2. the more detailed Phase 3/5 specs in `docs/instructions/implementation_roadmap_revision.md` (especially §8.2–§8.4 algorithm specs and §9 GUI display specs),
3. the actual `BaseAPI` contract (`src/api/base_api.py`) and its two implementations (`src/api/mms_api.py`, `src/api/sim_api.py`).

Goal: determine whether, once `run.py` is implemented exactly as currently specified, the two algorithms can actually be driven through it and run correctly inside the MMS GUI — not just under `SimAPI` in `tests/test_integration.py`, which is the only place they are currently exercised end-to-end. Report any logical inconsistency, missing feature, or flaw that would prevent this, and state precisely what edit to `implementation_roadmap.md` (and/or `docs/mms.md`) each finding implies.

## Discrepancies already found while preparing this request (starting points — verify, don't take as given)

These were found by grepping the current source tree; they are leads for the assessment, not its conclusions:

- **`docs/mms.md`'s documented run command contradicts the `run.py` plan.** Step 3 ("Run Command Format") tells the user to configure MMS with `.venv/bin/python -m src.algorithms.[algorithm_name]`, invoking an algorithm module directly — not `run.py` at the repo root. This predates Phase 5 and was never updated. Confirm whether `docs/mms.md` needs rewriting once `run.py` exists, and whether it should be superseded entirely rather than left as an alternate invocation path.
- **Only `src/algorithms/base.py` (the Phase 1 wall-follower) is runnable the way `docs/mms.md` currently describes** — it has `main()` and `if __name__ == "__main__"` (`src/algorithms/base.py:37,105`). `src/algorithms/astar.py` and `src/algorithms/dstar_lite.py` (the actual Phase 3 deliverables) have no `main()`, no `argparse`, and no `__main__` guard (confirmed via grep) — they are only ever instantiated directly from Python (currently only from `tests/test_integration.py`, via `SimAPI`). Confirm this means `run.py` is a hard prerequisite for GUI use of either algorithm (not an optional convenience), and check that `run.py`'s planned constructor call matches `BaseAlgorithm.__init__`'s actual signature (`src/algorithms/base_algorithm.py:35-44`).
- **Reset handling appears unimplemented despite being marked done.** `implementation_roadmap.md` Phase 3d (line 86) checks off "`run() → None` — sense → plan → act → log loop; handles `was_reset()` → `ack_reset()` → reset internal state" as complete. `BaseAPI.was_reset()`/`ack_reset()` exist (`src/api/base_api.py:113-119`) and `MmsAPI` implements them over the real MMS protocol (`src/api/mms_api.py:121-125,157-158`), but grepping `run()` in both `astar.py` and `dstar_lite.py` shows neither calls `was_reset()` or `ack_reset()` anywhere. In the MMS GUI, the user can press "Reset" mid-run (e.g., after reaching the goal, to re-run from the start cell without restarting the process) — confirm whether this gap would cause the algorithm to desync from the robot's actual position/heading on the next GUI run, or hang/error, and whether the Phase 3d checklist item should be reopened.
- **`BaseAlgorithm.__init__` calls `api.maze_width()`/`api.maze_height()` synchronously at construction** (`src/algorithms/base_algorithm.py:50-51`), before any wall query. Confirm this ordering is valid against the real MMS stdin/stdout protocol (i.e., that `mazeWidth`/`mazeHeight` may legally be the first commands issued) and isn't just an assumption that happens to hold under `SimAPI`.

## Deliverable

Update this file (`assesment.md`) with:
1. For each point above (and any other inconsistency found): confirmed / not an issue / partially an issue, with the evidence (file:line) either way.
2. Any additional gaps found beyond the above.
3. A concrete list of edits implied for `docs/implementation_roadmap.md` (e.g., reopen a checklist item, add a new Phase 5 sub-task, add an explicit "update `docs/mms.md`" task) and/or `docs/mms.md`, precise enough to apply directly in a follow-up pass. Do not edit `implementation_roadmap.md` itself in this pass — only record the findings and the specific edits they imply here.

---

## Assessment results

Full read of `src/algorithms/astar.py`, `src/algorithms/dstar_lite.py`, `src/algorithms/base_algorithm.py`, `src/api/base_api.py`, `src/api/mms_api.py`, `src/api/sim_api.py`, `src/api/README.md`, `docs/mms.md`, `docs/implementation_roadmap.md`, and `docs/instructions/implementation_roadmap_revision.md`.

### 1. `docs/mms.md` run command — **CONFIRMED**

`docs/mms.md:54` documents the run command as `.venv/bin/python -m src.algorithms.[algorithm_name]`. This invokes an algorithm module directly as `__main__`. It is inconsistent with the Phase 5 plan (both `implementation_roadmap.md:119` and `implementation_roadmap_revision.md:138`), where `run.py` at the repo root — not a module under `src/algorithms/`— is "the script path configured in MMS." `docs/mms.md` was written in Phase 1, before `run.py` was designed, and was never revisited. It must be rewritten, not left as an alternate path (see finding 2).

### 2. `run.py` is a hard prerequisite, not an alternative — **CONFIRMED**

Grep + full read confirm `src/algorithms/astar.py` and `src/algorithms/dstar_lite.py` have no `main()`, no `argparse`, no `if __name__ == "__main__"`. `AStarExplorer`/`DStarLiteExplorer` are pure classes; the only code that ever instantiates them today is `tests/test_integration.py`, always with `SimAPI`. So `docs/mms.md`'s current run-command template would fail outright if pointed at `src.algorithms.astar` or `src.algorithms.dstar_lite` (`python -m` would import the module and do nothing — no `__main__` block runs). `run.py` is therefore not an added convenience on top of an already-runnable module; it is the *only* way to drive either algorithm from MMS.

`run.py`'s planned constructor call (`implementation_roadmap_revision.md:138`: "instantiates `MmsAPI`, the chosen algorithm, `MazeMap`, `Robot`, and `MetricsLogger`") matches `BaseAlgorithm.__init__`'s actual signature (`base_algorithm.py:35-44`: `api, maze_map, robot, logger, goals=None, n_random_goals=None, random_seed=None`) — the planned `--goal X Y` (repeatable) / `--n-goals N` / `--seed S` CLI flags map cleanly onto `goals` / `n_random_goals` / `random_seed`. No mismatch here.

### 3. Reset handling — **CONFIRMED gap, but scope is narrower than "prevents running in the simulator"**

Neither `astar.py::run()` nor `dstar_lite.py::run()` calls `was_reset()` or `ack_reset()` anywhere, despite `implementation_roadmap.md:86` (Phase 3d) marking "handles `was_reset()` → `ack_reset()` → reset internal state" as done (`[X]`). `BaseAPI` declares both methods (`base_api.py:113-119`), `MmsAPI` wires them to the real protocol (`mms_api.py:121-125,157-158`), and `SimAPI` implements a working reset (`sim_api.py:135-142`) — only the algorithms never call them.

Concretely, this matters because MMS's "Reset" button teleports the simulated mouse back to the start cell (and is the mechanism MMS uses to let one process run the maze more than once — c.f. `getStat`'s `current-run-*` vs `best-run-*` vs `total-*` in `src/api/README.md:117-128`, which only make sense if a single process is expected to complete multiple runs). With no `was_reset()` polling, an in-progress `run()` call has no way to notice a Reset happened: the real mouse teleports, but the algorithm's `Robot` object keeps believing it is wherever it last moved itself. Every subsequent `wall_front()`/`wall_left()`/etc. reading from MMS is then interpreted against the wrong `(x, y, heading)`, corrupting `MazeMap` with walls attributed to the wrong cells, and the next `move_forward()` command is issued relative to a fictitious position — this can drive the real mouse into a wall MMS reports as a crash, or simply produce a nonsensical explored-map log. This is a real desync bug reachable any time a human clicks Reset while the process is still inside `run()`, which is a normal, expected interaction in the MMS GUI (not a corner case).

Note the scope precisely: this does **not** block a first, uninterrupted GUI run from completing (nothing here prevents `algorithm.run()` from finishing once, cleanly, in the MMS GUI, as long as nobody presses Reset mid-run). It blocks (a) recovering gracefully from a mid-run Reset, and (b) any workflow that expects the same process to complete a second, subsequent run (e.g., an MMS "speed run" after exploration) — which the roadmap doesn't currently ask for either, so this is a real but currently-out-of-scope-for-the-roadmap's-own-stated-goals gap. Recommendation: either reopen the Phase 3d checklist item and implement a `was_reset()` check inside each `run()` loop (call `logger`/state reset and re-sync `Robot` to the start position via `ack_reset()` on detection), or explicitly narrow the Phase 3d checklist wording to state that reset handling is out of scope for this project (single exploration run only) — right now the roadmap asserts something the code doesn't do, which is the actual inconsistency, independent of which resolution is chosen.

### 4. `maze_width()`/`maze_height()` called synchronously at construction — **NOT AN ISSUE**

`BaseAlgorithm.__init__` (`base_algorithm.py:50-51`) calls `api.maze_width()`/`api.maze_height()` before any wall query. This mirrors the standard `mms-python` usage pattern (`mazeWidth`/`mazeHeight` are ordinary synchronous stdin/stdout round-trips with no ordering precondition documented in `src/api/README.md` or upstream `mms-python`), and is exactly what the existing Phase-1 `src/algorithms/base.py` wall-follower does implicitly by hardcoding a 16×16 `np.zeros` maze instead of querying dimensions. No protocol violation.

### Additional gaps found (beyond the four leads)

- **A\* GUI cell-text display deviates from the §9 spec.** `implementation_roadmap_revision.md` §9 specifies "Cell text | After A* expansion | `set_text(x, y, str(f_value))` for each expanded cell." `astar.py::_gui_show_fvalues` (lines 146-158) instead displays f-values only for cells on the *planned path*, not for all nodes actually expanded during the search (the method's own docstring calls this "simplified from full A* expansion"). This is a deliberate simplification, not an oversight, but it will fail Phase 5's own verification step ("confirm that walls, cell colours, and cell text are rendered correctly per the GUI specs in §9") as currently written, since the spec and the implementation disagree. Either the implementation should be extended to log all expanded cells (requires threading the closed-set out of `_a_star`, which already computes it — see `_a_star`'s return tuple `(path, nodes_expanded, open_size, closed_size)` at `astar.py:65` — cheap to extend to return the actual cell set) or §9 should be edited to describe path-only f-value display.

**A\* GUI desired behavior**
The GUI should render the f-value of every cell that was expanded or added to the open list, in the current replanning cycle; After a replanning event, all cell text should be cleared and the new f-values redisplayed. Here is a table summarizing the desired behavior for A* in the GUI:

| Element | Condition | Action |
|---------|-----------|--------|
| Cell text | Initial state | Empty (`""`) for all cells except start: display `f = h(start)` |
| Cell text | After A* expansion | `set_text(x, y, str(f_value))` for each expanded cell and all cells added to open list |
| Cell color | When expanded | `set_color(x, y, 'b')` (blue) |
| Cell color | When added to open list | `set_color(x, y, 'R')` (Dark Red) |
| Cell color | Replanning event | Clear all cell colors with `clearAllColor()`|
| Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
| Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |
| Cell text | Replanning event | Clear all text with `clear_all_text()`; redisplay f-values from new search |

`note`: the **GUI text cell** accepts a maximum of 10 characters (2 lines of 5 characters each); the `f_value` should be displayed: `f-XXX` where `XXX` is the f-value, or `f-inf ` for infinity.

- **D\*-Lite cell text can go stale for cells whose `rhs` changes outside `_compute_shortest_path`.** `run()` calls `_update_rhs`/`_update_vertex` directly from the wall-sensing/goal-reached code paths (e.g. `dstar_lite.py:356-360`, `426-428`, `448-455`) without a matching `set_text` call; only nodes actually popped inside `_compute_shortest_path` (lines 239-240, 266-267) get their displayed `g:X r:Y` text refreshed. A cell whose `rhs` changes but is never popped before the next planning cycle keeps showing stale text in the GUI. Same Phase 5 §9-verification checklist item is affected.

**D\*-lite GUI desired behavior**
Cell text should always reflect the current `g` and `rhs` values for every cell, even if the cell is not popped from the queue. Here is a table summarizing the desired behavior for D*-Lite in the GUI:

| Element | Condition | Action |
|---------|-----------|--------|
| Cell text | Initial state | `set_text(x, y, "g-inf r-inf")` for all cells; goal cells: `"g-inf r-0"` |
| Cell text | After update | `set_text(x, y, f"g-{g} r-{rhs}")` whenever g or rhs changes |
| Goal cell color | Until reached | `set_color(x, y, 'G')` (Dark Green) |
| Goal cell color | When reached | `set_color(x, y, 'g')` (Green) |
| Inconsistent node | Inserted into queue | `set_color(x, y, 'R')` (Dark Red) |
| Consistent node | Removed from queue (expanded) | `set_color(x, y, 'b')` (Blue) |
| Trivial node | `g = rhs = ∞` | `clear_color(x, y)` |
| Wall display | New wall discovered | `set_wall(x, y, direction)` for each newly confirmed wall |

`note`: the **GUI text cell** accepts a maximum of 10 characters (2 lines of 5 characters each); the `g_value` and `rhs_value` should be displayed: `g-XXXr-YYY` where `XXX` is the g-value, and `YYY` is the rhs-value; `XXX` and `YYY` are `inf` for infinity.

- **`run.py`'s Phase 5 spec never says to persist a log for GUI runs.** Both `implementation_roadmap.md:119` and `implementation_roadmap_revision.md:138` say `run.py` "instantiates ... a `MetricsLogger` ... calls `algorithm.run()`" but neither mentions calling `logger.export_json(...)` afterward. Phase 6 asks to "manually verify at least two runs per algorithm in the MMS GUI to confirm that the headless `SimAPI` behaviour matches the real simulator" (`implementation_roadmap.md:141`) — that comparison needs a persisted JSON log from the GUI run to diff against a `SimAPI` log, so `run.py` should explicitly export one (with an output path, e.g. `results/logs/`, distinguishing GUI runs from batch runs by filename).

- **Minor/cosmetic:** `BaseAlgorithm.__init__` stores `self._start_pos = robot.position` (`base_algorithm.py:49`) but no code in `base_algorithm.py`, `astar.py`, or `dstar_lite.py` ever reads it. Harmless dead state, not GUI-blocking; worth a one-line cleanup whenever the file is next touched, not a roadmap item.

- No stray `print()`/stdout writes were found in `src/algorithms/*.py` or `src/api/mms_api.py` outside the intended protocol I/O — the MMS stdin/stdout stream is not at risk of corruption from debug output.

### Concrete edits implied for `docs/implementation_roadmap.md`

1. **`docs/mms.md`, Step 3 ("Run Command Format")** — rewrite the template from `.venv/bin/python -m src.algorithms.[algorithm_name]` to `.venv/bin/python run.py --algo [astar|dstar_lite] [--goal X Y ...] [--n-goals N] [--seed S]` (or equivalent), once `run.py` exists. Remove/replace the `--log`/`--verbose` "common flags" example, which refers to an `argparse` config that doesn't exist on `astar.py`/`dstar_lite.py` and won't exist on them post-`run.py` either (the flags belong to `run.py`, not the algorithm modules).
2. **`implementation_roadmap.md` Phase 5** — add an explicit checklist item: "Update `docs/mms.md` Step 3 run-command example to invoke `run.py` instead of `src.algorithms.[name]`, matching the actual `run.py` CLI." This is currently implied but not stated, and is exactly the kind of doc drift this assessment caught.
3. **`implementation_roadmap.md` Phase 3d, line 86** — reopen (uncheck) the "handles `was_reset()` → `ack_reset()`" sub-item, or split it into its own explicit checklist line so it isn't hidden inside an already-checked-off bullet. Add the missing `was_reset()`/`ack_reset()` polling to both `run()` loops (recommended: check once per iteration of the main sense→plan→act loop, immediately after `_sense_and_update` or `_move_to`).
4. **`implementation_roadmap.md` Phase 5** — add a checklist item for `run.py` to call `logger.export_json(...)` after `algorithm.run()` returns, with an output directory, so GUI runs produce a comparable log for the Phase 6 GUI-vs-`SimAPI` verification step.
5. **`implementation_roadmap_revision.md` §9 — replace the A\* GUI table** with the revised behavior above (§"A\* GUI desired behavior"): expanded cells get `set_color(x, y, 'b')`, cells added to the open list get `set_color(x, y, 'R')`, a replanning event clears all colors (`clearAllColor()`) in addition to all text, and cell text switches to the `f-XXX` / `f-inf ` format (10-char budget: `f-` + 3-digit value, or `f-inf ` for infinity). Pair with a Phase 5 (or reopened Phase 3a) checklist item: extend `AStarExplorer._a_star` to return the actual expanded-cell and open-list-cell sets (not just their counts — `nodes_expanded`/`open_size`/`closed_size` already computed at `astar.py:65,106,130` need to carry the cell identities, not just `len(...)`), and update `_gui_show_fvalues` (or a new method) to color/text every such cell instead of only the returned path.
6. **`implementation_roadmap_revision.md` §9 — replace the D\*-Lite GUI table's text-update rule** with the revised behavior above (§"D\*-Lite GUI desired behavior"): cell text must reflect current `g`/`rhs` for *every* cell whenever either value changes, not only for cells popped inside `_compute_shortest_path`; format changes to `g-XXXr-YYY` (10-char budget, `inf` for infinity). Pair with a Phase 5 (or reopened Phase 3b) checklist item: add a `set_text` call at every site in `dstar_lite.py` that mutates `_g` or `_rhs` outside `_compute_shortest_path` — at minimum `_update_rhs` itself (`dstar_lite.py:173-190`) and the call sites that change `_g`/`_rhs` directly during goal-reached and wall-discovery handling (`dstar_lite.py:356-360`, `410-411`, `426-428`, `448-455`) — so no cell can display stale text between planning cycles.
