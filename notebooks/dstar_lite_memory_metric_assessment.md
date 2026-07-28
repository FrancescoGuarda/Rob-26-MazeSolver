# Assessment: D*-Lite `memory_occupancy` non-monotonicity

Reminder for the report: the `memory_occupancy` metric logged for `DStarLiteExplorer`
is not monotonically non-decreasing, which contradicts what
[`docs/implementation_roadmap.md:182`](../docs/implementation_roadmap.md) originally
expected ("monotonically increasing for D\*-Lite"). This note assesses whether that
is a bug, an implementation-specific optimization, or standard D\*-Lite behaviour, and
whether the metric or the analysis needs to change as a result.

## 1. Background and context

`experiments/run_batch.py` logs one `memory_occupancy` value per replanning event for
both explorers (`src/metrics/logger.py::log_replanning_event`). For A\*, this is
`len(open_set) + len(closed_set)` of that event's from-scratch search — thrown away
after the event, so it is bounded and resets every time by construction
(`src/algorithms/astar.py:363`). For D\*-Lite it is
[`_memory_occupancy()`](../src/algorithms/dstar_lite.py#L455-L459):

```python
def _memory_occupancy(self) -> int:
    nontrivial_g = {s for s, v in self._g.items() if v != _INF}
    nontrivial_rhs = {s for s, v in self._rhs.items() if v != _INF}
    return len(nontrivial_g | nontrivial_rhs)
```

i.e. the count of cells currently holding a **non-default** `g` or `rhs` value. The
roadmap's Phase 6 checklist (`docs/implementation_roadmap.md:182`) predicted this
would grow monotonically, on the reasoning that D\*-Lite (unlike A\*) never discards
its search state between replans. That checklist line is still the single unchecked
`[ ]` item in an otherwise fully-checked roadmap — a fair signal that the discrepancy
was noticed but never resolved in writing.

`notebooks/data_analysis.ipynb`'s "Memory occupancy growth" section already knows
about this ("*non è strettamente monotòno: un muro può rimettere dei g/rhs a ∞, da cui
le oscillazioni locali*") and works around it by comparing the **peak** `memory_occupancy`
per run rather than the final value or a monotonic trend. This report checks whether
that workaround is technically justified, or whether the metric itself should change.

## 2. What actually causes the non-monotonicity (verified against logged data)

Two candidate mechanisms exist in the code, and only one of them is responsible for
what shows up in the exported logs:

1. **Goal retirement.** When a goal is reached, `self._rhs[reached] = self._g[reached]
   = _INF` (`dstar_lite.py:628-629`). But the goal-reached branch calls
   `self._compute_shortest_path()` and then `continue`s (`dstar_lite.py:649`),
   **skipping** the `if has_new:` block that calls `log_replanning_event`
   (`dstar_lite.py:660,682`). Goal retirement is explicitly *not* logged as a
   replanning event — so it cannot directly produce a decrease in the exported series
   (it can only affect the count at the *next* wall-triggered event afterward).
2. **Cost-increase propagation on wall discovery** — the `elif`/`else` split in
   `_compute_shortest_path` (`dstar_lite.py:396,426`): when `g(u) ≤ rhs(u)`, the
   *underconsistent* branch runs `self._g[u] = _INF`, exactly the standard D\*-Lite
   response to an edge-cost increase (a newly confirmed wall). This **is** logged,
   inside `if has_new:`.

I pulled the actual `replanning_events[*].memory_occupancy` sequences from
`results/logs/*/dstar_lite/*.json` (all 220 D\*-Lite runs: 55 mazes × k=1..4) to check
how often and how severely this occurs in practice:

| goal count k | runs with ≥1 decrease | median share of events that decrease | median (peak − final) | median (Σ drop / peak) |
|---|---|---|---|---|
| 1 | 55 / 55 | 25% | 14 cells | 0.13 |
| 2 | 53 / 55 | 22% | 8 cells | 0.13 |
| 3 | 55 / 55 | 28% | 15 cells | 0.20 |
| 4 | 55 / 55 | 26% | 19 cells | 0.20 |

Example (`00japan`, k=4, 69 events): the series rises 27→202 then settles with a
declining tail to 180, with 23 individual decreases along the way (e.g.
`153→135`, `198→186→185`), well before the run's final goal is even close to being
reached.

Since mechanism (1) is never logged, **the observed non-monotonicity in the exported
data is entirely attributable to mechanism (2)** — the standard cost-increase
(wall-discovery) update rule, not to this project's multi-goal extension. It is
frequent (roughly a quarter of all replanning events) and not a corner case.

## 3. Comparison with standard D\*-Lite

Koenig & Likhachev (AAAI 2002) initialise `g(s) = rhs(s) = ∞` for every state and
define `ComputeShortestPath`'s repair loop with exactly the two cases implemented
here:

```
if g(u) > rhs(u):  g(u) = rhs(u)                       # overconsistent
else:              g(u) = ∞;  UpdateVertex(u and preds) # underconsistent (cost increase)
```

Setting `g(u)` back to `∞` in the underconsistent branch is not an optimisation layered
on top of the algorithm — it *is* the algorithm's mechanism for propagating an edge-cost
increase backward through the search graph. A newly sensed wall is modelled exactly as
a cost increase (`1 → +∞` on that edge), so `dstar_lite.py:426` is a direct, faithful
implementation of the paper, not a project-specific shortcut. `∞` in both the paper and
this implementation means "no currently supported finite path is known" — a cell
legitimately re-enters that state whenever its only known support is invalidated,
independent of whether the search "remembers" having visited it before.

The one piece that *is* outside the base paper's scope is multi-goal retirement
(`dstar_lite.py:628-629`): the original D\*-Lite is framed around a single fixed goal
with a moving start. Retiring a reached goal by forcing `rhs = g = ∞` is this project's
own (reasonable, necessary) extension to support sequential multi-goal runs — but, per
§2, it never shows up in the logged metric anyway, so it doesn't bear on the observed
non-monotonicity.

**Conclusion:** reclaiming `g = rhs = ∞` cells from the *metric* is consistent with, and
directly derived from, the algorithm's own semantics. Nothing about it deviates from
standard D\*-Lite.

## 4. Does the metric measure "memory usage"?

This is the part that is genuinely a modelling choice, and it is more subtle than
"reclaimed vs. not reclaimed":

- **The Python dictionaries never shrink.** No code path ever calls `del` on `self._g`
  or `self._rhs`. Once a key is written, its entry persists (holding `_INF`) for the
  rest of the run. So the *actual* resident size of `self._g`/`self._rhs` as Python
  objects is monotonically non-decreasing — but `_memory_occupancy()` deliberately
  filters trivial entries out of the *count*, so the reported number is **not** "how
  many dict entries physically exist" (that number, uncomputed, would only ever grow).
- **Nor is raw `len(self._g)` the right monotonic fix**, because `self._g` and
  `self._rhs` are `defaultdict(lambda: _INF)`: any read of a missing key — e.g. every
  neighbour probe in `_update_rhs` or the expansion loop, whether or not that neighbour
  ever becomes part of a real path — auto-vivifies a `_INF` entry via `__missing__`.
  `len(self._g)` would therefore count cells that were merely glanced at, not cells the
  algorithm actually knows something about — arguably a worse proxy for "useful
  resident state" than the current filtered count.
- What `_memory_occupancy()` actually measures is **the number of cells currently
  holding non-default information** — a live, physically meaningful quantity (the
  "working set" D\*-Lite is relying on right now to avoid recomputation), and a
  deliberate, defensible way to make it comparable to A\*'s `open_set ∪ closed_set`
  (also a "currently relevant states" count, not a raw allocation count).

So there isn't a single "correct" notion of memory occupancy being violated here; there
are at least three distinct quantities in play (informational footprint / cumulative
distinct informative cells / raw dict size), and the implementation picked the first —
a reasonable one — while the roadmap's checklist implicitly assumed the second without
writing that down.

**Important consequence for the existing comparison:** since dict entries are never
deleted, the true cumulative distinct-informative-cell count at any point `i`
(call it `D(i)`) satisfies `D(i) ≥ max_{j≤i} memory_occupancy(j)` (any cell that ever
became non-trivial stays counted in `D`, even if it later reverts). That means the
**peak** `memory_occupancy` already used in the notebook's Figure 2 is a valid lower
bound on `D` at the end of the run — it can only *understate* D\*-Lite's true
cumulative footprint, never overstate it. The qualitative comparison the notebook
draws (D\*-Lite retains far more state at once than A\*'s from-scratch searches, a gap
of roughly 2.5×–3.8× depending on k) is therefore safe, and if anything conservative.

## 5. Impact assessment of the alternatives considered

**Option A — leave the metric/analysis exactly as is (implicit).** Technically fine,
but leaves a documented discrepancy (the open roadmap checklist item) unexplained,
which is a bad look for the final report if a reviewer notices the same thing the task
prompt did.

**Option B — revise `_memory_occupancy()` to a strictly monotonic cumulative count**
(track a `set` of every cell that has ever been non-trivial, unioned across the whole
run, instead of filtering live). Impact:
- Only `DStarLiteExplorer` changes; A\*'s metric, the replanning triggers, node
  expansions, planning times, and moves are all untouched — nothing about *when* or
  *how* the robot moves changes, only one logged number per event.
- Cheap to regenerate: only the 220 D\*-Lite runs need re-running (headless `SimAPI`,
  deterministic, seconds each), against goal placements already fixed and recorded in
  each log's `scenario` field. This is unlike `detour_metric_limitations.md`'s
  precedent, where fixing the metric would change *which goals* get placed and force
  re-running the full 440-run corpus (both algorithms) — that cost/benefit trade-off
  does not transfer here.
- Expected effect on results: per §4, `D(i) ≥ peak`, so the corrected peak-per-run
  values could only move up, not down. The reported 2.5×–3.8× D\*-Lite/A\* memory gap
  (`memory_peak_distribution.svg`) would likely widen slightly, not reverse or
  disappear. `memory_run_example.svg`'s single-run trajectory would lose its dips and
  become a step function that only rises. `memory_vs_event_trend.svg`'s linear trend
  would steepen slightly. No other figure or table in `data_analysis.ipynb` (the
  `nodes_expanded`/`residual_distance` regressions, the replanning-cost bars, the
  completeness table) depends on `memory_occupancy` at all.
- No currently-drawn conclusion would need to be walked back — at most, the magnitude
  of the memory-usage gap would be reported as slightly larger and the trajectory
  figure would look cleaner.

**Option C — keep the current definition, document the behaviour.** Zero cost, no
re-running, no figure changes. Requires only: closing the roadmap checklist item with
an explanation, and a short prose note (this file) that the peak-based comparison
already used is a deliberate and valid choice, not an oversight.

## 6. Final recommendation

**Keep** the current implementation and the current `_memory_occupancy()` definition,
and **document** the reasoning — do not revise the metric or re-run experiments.

Justification:
- The behaviour is standard D\*-Lite, not an implementation-specific hack: setting
  `g(u) = ∞` on a cost increase (`dstar_lite.py:426`) is the textbook underconsistent
  branch from Koenig & Likhachev, and it is the *only* mechanism that produces the
  non-monotonicity actually seen in the logs (§2) — multi-goal retirement, the one
  genuinely project-specific piece, never even reaches the exported metric.
- The metric's filtering of trivial cells is a coherent, physically meaningful choice
  ("cells currently holding useful information"), not an arbitrary discard — and it is
  actually a *better* proxy for that purpose than the naive monotonic alternative
  (`len(self._g)`), which would be inflated by `defaultdict` auto-vivification on mere
  neighbour probes (§4).
- The existing notebook analysis already handles this correctly: it compares
  **peak-per-run**, explicitly avoids assuming monotonicity, and (per the `D(i) ≥ peak`
  argument in §4) that choice is a safe lower bound, not a workaround papering over a
  wrong number. None of the currently-drawn conclusions are at risk.
- Unlike the detour-index precedent (`docs/detour_metric_limitations.md`), a revision
  here would be cheap (Option B), but there is no accuracy problem to justify paying
  even that small cost — the only thing actually wrong is that the roadmap's
  expectation was never corrected in writing.

**Action items** (documentation only):
1. Update `docs/implementation_roadmap.md:182` from "monotonically increasing for
   D\*-Lite" to reflect the peak/working-set framing actually implemented and analysed,
   and check the box.
2. Add one or two sentences to `src/algorithms/README.md`'s D\*-Lite section (or a
   cross-reference to this file) noting that `memory_occupancy` can locally decrease
   on wall discovery, by design, and why.
3. Optionally, for extra rigor in the final report only (not required to validate any
   existing figure): add a second, strictly monotonic `cumulative_memory_occupancy`
   field alongside the existing one, and a supplementary panel showing it converges to
   the same qualitative story with a larger gap. This is a nice-to-have, not a
   correction.
