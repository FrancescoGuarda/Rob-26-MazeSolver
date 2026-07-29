# Experimental Evaluation of A\* and D\*-Lite for Maze Exploration

Technical report on the experimental campaign described in Phase 6 of
`docs/implementation_roadmap.md`. It summarises the setup, the metrics, the
results, and the observations drawn from them. All figures referenced here are
produced by `notebooks/data_analysis.ipynb` and stored under `docs/res/`; all
numbers are taken from that notebook's tables or from the same underlying run
logs.

---

## 1. Experimental setup

### 1.1 Algorithms compared

Two exploration strategies for an initially unknown maze, both adopting the
**freespace assumption** (every unsensed cell is treated as passable until a
wall is confirmed) and both replanning on the same trigger — after each forward
move the robot senses all four walls, and a **replanning event** occurs when a
newly confirmed wall lies on the plan currently being executed.

| | **A\*** | **D\*-Lite** |
|---|---|---|
| Strategy | Replans from scratch on the current partial map at every event | Repairs the existing plan incrementally from the changed edges |
| Heuristic | Straight-line Manhattan distance to the nearest remaining goal, ignoring wall knowledge | Manhattan distance from a state to the robot's current position, with a key offset correcting for the moving start |
| State between events | Discarded | Retained |
| Multi-goal handling | Goal removed from the remaining set on arrival, heuristic recomputed | All goals seeded simultaneously; a reached goal is retired and the plan re-routed |

The two therefore differ in *where* they pay: A\* pays repeatedly in search
effort, D\*-Lite pays continuously in retained state. The evaluation is built
around measuring both sides of that trade.

### 1.2 Maze corpus

55 mazes, all **16×16** (256 cells), drawn from the micromouse competition
corpus in `mazes/txt/`. Every maze is fully connected — a hard precondition, so
that the distance-to-goal metric is finite at every logged event. All runs start
from cell `(0,0)` facing North. Because difficulty is manufactured through goal
placement rather than through maze structure, the *same* 55 mazes are reused at
every difficulty level; no maze is set aside as "easy" or "hard".

### 1.3 Goal-count scenarios

Goals are placed deterministically by maximising the **detour index**
`detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)` — the ratio between the
true in-maze distance and the straight-line lower bound. A cell with a high
detour *looks* close but is actually far, which is precisely the configuration
that defeats a planner working from a partial map. The first goal maximises the
detour from the start; each subsequent goal maximises the *minimum* detour over
the start and all goals already placed.

The same procedure generates all four scenarios: placement is **nested**
across *k* — the first *L* goals of a *k*-goal scenario are exactly the goals of
the *L*-goal scenario. Scenarios `k = 1, 2, 3, 4` therefore form a strictly
growing goal set rather than four independent configurations, and every run at
every *k* must reach **all** of its placed goals.
`docs/res/goal_heatmap_evolution.svg` (produced by `notebooks/goals_analysis.ipynb`)
shows the score map being maximised at each step for a representative maze.

This construction has a known bias, discussed in §5.3, which qualifies any
reading of *k* as a difficulty axis.

### 1.4 Run matrix

**2 algorithms × 55 mazes × 4 goal-count scenarios = 440 runs**, executed
headlessly. Each run produced one log containing run-level scalars (moves,
distinct cells visited, replanning-event count, cumulative planning time,
cumulative nodes expanded) and a per-event record (position, planning time,
nodes expanded, residual distance to goal, memory occupancy). The 440 runs cover
each `(algorithm, maze, k)` combination exactly once; across all of them the
logs contain **18 176 replanning events** (9 831 for A\*, 8 345 for D\*-Lite).

---

## 2. Methodology

The evaluation is organised along three axes, each answering a different
question about the same trade-off.

### 2.1 Cumulative cost of replanning

**Metrics.** `cumulative_nodes_expanded` — the total number of node expansions
summed over every replanning event in a run — and `cumulative_planning_time_s`,
the total wall-clock time spent planning over those same events.

**What they measure.** Node count is the algorithm-intrinsic measure of search
effort: how much searching happened, independent of language, machine, or
implementation constant factors. Planning time is the practical cost of that
searching once each implementation's per-node overhead is included. Reported
separately, the pair distinguishes *how much work was done* from *how expensive
that work was in practice* — the two need not move together, and §4.1 shows
where they part company.

**Why it belongs in the evaluation.** This is the headline comparison: it
measures the total price of keeping a plan valid over a full exploration run,
which is the quantity D\*-Lite's incremental repair is designed to reduce.

### 2.2 Per-event cost versus residual distance

**Metrics.** `nodes_expanded` for a single replanning event, plotted against
`residual_distance` — the wall-aware shortest-path distance from the robot's
position at that event to the nearest remaining goal.

**Fairness of the x-axis.** `residual_distance` is computed identically for both
algorithms and independently of either one's own planning heuristic. It is a
property of the world state at that moment, not of the planner, and is therefore
a legitimate common abscissa.

**What it measures.** For A\*, `nodes_expanded` is the size of that event's
complete from-scratch search. For D\*-Lite, it is the number of non-stale states
actually repaired (states re-extracted from the queue with an outdated key are
not counted). The regression of one on the other characterises how each
algorithm's per-decision effort scales with distance-to-go.

**Why it belongs in the evaluation.** The cumulative view of §2.1 confounds two
effects — cost per event and number of events. This axis isolates the first,
answering whether D\*-Lite's advantage comes from cheaper individual repairs, and
whether that advantage grows or shrinks as the goal gets further away.

**Restriction.** The regression is fitted on the *dense band*
`residual_distance ≤ 30`, which contains 18 047 of the 18 176 events (99.3%).
Beyond that threshold, bins hold fewer than 60 events each and their confidence
intervals become too wide to support a fit; the completeness table nonetheless
reports every bin, marking which were used.

### 2.3 Memory occupancy

**Metric.** `memory_occupancy`, logged per replanning event, counted in cells
and therefore bounded above by 256 (the whole maze).

* **A\*** — the size of the open set plus the closed set of that event's single
  from-scratch search. It is discarded once the plan is produced, so it measures
  a *transient* peak, not accumulated state.
* **D\*-Lite** — the number of cells currently holding a finite `g` or `rhs`
  value, i.e. the working set the algorithm keeps resident in order to repair
  the plan incrementally rather than re-derive it.

Both are "cells currently relevant to the search" counts, which is what makes
them comparable; neither is a raw allocation figure.

**Why it belongs in the evaluation.** It is the counterpart to §2.1: A\* buys
statelessness at the price of repeated search, D\*-Lite buys cheap repair at the
price of retained state. Without this axis the comparison would report only the
side of the trade where D\*-Lite wins.

**Presentation.** Four views, in increasing order of aggregation:

1. A single representative run, to show the raw shape of the trajectory before
   any averaging. Run lengths vary from 1 to 113 events, so averaging occupancy
   at a given event index across all runs suffers survivorship bias (only the
   longest runs contribute at high indices). One concrete run avoids that. The
   run shown is chosen as the maze whose D\*-Lite peak is closest to the median
   at `k = 4`, so it is typical rather than selected for effect.
2. Per-run peak and mean occupancy pooled over all 220 runs per algorithm.
3. The occupancy trend against event index, broken down by *k*.
4. The same trend pooled across *k*, plus a completeness table by event-index
   bin.

Views 3 and 4 are computed on the central 95% of runs by length (`n_events`
between 3 and 88, retaining 420 of 440 runs), the same dense-band logic applied
to run length instead of distance, which bounds the survivorship effect.

---

## 3. Results

### 3.1 Cumulative cost of replanning

`docs/res/replanning_cost_bars.svg` — mean per algorithm and *k*, error bars ±1
standard deviation across the 55 mazes.

| *k* | A\* nodes | D\*-Lite nodes | A\* time (s) | D\*-Lite time (s) |
|---|---|---|---|---|
| 1 | 541.6 ± 578.6 | 182.9 ± 174.7 | 0.0039 ± 0.0036 | 0.0013 ± 0.0012 |
| 2 | 826.2 ± 597.5 | 339.0 ± 199.8 | 0.0063 ± 0.0034 | 0.0024 ± 0.0015 |
| 3 | 1016.5 ± 559.2 | 437.6 ± 171.2 | 0.0084 ± 0.0035 | 0.0032 ± 0.0013 |
| 4 | 1098.1 ± 697.3 | 442.5 ± 170.6 | 0.0097 ± 0.0040 | 0.0034 ± 0.0017 |

**D\*-Lite expands roughly a third to a half of A\*'s nodes at every scenario**:
the A\*/D\*-Lite node ratio is 2.96× at `k = 1`, then 2.44×, 2.32× and 2.48× at
`k = 2, 3, 4`; the corresponding planning-time ratios are 3.05×, 2.61×, 2.61×
and 2.81×. The result is not an artefact of averaging — D\*-Lite expands fewer
nodes in **212 of the 220** matched `(maze, k)` pairs (5 losses, 3 ties) and
records lower planning time in **219 of 220**.

Both cumulative node curves rise monotonically with *k*; neither is flat: A\*
goes 542 → 826 → 1017 → 1098 (+103% from `k = 1` to `k = 4`), D\*-Lite
183 → 339 → 438 → 443 (+142%). D\*-Lite's *relative* growth is the larger of the
two, but its absolute increment is less than half of A\*'s (+260 versus +556
nodes), and it flattens between `k = 3` and `k = 4` (+1.1%) where A\*'s is still
climbing (+8.0%). D\*-Lite's repair cost scales with the number of remaining
goals through `k = 3` and only levels off at `k = 4`.

Dispersion is large and mostly reflects maze-to-maze variation: A\*'s standard
deviation across mazes is 107% of its mean at `k = 1` and 55–72% at `k ≥ 2`.
D\*-Lite's is both absolutely and relatively smaller from `k = 2` onward (39% at
`k = 3` and `k = 4`), so its cost is not only lower but more predictable across
the corpus. Both algorithms' dispersion is at its worst at `k = 1`, for the
reason set out in §4.2: a single detour-placed goal can be 5 or 107 real steps
from the start depending on the maze.

Two secondary quantities from the same runs sharpen the picture:

* **Replanning events per run** — A\* 25.5 / 41.6 / 53.5 / 58.3 for `k = 1…4`,
  D\*-Lite 20.6 / 35.8 / 46.1 / 49.1. D\*-Lite triggers *fewer* events, in 154 of
  220 matched pairs (39 losses, 27 ties), 15.1% fewer on average.
* **Nodes per event** — A\* 21.3 / 19.9 / 19.0 / 18.9, D\*-Lite 8.9 / 9.5 / 9.5 /
  9.0. Roughly a factor of two, stable across *k*.

So D\*-Lite's cumulative advantage decomposes into two independent contributions
of unequal size: it replans about 15% less often, and each replan costs about
half as much.

### 3.2 Per-event cost versus residual distance

`docs/res/nodes_vs_residual_distance_by_k.svg` (one panel per *k*) and
`docs/res/nodes_vs_residual_distance_aggregate.svg` (pooled).

Linear fits on the dense band (`residual_distance ≤ 30`):

| *k* | A\* slope (R²) | D\*-Lite slope (R²) |
|---|---|---|
| 1 | 1.83 (0.56) | 0.66 (0.18) |
| 2 | 2.12 (0.55) | 0.69 (0.14) |
| 3 | 2.49 (0.61) | 0.86 (0.15) |
| 4 | 2.75 (0.64) | 0.81 (0.15) |
| pooled | 2.32 (0.59) | 0.76 (0.15) |

**A\*'s per-event cost grows about 2.3 nodes for every additional cell of
distance-to-go; D\*-Lite's grows less than 1.** Pooled across all runs, A\*'s
slope is 3.1× D\*-Lite's. The separation is present at every *k* and widens with
it: A\*'s slope climbs steadily from 1.83 to 2.75 as goals are added, while
D\*-Lite's stays in a narrow 0.66–0.86 band.

The completeness table quantifies the same divergence across the distance range:

| Residual distance | A\* — mean [95% CI] | D\*-Lite — mean [95% CI] | Ratio | In trend |
|---|---|---|---|---|
| (0, 5] | 6.6 [6.5, 6.7] (n=2211) | 4.7 [4.6, 4.8] (n=1772) | 1.4× | yes |
| (5, 10] | 13.5 [13.3, 13.6] (n=3397) | 6.9 [6.7, 7.0] (n=2788) | 2.0× | yes |
| (10, 15] | 22.8 [22.4, 23.1] (n=2535) | 9.1 [8.7, 9.4] (n=2177) | 2.5× | yes |
| (15, 20] | 34.8 [33.7, 35.9] (n=1144) | 13.2 [12.4, 14.0] (n=1053) | 2.6× | yes |
| (20, 25] | 52.6 [49.9, 55.3] (n=403) | 20.0 [17.1, 22.9] (n=366) | 2.6× | yes |
| (25, 30] | 77.7 [70.2, 85.3] (n=87) | 30.7 [23.9, 37.4] (n=114) | 2.5× | yes |
| (30, 35] | 120.8 [101.5, 140.1] (n=21) | 56.9 [34.3, 79.5] (n=42) | 2.1× | no |
| (35, 40] | 98.5 [79.9, 117.2] (n=13) | 87.8 [58.0, 117.6] (n=16) | 1.1× | no |
| (40, 59] | 95.1 [83.9, 106.3] (n=20) | 83.2 [53.8, 112.6] (n=17) | 1.1× | no |

Within the dense band the ratio climbs from 1.4× in the nearest bin to 2.6× by
`(15, 20]` and then holds at 2.5–2.6×: **the advantage grows sharply over short
range and then stabilises**, rather than widening without limit. The three
excluded bins hold 13 to 42 events each; their means stop increasing and their
intervals overlap heavily, which is exactly the instability the `≤ 30` cutoff
was introduced to exclude. They should not be read as evidence that the gap
closes at long range.

A third figure of merit makes the difference dimensionless. Expressing each
event's cost as nodes expanded per unit of residual distance, A\* records a
median of **1.67** and D\*-Lite **0.75**. D\*-Lite therefore expands, on the
typical event, *fewer nodes than there are cells between it and the goal* —
repair cost is sublinear in distance-to-go, whereas A\* pays more than one
expansion per cell of remaining path.

The low R² of D\*-Lite's fits (0.14–0.18, against 0.55–0.64 for A\*) is itself a
result rather than a defect: A\*'s from-scratch search size is largely determined
by how far the goal is, while D\*-Lite's repair size is dominated by *what
changed* — how much of the retained plan the newly sensed wall invalidated —
which is only weakly related to distance. The fitted line describes D\*-Lite's
central tendency, not a strong predictive relationship.

### 3.3 Memory occupancy

**Single representative run** — `docs/res/memory_run_example.svg`, maze
`00japan` at `k = 4` (the median-peak D\*-Lite run at that *k*).

| | events | first | peak | final | mean |
|---|---|---|---|---|---|
| A\* | 77 | 10 | 122 | 15 | 35.4 |
| D\*-Lite | 69 | 27 | 202 | 180 | 141.3 |

The two trajectories have qualitatively different shapes. A\* oscillates within a
low band, each event's search independent of the last, ending essentially where
it started (15 cells). D\*-Lite climbs from 27 to a plateau near 200 — 79% of the
entire maze — and stays there, ending at 180.

D\*-Lite's climb is not perfectly monotonic — in this run it falls 23 times over
its 69 events even while climbing overall from 27 to 202. A newly confirmed
wall is modelled as an edge-cost increase, and D\*-Lite's textbook response is
to reset the affected cell's `g` value to infinity and propagate the change
backward, which removes that cell from the working-set count until a new
shortest path re-supports it. Across all 220
D\*-Lite runs, **26.6% of consecutive event-to-event transitions are decreases**
(2 161 of 8 125) — A\* shows an even higher share (33.1%), since each of its
events is an independent from-scratch search unrelated to the last.
`memory_occupancy` therefore measures a *live* working set, not a monotonically
accumulating count. Because entries are never physically reclaimed from the
underlying `g`/`rhs` maps, though, the true cumulative footprint is bounded
below by the peak — so the peak-based comparisons used throughout this section
are conservative: if anything they understate D\*-Lite's memory disadvantage,
never overstate it.

**Pooled across all 220 runs per algorithm** — `docs/res/memory_peak_distribution.svg`.

| | peak (median) | peak (std) | mean (median) | mean (std) |
|---|---|---|---|---|
| A\* | 69.0 | 38.76 | 29.51 | 10.35 |
| D\*-Lite | 182.5 | 71.63 | 116.58 | 47.21 |

**D\*-Lite's median peak occupancy is 2.6× A\*'s, and its median per-run average
is 4.0× A\*'s.** In relative terms A\* peaks at 27.0% of the maze and D\*-Lite at
71.3%. The gap on the *mean* exceeds the gap on the *peak*, which reflects the
shape difference above: A\* only briefly touches its peak, D\*-Lite sits near
its own for most of the run.

The distributions barely overlap. Across all 220 A\* runs the largest peak
observed is 192 cells (75% of the maze), and **no** A\* run exceeds 230 (90%).
Among D\*-Lite runs, **31 of 220 exceed 230** and **4 reach the full 256** — the
entire maze resident at once — spread over three mazes (`2009japan`,
`2012uk-techfest`, `90tor`). D\*-Lite's memory is bounded only by the maze
itself.

Broken down by *k*, the median peaks are:

| *k* | A\* | D\*-Lite | ratio |
|---|---|---|---|
| 1 | 45 | 78 | 1.73× |
| 2 | 63 | 182 | 2.89× |
| 3 | 78 | 188 | 2.41× |
| 4 | 80 | 202 | 2.52× |

Both algorithms' peaks grow with *k*, but very differently: A\*'s rises 78%
(45 → 80) and D\*-Lite's 159% (78 → 202), almost all of the latter between
`k = 1` and `k = 2`. The `k = 1` figure is the one that needs care — it is low
not because D\*-Lite's working set is intrinsically small with one goal, but
because `k = 1` runs are short (20.6 events on average, §3.1) and most of them
terminate before the working set saturates. From `k = 2` on, where runs are long
enough to reach the plateau, the ratio settles at 2.4–2.9×. The per-event trend
below separates the two effects — run length from goal count.

**Trend over the run** — `docs/res/memory_vs_event_by_k.svg` (by *k*) and
`docs/res/memory_vs_event_trend.svg` (pooled), on the trimmed run pool.

| *k* | A\* slope (R²) | D\*-Lite slope (R²) | D\*-Lite intercept |
|---|---|---|---|
| 1 | 0.38 (0.07) | 4.09 (0.72) | 32.3 |
| 2 | 0.28 (0.05) | 3.30 (0.60) | 51.7 |
| 3 | 0.29 (0.05) | 2.46 (0.51) | 69.4 |
| 4 | 0.24 (0.04) | 2.28 (0.45) | 75.4 |
| pooled | 0.26 (0.04) | 2.74 (0.54) | 62.0 |

**A\*'s occupancy is statistically flat over the course of a run** (pooled slope
0.26 cells/event, R² = 0.04 — the fit explains 4% of the variance), which is the
signature of a bounded, resetting structure. **D\*-Lite's grows at about
2.7 cells per replanning event** with a fit an order of magnitude tighter
(R² = 0.54), the signature of accumulating state.

The per-*k* rows form an orderly family rather than four unrelated trends: as
*k* rises the intercept rises (32.3 → 51.7 → 69.4 → 75.4) and the slope falls
(4.09 → 3.30 → 2.46 → 2.28). Mean occupancy over the first ten events follows
the intercept (40.8 / 49.8 / 53.5 / 55.5), while by events 30–50 all four
scenarios have converged into a narrow band (179.5–187.9). The mechanism is the
multi-goal initialisation: all `k` goals are seeded with `rhs = 0`
simultaneously, so the initial backward search reaches the start sooner — and
leaves a larger initial working set behind — the more goals there are. With a
single goal D\*-Lite starts lean and fills the maze as it explores; with four it
starts fuller and has less room left to grow. **The end state is the same
either way**: D\*-Lite converges on holding roughly 70% of the maze regardless
of *k*, which is why the plateau, and not the goal count, is what determines
its memory cost (§4.3) — and, per the `k = 1` note above, why that scenario's
median peak looks low only because its runs are typically too short to reach
the plateau at all.

A\*'s fits show no comparable structure (slopes 0.24–0.38, R² ≤ 0.07 at every
*k*) — consistent with a search that is rebuilt from nothing at every event and
therefore has no initial state to scale.

The completeness table by event-index bin shows that this growth is not
unbounded but *saturating*:

| Event index | A\* — mean [95% CI] | D\*-Lite — mean [95% CI] |
|---|---|---|
| (0, 10] | 20.8 [20.3, 21.3] (n=1973) | 53.5 [52.2, 54.8] (n=2025) |
| (10, 20] | 34.3 [33.5, 35.1] (n=1744) | 109.3 [107.5, 111.1] (n=1802) |
| (20, 30] | 41.0 [39.9, 42.2] (n=1617) | 165.6 [163.5, 167.7] (n=1558) |
| (30, 40] | 39.6 [38.1, 41.0] (n=1362) | 180.8 [178.8, 182.8] (n=1264) |
| (40, 50] | 40.1 [38.3, 41.8] (n=1000) | 185.1 [182.9, 187.4] (n=708) |
| (50, 60] | 37.8 [35.5, 40.0] (n=616) | 185.0 [182.2, 187.7] (n=377) |
| (60, 70] | 31.7 [29.2, 34.2] (n=331) | 170.3 [166.6, 174.0] (n=207) |
| (70, 80] | 25.4 [23.0, 27.7] (n=148) | 153.8 [148.3, 159.3] (n=66) |
| (80, 87] | 20.9 [16.6, 25.3] (n=31) | 164.0 [159.8, 168.1] (n=21) |

D\*-Lite rises steeply over the first 30 events, plateaus at 180–185 through
event 60, then declines. A\* traces a much smaller version of the same arc
(20.8 → 41.0 → 20.9). The linear fits above therefore describe the *rising*
portion; both algorithms' occupancy turns over in the late-run bins, where the
sample also thins considerably.

---

## 4. Key observations

### 4.1 Planning time grows with goal count for both algorithms

Cumulative planning time grows with `k` for both algorithms: more goals mean
more exploration, more walls discovered, and more replanning events for
*either* algorithm. A\*'s time steps in lockstep with its node counts (§3.1);
D\*-Lite's time also rises with `k` (0.0013 → 0.0034 s, +162%, tracking its own
node growth) rather than staying flat. Time and nodes agree on every ordering
in this dataset, and this growth does not erode D\*-Lite's advantage — the
absolute gap and the 2.6–3.1× ratio over A\* hold at every `k` (§3.1); the
evaluation's total workload increases at every `k`, for both algorithms, not a
shift in how that workload is split between them.

They do disagree on scale, because time per expanded node is not a constant:

| *k* | A\* | D\*-Lite |
|---|---|---|
| 1 | 7.2 µs | 7.0 µs |
| 2 | 7.6 µs | 7.1 µs |
| 3 | 8.3 µs | 7.4 µs |
| 4 | 8.8 µs | 7.8 µs |

Per-node cost drifts upward with *k* for both algorithms — +23% for A\* and +12%
for D\*-Lite across the range. The drift is monotone, present in both
implementations, and correlated with the size of the structures each is
manipulating (longer runs, larger explored maps, bigger priority queues), which
is what one would expect from allocation and cache effects rather than from an
algorithmic change. It is nonetheless enough to inflate A\*'s time ratio relative
to its node ratio by roughly a tenth.

**Node counts should therefore remain the primary cost measure throughout the
final report**, with wall-clock time cited as corroborating rather than
independent evidence — the more so because timing is the one metric in this
census that carries genuine measurement noise (§5.1).

### 4.2 `k` reliably scales exploration effort, not per-goal difficulty

Because placement is nested (§1.3) and each additional goal maximises the
*minimum* detour over the start and every goal already placed, every goal in a
scenario stays comparably deceptive relative to what the robot already knows —
the placement scheme does not get systematically easier or harder to reach as
goals are added, by construction. Adding a goal therefore adds comparably-hard
territory to explore, and every aggregate quantity increases monotonically with
`k` as a direct consequence: replanning events (A\* 25.5 → 58.3), total moves
(86.1 → 236.5), distinct cells visited (54.4 → 130.7), cumulative nodes for both
algorithms, and peak memory for both algorithms. This is `k` doing exactly what
Phase 4's placement scheme is designed for: manufacturing more exploration
effort per additional goal.

What `k` does not do is calibrate the *absolute* difficulty of any individual
goal, since the underlying detour index is a relative, ratio-based score
(§5.3), not an absolute-distance one. The increments between scenarios are
correspondingly uneven — `k = 1 → 2` adds 16.1 replanning events to an A\* run
while `k = 3 → 4` adds only 4.8 — and the spread *within* a fixed `k` is large:
A\*'s cumulative node count has a standard deviation of 107% of its mean at
`k = 1` and 64% at `k = 4` (§3.1). Across the 55 mazes the `k = 1` goal's true
distance from the start ranges from 5 to 107 steps (median 31), and in 26 of
them the selected cell is at Manhattan distance 1 from the start — a
consequence of the placement metric's known bias (§5.3), not of `k` failing to
do its job.

**Any statement in the final report of the form "difficulty increases with `k`"
should therefore be read as "the exploration effort required increases with
`k`, by design"**: `k` reliably orders and scales the total work asked of both
algorithms across the corpus, while the difficulty of any single goal remains
governed by the placement metric discussed in §5.3.

### 4.3 The trade-off reverses on memory

Across the two cost axes D\*-Lite dominates: 2.3–3.0× fewer node expansions,
2.6–3.1× less planning time, fewer replanning events, cheaper per-event repair,
and lower variance across mazes from `k = 2` onward. On the memory axis the
ordering **reverses decisively**: 2.6× higher median peak, 4.0× higher median
average, and 31 runs holding over 90% of the maze resident against zero for A\*.

This is the central finding of the evaluation and the reason all three axes were
measured. Neither algorithm dominates outright. The choice is governed by which
resource is scarce: on a platform where memory is the binding constraint — which
is the realistic case for micromouse-class embedded hardware — A\*'s bounded,
resetting footprint is the safer design, even at 2.5× the search cost. Where
planning latency is binding, D\*-Lite is clearly preferable.

The `k`-dependence qualifies *how much* is at stake rather than which way the
comparison points. At `k = 1` the memory penalty is at its mildest (1.73× median
peak), because short runs end before D\*-Lite's working set saturates; from
`k = 2` on, once runs are long enough to reach the plateau, it settles at
2.4–2.9× and stops responding to further goals. D\*-Lite's memory cost is
therefore driven by run *length*, not by mission complexity — and any mission
long enough to be interesting pays it in full.

### 4.4 The two algorithms travel comparable distances, with no consistent winner

D\*-Lite's planning advantage does not translate into shorter paths. It records
**fewer** total moves than A\* at `k = 1` (83.5 versus 86.1, −3.1%) and **more**
at every higher *k* (242.6 versus 236.5 at `k = 4`, +2.6%; +6.6% at `k = 2`,
+4.2% at `k = 3`). Across the 220 matched pairs it travels further in 97 cases,
shorter in 88, and identically in 35 — no systematic ordering.

Both algorithms plan optimally with respect to their current knowledge, so this
is not a path-quality deficit in the usual sense; it reflects different
tie-breaking among equal-cost paths, which sends the two robots down different
corridors and hence exposes them to different walls. The practical consequences
are that D\*-Lite's planning advantage is neither reinforced nor meaningfully
offset by execution distance, and that **the two algorithms' replanning-event
counts are not directly comparable as a measure of "how much the world surprised
them"** — they explored different parts of the maze.

---

## 5. Caveats

### 5.1 The dataset is a census, not a sample

All 440 runs are deterministic and exhaustive over the `(algorithm, maze, k)`
grid — each combination appears exactly once, and re-running any of them
reproduces identical values for every non-timing metric (moves, replanning
events, nodes expanded, memory occupancy). There is no sampling variability to
estimate, so **hypothesis tests, p-values, and resampling do not apply** and none
are reported.

Every error bar and confidence interval in this report is therefore
**descriptive**: it summarises spread within the observed data — across mazes in
§3.1, across replanning events in §3.2, across runs in §3.3 — and supports no
inferential claim about mazes outside this 55-maze corpus. A statement such as
"the intervals do not overlap" means the observed distributions are separated in
this corpus, not that a difference has been established at some significance
level.

Wall-clock timings are the sole exception: they carry genuine measurement noise,
and §4.1 documents the systematic part of it (a per-node cost that drifts with
scenario size).

### 5.2 Metric definitions constrain interpretation

`memory_occupancy` is a **working-set** count — cells currently relevant to the
search — not a raw allocation measurement. For D\*-Lite it is a lower bound on
the cumulative footprint (§3.3): the comparison therefore errs on the side of
*understating* D\*-Lite's memory disadvantage, never overstating it.

Two restrictions were applied to keep aggregates meaningful and should be stated
wherever the corresponding figures are reproduced:

* the per-event cost regressions use `residual_distance ≤ 30`, covering 99.3% of
  events; the excluded tail bins are reported in full but are too sparse (13–42
  events) to support a fit;
* the memory-trend figures use the central 95% of runs by length (3 to 88
  events, 420 of 440 runs), bounding the survivorship bias that would otherwise
  arise from averaging across runs of very different durations.

Both are documented in the notebook and neither changes the direction of any
result.

### 5.3 The detour index measures deception, not distance

The goal-placement metric normalises the true in-maze distance by the Manhattan
distance, so maximising it rewards **small denominators**. Goals consequently
cluster near the start, and the score measures *relative* deception rather than
*absolute* difficulty. The effect is concentrated in the first goal, which every
scenario shares: in 26 of the 55 mazes the `k = 1` goal sits at Manhattan
distance 1 from the start. In small or open mazes this produces goals that are
formally "hardest" but trivially close — in `museum`, the selected cell is five
real steps from the start, scoring 5.0 only because its Manhattan distance is 1.
The ranking is trustworthy only where the underlying in-maze distance is also
substantial, as in the complex competition mazes (`2015japan`, `2017apec`,
`zigzag`, whose first goals are 75, 99 and 107 real steps away while scoring
25.0, 19.8 and 35.7).

This has three consequences for the final report:

1. **`k` reliably scales exploration effort but does not calibrate the
   difficulty of individual goals** (§4.2) — the uneven steps and large
   within-`k` spread are a direct consequence of the ratio-based scoring
   described above.
2. Cross-maze aggregation mixes genuinely hard scenarios with degenerate ones,
   which is the main reason maze-to-maze dispersion is so large in §3.1 —
   107% of the mean at `k = 1`.
3. The known remedy — scoring by the absolute excess `d_BFS − d_Manhattan`
   instead of the ratio — was identified and deliberately not adopted, since it
   would change which cells are selected and invalidate the entire collected
   corpus. It should be cited as future work rather than presented as an
   oversight.

Full discussion in `docs/detour_metric_limitations.md`.

---

## 6. Summary

Over 440 deterministic runs spanning 55 mazes and four nested goal-count
scenarios, D\*-Lite expands **2.3–3.0× fewer nodes** than A\* and spends
**2.6–3.1× less time planning**, winning 212 and 219 of 220 matched comparisons
respectively. Its advantage decomposes into roughly 15% fewer replanning events
and roughly half the cost per event, and it grows with distance-to-goal:
per-event cost rises at 0.76 nodes per cell of residual distance against A\*'s
2.32, so D\*-Lite's repair is sublinear in distance-to-go where A\*'s search is
not.

That advantage is paid for in memory. D\*-Lite's median peak occupancy is
**2.6× A\*'s** (182.5 versus 69 cells of a 256-cell maze) and its median per-run
average **4.0× A\*'s**; 31 of its 220 runs hold more than 90% of the maze
resident and four hold all of it, while no A\* run exceeds 75%. A\*'s footprint is
flat over a run (0.26 cells/event, R² = 0.04) whereas D\*-Lite's grows at
2.7 cells/event before saturating near 183. The penalty is mildest at `k = 1`
(1.7×), where runs end before the working set saturates, and settles at 2.4–2.9×
from `k = 2` on.

Both algorithms' cumulative planning time scales with `k`: A\*'s steps in
lockstep with its node counts, and D\*-Lite's grows too (+142% from `k = 1` to
`k = 4`, versus A\*'s +103%) rather than staying flat, because more goals mean
more exploration and more replanning for both algorithms — D\*-Lite's
incremental-repair advantage itself does not narrow, and holds at every `k`
(§3.1, §4.1). D\*-Lite's memory occupancy is similarly not strictly monotonic
within a run (26.6% of transitions are decreases): this is the standard
response to a newly discovered wall, correctly reflected by a working-set
metric, and it does not weaken the peak-based comparison used throughout,
which if anything understates D\*-Lite's memory disadvantage (§3.3). A third
qualification applies throughout: `k` reliably scales exploration effort —
every aggregate cost rises monotonically with it — but does not calibrate the
difficulty of any individual goal, since the detour index is a relative, not
absolute, measure (§4.2, §5.3).