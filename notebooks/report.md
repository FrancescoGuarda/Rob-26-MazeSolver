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
| Heuristic | Wall-aware exact distance to the nearest remaining goal under current knowledge, recomputed per event | Manhattan distance to the robot's current position, with a key offset correcting for the moving start |
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

Placement is nested across *k*: the first *L* goals of a *k*-goal scenario are
exactly the goals of the *L*-goal scenario. Scenarios `k = 1, 2, 3, 4` therefore
form a strictly growing goal set rather than four independent configurations.
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
logs contain **21 188 replanning events** (11 462 for A\*, 9 726 for D\*-Lite).

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
that work was in practice* — the two can diverge, and in this dataset they do
(§4.1).

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
`residual_distance ≤ 30`, which contains 21 059 of the 21 188 events (99.4%).
Beyond that threshold, bins hold fewer than 50 events each and their confidence
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
   any averaging. Run lengths vary from 8 to 125 events, so averaging occupancy
   at a given event index across all runs suffers survivorship bias (only the
   longest runs contribute at high indices). One concrete run avoids that. The
   run shown is chosen as the maze whose D\*-Lite peak is closest to the median
   at `k = 4`, so it is typical rather than selected for effect.
2. Per-run peak and mean occupancy pooled over all 220 runs per algorithm.
3. The occupancy trend against event index, broken down by *k*.
4. The same trend pooled across *k*, plus a completeness table by event-index
   bin.

Views 3 and 4 are computed on the central 95% of runs by length (`n_events`
between 11 and 92, retaining 418 of 440 runs), the same dense-band logic applied
to run length instead of distance, which bounds the survivorship effect.

---

## 3. Results

### 3.1 Cumulative cost of replanning

`docs/res/replanning_cost_bars.svg` — mean per algorithm and *k*, error bars ±1
standard deviation across the 55 mazes.

| *k* | A\* nodes | D\*-Lite nodes | A\* time (s) | D\*-Lite time (s) |
|---|---|---|---|---|
| 1 | 752.3 ± 428.0 | 379.1 ± 168.9 | 0.0110 ± 0.0048 | 0.0032 ± 0.0013 |
| 2 | 826.2 ± 597.5 | 339.0 ± 199.8 | 0.0069 ± 0.0035 | 0.0028 ± 0.0019 |
| 3 | 1016.5 ± 559.2 | 437.6 ± 171.2 | 0.0097 ± 0.0042 | 0.0037 ± 0.0018 |
| 4 | 1098.1 ± 697.3 | 442.5 ± 170.6 | 0.0109 ± 0.0055 | 0.0036 ± 0.0014 |

**D\*-Lite expands roughly half to two-fifths of A\*'s nodes**, and the gap widens
with *k*: the A\*/D\*-Lite node ratio is 1.98× at `k = 1`, then 2.44×, 2.32× and
2.48× at `k = 2, 3, 4`. The result is not an artefact of averaging — D\*-Lite
expands fewer nodes in **213 of the 220** matched `(maze, k)` pairs and records
lower planning time in **219 of 220**.

The *shape* of the two node curves differs as expected. A\*'s cumulative count
rises monotonically across *k* (752 → 826 → 1017 → 1098, a 46% increase from
`k = 1` to `k = 4`), consistent with each additional goal adding a further round
of from-scratch searches. D\*-Lite's is markedly flatter and non-monotone
(379 → 339 → 438 → 443, a 17% net increase), consistent with plan repair being
largely insensitive to how many goals remain.

Dispersion is large and mostly reflects maze-to-maze variation: A\*'s standard
deviation across mazes reaches 63% of its mean at `k = 4`. D\*-Lite's is both
absolutely and relatively smaller (39% at `k = 4`), so its cost is not only
lower but more predictable across the corpus.

Two secondary quantities from the same runs sharpen the picture:

* **Replanning events per run** — A\* 55.1 / 41.6 / 53.5 / 58.3 for `k = 1…4`,
  D\*-Lite 45.7 / 35.8 / 46.1 / 49.1. D\*-Lite triggers *fewer* events, in 167 of
  220 matched pairs.
* **Nodes per event** — A\* 13.6 / 19.9 / 19.0 / 18.9, D\*-Lite 8.3 / 9.5 / 9.5 /
  9.0. Roughly a factor of two, stable across *k*.

So D\*-Lite's cumulative advantage decomposes into two independent contributions
of comparable size: it replans about 15% less often, and each replan costs about
half as much.

Wall-clock planning time does **not** reproduce the monotone step pattern of the
node counts for A\* — the `k = 1` value (0.0110 s) exceeds `k = 2` (0.0069 s)
despite 10% fewer nodes. This is a measurement artefact, analysed in §4.1.

### 3.2 Per-event cost versus residual distance

`docs/res/nodes_vs_residual_distance_by_k.svg` (one panel per *k*) and
`docs/res/nodes_vs_residual_distance_aggregate.svg` (pooled).

Linear fits on the dense band (`residual_distance ≤ 30`):

| *k* | A\* slope (R²) | D\*-Lite slope (R²) |
|---|---|---|
| 1 | 2.60 (0.62) | 1.89 (0.30) |
| 2 | 2.12 (0.55) | 0.69 (0.14) |
| 3 | 2.49 (0.61) | 0.86 (0.15) |
| 4 | 2.75 (0.64) | 0.81 (0.15) |
| pooled | 2.49 (0.61) | 0.93 (0.17) |

**A\*'s per-event cost grows about 2.5 nodes for every additional cell of
distance-to-go; D\*-Lite's grows less than 1.** Pooled across all runs, A\*'s
slope is 2.7× D\*-Lite's. The separation is present at every *k* and is widest at
`k = 2…4`, where D\*-Lite's slope falls below 0.9.

The completeness table quantifies the same divergence and shows it *widening*
with distance:

| Residual distance | A\* — mean [95% CI] | D\*-Lite — mean [95% CI] | Ratio | In trend |
|---|---|---|---|---|
| (0, 5] | 6.3 [6.2, 6.4] (n=2662) | 4.2 [4.2, 4.3] (n=2097) | 1.5× | yes |
| (5, 10] | 12.8 [12.6, 12.9] (n=4518) | 6.6 [6.5, 6.8] (n=3718) | 1.9× | yes |
| (10, 15] | 22.2 [21.9, 22.6] (n=2918) | 9.7 [9.4, 10.1] (n=2551) | 2.3× | yes |
| (15, 20] | 36.7 [35.5, 37.9] (n=957) | 15.4 [14.2, 16.6] (n=915) | 2.4× | yes |
| (20, 25] | 59.4 [56.0, 62.7] (n=286) | 24.1 [20.0, 28.2] (n=268) | 2.5× | yes |
| (25, 30] | 93.3 [84.8, 101.8] (n=72) | 35.3 [26.3, 44.3] (n=97) | 2.6× | yes |
| (30, 35] | 122.3 [102.1, 142.6] (n=18) | 54.8 [32.9, 76.7] (n=44) | 2.2× | no |
| (35, 40] | 95.4 [77.0, 113.7] (n=14) | 89.5 [63.2, 115.9] (n=19) | 1.1× | no |
| (40, 59] | 94.3 [81.2, 107.4] (n=17) | 84.7 [55.4, 114.0] (n=17) | 1.1× | no |

Within the dense band the ratio climbs steadily from 1.5× to 2.6×: **the further
the goal, the more D\*-Lite saves per decision.** The three excluded bins hold 14
to 44 events each; their means stop increasing and their intervals overlap
heavily, which is exactly the instability the `≤ 30` cutoff was introduced to
exclude. They should not be read as evidence that the gap closes at long range.

A third figure of merit makes the difference dimensionless. Expressing each
event's cost as nodes expanded per unit of residual distance, A\* records a
median of **1.53** and D\*-Lite **0.75**. D\*-Lite therefore expands, on the
typical event, *fewer nodes than there are cells between it and the goal* —
repair cost is sublinear in distance-to-go, whereas A\* pays more than one
expansion per cell of remaining path.

The low R² of D\*-Lite's fits (0.14–0.30, against 0.55–0.64 for A\*) is itself a
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

**Pooled across all 220 runs per algorithm** — `docs/res/memory_peak_distribution.svg`.

| | peak (median) | peak (std) | mean (median) | mean (std) |
|---|---|---|---|---|
| A\* | 66.0 | 35.54 | 28.35 | 8.55 |
| D\*-Lite | 192.0 | 52.15 | 132.98 | 39.84 |

**D\*-Lite's median peak occupancy is 2.9× A\*'s, and its median per-run average
is 4.7× A\*'s.** In relative terms A\* peaks at 25.8% of the maze and D\*-Lite at
75.0%. The gap on the *mean* exceeds the gap on the *peak*, which reflects the
shape difference above: A\* only briefly touches its peak, D\*-Lite sits near
its own for most of the run.

The distributions barely overlap. Across all 220 A\* runs the largest peak
observed is 184 cells, and **no** A\* run exceeds 230 (90% of the maze). Among
D\*-Lite runs, **41 of 220 exceed 230** and **5 reach the full 256** — the entire
maze resident at once — spread over four mazes (`2009japan`, `2012uk-techfest`,
`90tor`, `93apec`). D\*-Lite's memory is bounded only by the maze itself.

Broken down by *k*, the median peaks are:

| *k* | A\* | D\*-Lite | ratio |
|---|---|---|---|
| 1 | 50 | 192 | 3.84× |
| 2 | 63 | 182 | 2.89× |
| 3 | 78 | 188 | 2.41× |
| 4 | 80 | 202 | 2.52× |

A\*'s peak grows steadily with *k* (50 → 80, +60%) while D\*-Lite's is nearly
flat (182–202). The ratio narrows accordingly, but D\*-Lite remains at least 2.4×
higher at every *k*. Notably D\*-Lite is *already* near its ceiling at `k = 1`:
additional goals cost it almost no extra memory, because it is already holding
most of the maze.

**Trend over the run** — `docs/res/memory_vs_event_by_k.svg` (by *k*) and
`docs/res/memory_vs_event_trend.svg` (pooled), on the trimmed run pool.

| *k* | A\* slope (R²) | D\*-Lite slope (R²) | D\*-Lite intercept |
|---|---|---|---|
| 1 | −0.11 (0.02) | 1.14 (0.25) | 137.2 |
| 2 | 0.26 (0.04) | 3.23 (0.59) | 54.2 |
| 3 | 0.25 (0.04) | 2.46 (0.51) | 69.4 |
| 4 | 0.20 (0.04) | 2.28 (0.46) | 75.4 |
| pooled | 0.15 (0.02) | 2.16 (0.41) | 86.1 |

**A\*'s occupancy is statistically flat over the course of a run** (pooled slope
0.15 cells/event, R² = 0.017 — the fit explains under 2% of the variance), which
is the signature of a bounded, resetting structure. **D\*-Lite's grows at about
2.2 cells per replanning event** with a fit an order of magnitude tighter
(R² = 0.41), the signature of accumulating state.

The completeness table by event-index bin shows that this growth is not
unbounded but *saturating*:

| Event index | A\* — mean [95% CI] | D\*-Lite — mean [95% CI] |
|---|---|---|
| (0, 10] | 21.7 [21.2, 22.2] (n=2060) | 73.1 [71.3, 74.8] (n=2120) |
| (10, 20] | 31.1 [30.4, 31.8] (n=1992) | 125.2 [123.3, 127.2] (n=2036) |
| (20, 30] | 35.8 [34.9, 36.8] (n=1855) | 171.3 [169.4, 173.2] (n=1818) |
| (30, 40] | 36.2 [34.9, 37.6] (n=1592) | 183.3 [181.5, 185.1] (n=1491) |
| (40, 50] | 35.7 [34.2, 37.2] (n=1195) | 187.4 [185.3, 189.5] (n=876) |
| (50, 60] | 32.5 [30.6, 34.4] (n=787) | 186.4 [183.8, 188.9] (n=482) |
| (60, 70] | 28.6 [26.5, 30.6] (n=457) | 172.1 [168.8, 175.3] (n=289) |
| (70, 80] | 24.9 [22.8, 27.0] (n=206) | 161.2 [155.9, 166.5] (n=120) |
| (80, 91] | 20.7 [17.3, 24.2] (n=71) | 173.8 [167.9, 179.7] (n=41) |

D\*-Lite rises steeply over the first 30 events, plateaus at 183–187 through
event 60, then declines. A\* traces a much smaller version of the same arc
(21.7 → 36.2 → 20.7). The linear fits above therefore describe the *rising*
portion; both algorithms' occupancy turns over in the late-run bins, where the
sample also thins considerably.

---

## 4. Key observations and anomalies

### 4.1 Wall-clock planning time contradicts the node counts at `k = 1`

Phase 6 of `implementation_roadmap.md` anticipated "stepped bars for A\*, near-flat
for D\*-Lite" on cumulative planning time. The **node counts** match that
prediction; the **times** do not — A\* records 0.0110 s at `k = 1` against
0.0069 s at `k = 2`, despite expanding 10% *fewer* nodes at `k = 1`.

Normalising resolves it. Time per expanded node is:

| *k* | A\* | D\*-Lite |
|---|---|---|
| 1 | 14.6 µs | 8.4 µs |
| 2 | 8.3 µs | 8.3 µs |
| 3 | 9.6 µs | 8.4 µs |
| 4 | 9.9 µs | 8.2 µs |

D\*-Lite's per-node cost is stable to within 3% across all four scenarios, while
A\*'s is 50% higher at `k = 1` than at `k = 2` and then settles into the same
8–10 µs range. A genuine algorithmic effect would not appear in one algorithm at
one scenario and vanish thereafter; a warm-up effect in the first scenario
executed would look exactly like this. **The `k = 1` A\* planning time should be
treated as contaminated by measurement overhead, and node counts should be the
primary cost measure throughout the final report**, with wall-clock time cited
as corroborating rather than independent evidence.

This does not affect the headline conclusion: D\*-Lite is faster in 219 of 220
matched pairs, and the effect is far larger than the artefact.

### 4.2 D\*-Lite's memory occupancy is not monotonically increasing

Phase 6 also anticipated occupancy "monotonically increasing for D\*-Lite". It is
not: **27.4% of consecutive event-to-event transitions in D\*-Lite runs are
decreases** (2 606 of 9 506). In the representative run of §3.3, occupancy falls
23 times over 69 events.

This is standard behaviour, not a defect. A newly sensed wall is modelled as an
edge-cost increase, and D\*-Lite's textbook response to a cost increase is to
reset the affected state's `g` value to infinity and propagate backwards — which
removes those cells from the count of cells holding finite values until they are
re-supported. The metric measures the *live working set*, so it legitimately
shrinks when information is invalidated. (A\* shows decreases even more often,
35.0%, since each event's search is independent of the last.) The full analysis
is in `notebooks/dstar_lite_memory_metric_assessment.md`.

Two consequences for the report. First, the roadmap's expectation should be
restated in terms of the working set rather than monotone growth. Second,
because entries are never physically reclaimed, the true cumulative footprint is
bounded below by the peak — so **the peak-based comparison used throughout §3.3
can only understate D\*-Lite's memory disadvantage, never overstate it**. The
2.4×–3.8× gap is conservative.

### 4.3 `k` is not a monotone difficulty axis

Adding a goal does not reliably make a scenario harder. At `k = 2` both
algorithms record *fewer* replanning events and *fewer* moves than at `k = 1`
(A\*: 41.6 events / 154.9 moves at `k = 2` versus 55.1 / 183.7 at `k = 1`;
D\*-Lite: 35.8 / 165.2 versus 45.7 / 191.8). The same dip appears in cumulative
node counts for D\*-Lite (339 at `k = 2` against 379 at `k = 1`).

The explanation lies in the goal-placement scheme (§1.3, §5.3). Because
scenarios are nested and the detour index rewards small Manhattan denominators,
goals cluster near the start; the second goal can lie *on or near* the route to
the first, so a two-goal tour is not necessarily longer than a one-goal tour.
**Any statement in the final report of the form "difficulty increases with `k`"
must be qualified**: what increases with `k` is the number of goals, not a
measured difficulty. The monotone trends that do hold — A\*'s cumulative node
count, A\*'s peak memory — hold in spite of this, not because of it.

### 4.4 The trade-off reverses on memory

Across the two cost axes D\*-Lite dominates: 2.0–2.5× fewer node expansions,
2.4–3.5× less planning time, fewer replanning events, cheaper per-event repair,
and lower variance across mazes. On the memory axis the ordering **reverses
decisively**: 2.9× higher median peak, 4.7× higher median average, and 41 runs
holding over 90% of the maze resident against zero for A\*.

This is the central finding of the evaluation and the reason all three axes were
measured. Neither algorithm dominates outright. The choice is governed by which
resource is scarce: on a platform where memory is the binding constraint — which
is the realistic case for micromouse-class embedded hardware — A\*'s bounded,
resetting footprint is the safer design, even at 2.5× the search cost. Where
planning latency is binding, D\*-Lite is clearly preferable.

The `k`-dependence sharpens this: A\*'s peak memory grows 60% from `k = 1` to
`k = 4` while D\*-Lite's stays flat at 182–202. D\*-Lite is already near its
ceiling with a single goal, so its memory cost does not scale with mission
complexity — it is simply high from the outset.

### 4.5 D\*-Lite plans less but travels slightly further

D\*-Lite records **more** total moves than A\* at every *k* (242.6 versus 236.5
at `k = 4`, +2.6%; the same ordering holds at `k = 1, 2, 3`) and visits slightly
more distinct cells. Across matched pairs it travels further in 107 cases,
shorter in 88, and identically in 25.

Both algorithms plan optimally with respect to their current knowledge, so this
is not a path-quality deficit in the usual sense; it reflects different
tie-breaking among equal-cost paths, which sends the two robots down different
corridors and hence exposes them to different walls. The practical consequence
is that D\*-Lite's planning advantage is partially offset by a small execution
penalty, and that **the two algorithms' replanning-event counts are not directly
comparable as a measure of "how much the world surprised them"** — they explored
different parts of the maze.

### 4.6 D\*-Lite's memory profile is structurally different at `k = 1`

At `k = 1`, D\*-Lite's occupancy trend has a high intercept and a shallow slope
(137.2, 1.14 cells/event); at `k = 2, 3, 4` it has a low intercept and a steep
slope (54–75, 2.3–3.2). Mean occupancy over the first ten events is 115.9 at
`k = 1` against 47.0–52.0 at higher *k*, yet by events 30–50 all scenarios have
converged to 180–195.

The likely mechanism is the multi-goal initialisation: all `k` goals are seeded
simultaneously, so the initial backward search terminates as soon as it reaches
the start from the *nearest* of them. With one goal that initial search must
cover much more of the maze; with four, it stops early. The end state is the
same either way — D\*-Lite converges on holding roughly three-quarters of the
maze regardless of *k* — but it arrives there by a different route. Worth a
sentence in the final report, since it explains why the `k = 1` panel of
`docs/res/memory_vs_event_by_k.svg` looks unlike the other three.

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
and §4.1 documents a case where that noise is large enough to distort a trend.

### 5.2 Metric definitions constrain interpretation

`memory_occupancy` is a **working-set** count — cells currently relevant to the
search — not a raw allocation measurement. For D\*-Lite it is a lower bound on
the cumulative footprint (§4.2): the comparison therefore errs on the side of
*understating* D\*-Lite's memory disadvantage, never overstating it.

Two restrictions were applied to keep aggregates meaningful and should be stated
wherever the corresponding figures are reproduced:

* the per-event cost regressions use `residual_distance ≤ 30`, covering 99.4% of
  events; the excluded tail bins are reported in full but are too sparse (14–44
  events) to support a fit;
* the memory-trend figures use the central 95% of runs by length (11 to 92
  events, 418 of 440 runs), bounding the survivorship bias that would otherwise
  arise from averaging across runs of very different durations.

Both are documented in the notebook and neither changes the direction of any
result.

### 5.3 The detour index measures deception, not distance

The goal-placement metric normalises the true in-maze distance by the Manhattan
distance, so maximising it rewards **small denominators**. Goals consequently
cluster near the start, and the score measures *relative* deception rather than
*absolute* difficulty. In small or open mazes this produces goals that are
formally "hardest" but trivially close — in `museum`, the selected goal is five
real steps from the start, scoring highly only because its Manhattan distance is
1. The ranking is trustworthy only where the underlying in-maze distance is also
substantial, as in the complex competition mazes (`2015japan`, `2017apec`, with
true distances of 75–99 steps).

This has three consequences for the final report:

1. **`k` cannot be presented as a calibrated difficulty axis** — §4.3 shows the
   empirical non-monotonicity this predicts.
2. Cross-maze aggregation mixes genuinely hard scenarios with degenerate ones,
   which is part of why maze-to-maze dispersion is so large in §3.1.
3. The known remedy — scoring by the absolute excess `d_BFS − d_Manhattan`
   instead of the ratio — was identified and deliberately not adopted, since it
   would change which cells are selected and invalidate the entire collected
   corpus. It should be cited as future work rather than presented as an
   oversight.

Full discussion in `docs/detour_metric_limitations.md`.

---

## 6. Summary

Over 440 deterministic runs spanning 55 mazes and four goal-count scenarios,
D\*-Lite expands **2.0–2.5× fewer nodes** than A\* and spends **2.4–3.5× less
time planning**, winning 213 and 219 of 220 matched comparisons respectively.
Its advantage decomposes into roughly 15% fewer replanning events and roughly
half the cost per event, and it widens with distance-to-goal: per-event cost
grows at 0.93 nodes per cell of residual distance against A\*'s 2.49, so
D\*-Lite's repair is sublinear in distance-to-go where A\*'s search is not.

That advantage is paid for in memory. D\*-Lite's median peak occupancy is
**2.9× A\*'s** (192 versus 66 cells of a 256-cell maze) and its median per-run
average **4.7× A\*'s**; 41 of its 220 runs hold more than 90% of the maze
resident and five hold all of it, while no A\* run exceeds 72%. A\*'s footprint is
flat over a run (0.15 cells/event, R² = 0.02) whereas D\*-Lite's grows at
2.2 cells/event before saturating near 185.

Two roadmap expectations were not met and are explained rather than
contradicted: A\*'s wall-clock planning time does not step monotonically with *k*
(a warm-up artefact at `k = 1`, §4.1), and D\*-Lite's memory occupancy is not
monotonically increasing (standard cost-increase propagation, §4.2). A third
qualification applies throughout: *k* counts goals but does not calibrate
difficulty, a direct consequence of the detour index's documented bias.
