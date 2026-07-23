# Rob-26-MazeSolver_report outline

Authoritative blueprint for drafting `Rob-26-MazeSolver_report.md`. It fixes the section/subsection structure, the content of each subsection, every figure and table to produce, and the literature to cite. The report itself is a single continuous document (`documentclass: article`, 11pt, a4paper, 1.08 line stretch, single column, 2 cm margins, per the existing front matter already in `Rob-26-MazeSolver_report.md`); **only the material under "Report Body" counts against the 5-page limit** — the title page, table of contents, and the references page are additional.

Central theme: the experimental comparison of `AStarExplorer` and `DStarLiteExplorer` as goal-directed maze-exploration strategies. Four supporting contributions are woven through the methodology and evaluation rather than given their own standalone sections: the headless MMS extension (`SimAPI`) that made a 440-run batch campaign possible; native multi-goal exploration; the shared `BaseAlgorithm`/`BaseAPI` interface; and the detour-index metric for manufacturing exploration difficulty (single- and multi-goal). Implementation detail not needed to understand these contributions (file layout, CLI flags, GUI rendering specifics) is left out of the report entirely — that material already lives in the repository's own `README.md` files and is not repeated here.

> **Licensing Note**
>
> *This work is licensed under the Creative Commons Attribution 4.0 International License. Copyright for components of this work owned by other than the authors and the University of Brescia must be honoured.*

## Page budget

| Section | Target length | Carries |
|---|---|---|
| 1. Introduction | ~1/3 page | Motivation, contributions summary |
| 2. Background | ~1/2 page | Problem framing, algorithmic background, citations |
| 3. Methodology | ~1.5 pages | System design, both algorithms, detour index |
| 4. Experimental Evaluation | ~2.2 pages | Setup + all results — the centre of gravity of the report |
| 5. Conclusions and Future Work | ~1/3 page | Findings, limitations, next steps |
| **Total body** | **~5 pages** | |
| Acknowledgements | few lines, end of body | boilerplate (below) |
| References | 1 page, own page | full list at the end |

## Planned figures

| # | Figure | Used in |
|---|---|---|
| F1 | Detour-index score-map evolution: for one representative maze, the score map that `place_goals` maximizes at each step, k = 1..4 | §3.5 or §4.2 |
| F2 | Grouped bar chart: cumulative nodes expanded and cumulative planning time, A\* vs D\*-Lite, grouped by goal-count scenario (mean across mazes, std-dev error bars) | §4.4 |
| F3 | Regression plot: nodes expanded vs. residual distance to goal, per replanning event, faceted by goal-count scenario (k=1..4), linear trend with 95% band | §4.5 |
| F4 | Same regression, all scenarios pooled into one pair of trend lines | §4.5 |
| F5 *(pending)* | Memory occupancy of search structures over successive replanning events — one representative run per algorithm, expected bounded/resetting for A\* vs. monotonically increasing for D\*-Lite | §4.6 |

Keep to these five; a system-architecture diagram was considered for §3.1 but is optional and should be cut first if space runs short — the interface is adequately described in prose.

## Planned tables

| # | Table | Used in |
|---|---|---|
| T1 | Top-5 hardest mazes per goal-count scenario, ranked by mean/total detour of the placed goal set | §4.2 |
| T2 | Aggregate results by algorithm × goal-count scenario: goal-reached rate, avg. total moves, avg. replanning events, avg. cumulative planning time | §4.3 |
| T3 | Mean nodes expanded with 95% CI, by residual-distance bin and algorithm, flagging which bins fall inside the fitted trend region | §4.5 |

---

## Report Body

### 1. Introduction
- The classical Micromouse paradigm is *full exploration*: map the maze, then solve it offline. This project targets a different, goal-directed paradigm: the start and goal cells are known in advance, and the agent must reach the goal while minimizing exploration effort — full mapping is neither required nor rewarded.
- This is framed as **online replanning under the freespace assumption**: every unexplored cell is optimistically treated as passable until a wall is sensed; the agent alternates sensing, (re)planning, and acting, replanning whenever new information invalidates its current plan.
- One sentence on scope: two replanning search strategies, A\* and D\*-Lite, are implemented against this framework and evaluated head-to-head, rather than the wall-following/flood-fill/A\* comparison of the original proposal — the freespace/replanning framing subsumes wall-following and flood-fill as special or degenerate cases, so the sharper comparison is between a from-scratch replanner and an incremental one.
- Contributions (bullet list, kept tight):
  - A shared exploration-algorithm interface (`BaseAlgorithm`/`BaseAPI`) that made A\* and D\*-Lite drop-in implementations of the same sense-plan-act loop.
  - A headless extension of the MMS simulator interface enabling fully automated, GUI-free batch evaluation at scale.
  - Native multi-goal exploration, treated as a first-class capability rather than a special case of single-goal search.
  - The **detour index**, a metric for manufacturing exploration difficulty by goal placement rather than by maze selection, extended from a single reference point to nested multi-goal scenarios.
  - A 440-run controlled experimental comparison of A\* and D\*-Lite, evaluating D\*-Lite specifically as an *incremental* heuristic search algorithm that reuses prior search effort across replanning events, not merely as an alternative exploration heuristic.

*(No figures/tables.)*

### 2. Background
- **The Micromouse problem and the MMS simulator:** brief description of the Micromouse competition setting; the `mms` simulator's text-based `stdin`/`stdout` wall-sensing and movement protocol; why it was adopted (standard interface, GUI visualization, existing Python bindings).
- **Search under partial observability:** the freespace/optimistic assumption that underlies incremental replanning approaches to navigation in partially known environments; A\* as the classical heuristic best-first search baseline; D\*-Lite as a heuristic search algorithm designed explicitly to reuse previous search results when the environment (or the agent's knowledge of it) changes slightly, rather than resolving the whole problem again.
- One or two sentences situating this project relative to prior Micromouse-specific approaches consulted during design (potential-value flood fill; simulated 3-D Micromouse environments) — enough to acknowledge the related work without dwelling on it.

*(No figures/tables; keep this section to short paragraphs — it exists to give the reader vocabulary for §3–4, not to survey the field.)*

### 3. Methodology
#### 3.1 A common interface for exploration algorithms
- `BaseAPI`: an abstract contract (wall sensing, movement, display, reset) that decouples an exploration algorithm from its I/O backend.
- `BaseAlgorithm`: the shared sense→plan→act→log loop, goal resolution (single goal, explicit multi-goal, random goals, or a default centre area), and metrics hooks common to every concrete algorithm.
- Consequence worth stating explicitly: adding a third algorithm to this project would mean implementing one class against this interface, with no changes to execution, logging, or evaluation tooling.

#### 3.2 Headless execution for automated evaluation
- A second, headless backend (`SimAPI`) implements the same `BaseAPI` contract entirely in-memory, without the MMS GUI process.
- This is what makes a large deterministic batch campaign (hundreds of runs) practical; the identical algorithm code path was cross-checked against the real MMS GUI to confirm behavioural equivalence between backends.

#### 3.3 Exploration algorithms
- **A\*:** replans from scratch after every newly discovered wall that invalidates the current plan; heuristic is a wall-aware shortest-known-distance estimate by default (a straight-line variant is also available); multi-goal handled by greedily re-targeting the nearest remaining goal after each one is reached.
- **D\*-Lite:** maintains incremental search state (`g`/`rhs` values in a priority queue) across the entire episode; on a newly discovered wall, only the affected region of this state is repaired, rather than the whole plan being recomputed; multi-goal reached by reinitializing the search at the newly reached goal while retaining the reusable state built up over the rest of the maze.
- Flag explicitly: this reuse of prior search state is the property placed under direct experimental scrutiny in §4.4–4.5 — D\*-Lite is evaluated not just as an alternative path to the goal, but as an incremental-search algorithm whose central claimed advantage (cheaper repair vs. cheaper-from-scratch replanning) is directly measurable.

#### 3.4 Multi-goal exploration
- Goals may be an explicit list, a random sample, or (for evaluation, see §3.5) an automatically placed set; both algorithms consume whichever set of goals is given identically, visiting them in a greedy nearest-goal order.
- Framed as a capability of the shared interface (§3.1), not an algorithm-specific extension.

#### 3.5 Modeling exploration difficulty: the detour index
- Definition: for a reference cell and a candidate cell, the detour index is the ratio of true in-maze (BFS) distance to straight-line (Manhattan) distance — a cell that "looks" close but is actually far behind walls scores highest, and is precisely the kind of cell that defeats a greedy planner working from a partial map.
- Single-goal placement: the goal is the free cell maximizing detour from the start.
- **Multi-goal extension:** each additional goal maximizes the *minimum* detour over the start and every previously placed goal, so a k-goal scenario stays jointly deceptive rather than just individually so; scenarios are nested (a k-goal scenario's first *j* goals equal the *j*-goal scenario's goals).
- This metric, not maze selection, is the difficulty axis of the whole experimental campaign — every maze in the corpus is used at every difficulty level (§4.1).
- One sentence flagging a known bias (goals cluster near the start because the ratio rewards small denominators) — full discussion deferred to §5.

**Figure F1** goes here (or is deferred to §4.2, whichever a first draft finds reads better — it illustrates both the metric's mechanism and the experimental corpus, so it can anchor either section).

### 4. Experimental Evaluation
#### 4.1 Setup
- Corpus: 55 standard Micromouse competition mazes, filtered to guarantee full connectivity from the start cell.
- Four goal-count scenarios per maze: k=1 (default 4-cell centre area) and k=2,3,4 (detour-index placement, §3.5).
- Full factorial, headless batch campaign: 2 algorithms × 55 mazes × 4 scenarios = **440 runs**, each producing a JSON log of scalar metrics (moves, distinct cells visited) and per-event replanning records (nodes expanded, planning time, residual distance, memory occupancy).
- Determinism note: every metric except wall-clock time is exactly reproducible run to run: the campaign is a full census of the (algorithm × maze × scenario) space, not a sample, so the analysis is descriptive/paired rather than inferential (no p-values or resampling) — planning *time* is the only quantity with real measurement noise, summarized via median across repeats.
- Manual cross-checks: at least two runs per algorithm reproduced in the live MMS GUI, confirming the headless results are representative of real simulator behaviour.

#### 4.2 Goal-placement difficulty across the maze corpus
- How detour-based placement concentrates difficulty differently across the corpus as k grows; which mazes are hardest at each scenario and why (walls forcing long detours around cells that are Manhattan-close).
- **Table T1**, **Figure F1** (if not already placed in §3.5).

#### 4.3 Exploration performance: A\* vs D\*-Lite
- Goal-reached rate across all 440 runs (state the actual figure once computed — expected at or near 100%, confirm from the batch summary before drafting).
- Total moves and distinct cells visited, by algorithm and goal-count scenario; whether — and how much — either algorithm's exploration footprint grows with k.
- **Table T2.**

#### 4.4 Replanning cost: nodes expanded vs. wall-clock planning time
- Central counter-intuitive finding: D\*-Lite consistently expands **fewer** nodes per replanning event than A\* at every k, yet accumulates **more** wall-clock planning time — attributed to the higher constant overhead of its priority-queue machinery per node processed, versus A\*'s simpler from-scratch recomputation.
- Methodological point worth stating plainly: nodes expanded is the algorithm-intrinsic, implementation-independent measure; wall-clock time is useful but implementation- and hardware-dependent, and the two need not agree.
- **Figure F2.**

#### 4.5 Search-cost scaling: evidence for incremental reuse
- Nodes expanded vs. residual distance to the goal, per replanning event: A\*'s slope is steeper than D\*-Lite's at every k, and the gap widens as the residual distance grows.
- Interpretation: this is direct empirical support for D\*-Lite's central algorithmic claim (§3.3) — its cost scales with the *size of the affected region* after a change, not with the size of the whole remaining problem, whereas A\*'s from-scratch replan cost grows with the full remaining search space.
- **Figures F3, F4; Table T3.**

#### 4.6 Memory footprint of search structures *(pending)*
- Not yet produced — flagged here so the person drafting this subsection knows to generate it before submission (source data: the `memory_occupancy` field already present in every replanning-event record).
- Expected pattern to check for: A\*'s open/closed sets reset at every replan (bounded, sawtooth); D\*-Lite's `g`/`rhs`/priority-queue state persists and is expected to grow monotonically across the episode.
- **Figure F5.**

### 5. Conclusions and Future Work
- Recap the central result in one or two sentences: both algorithms reliably solve the full corpus across every goal-count scenario; A\* and D\*-Lite trade off per-event node-expansion efficiency against wall-clock overhead, with D\*-Lite's incremental repair empirically validated as the more efficient strategy in the algorithm-intrinsic measure, and increasingly so as replanning distance grows.
- Restate the supporting contributions in one line each (interface, headless execution, multi-goal, detour index) as reusable project infrastructure beyond this specific comparison.
- Limitations: the detour index's bias toward the maze start (goals cluster near the start because the ratio rewards small denominators — see the dedicated discussion in the repository if more depth is wanted than fits here); wall-clock planning time as a hardware/implementation-dependent measure; the memory-footprint analysis (§4.6) still pending at time of writing.
- Future work: additional algorithms via the shared interface (e.g. weighted or anytime variants); a start-proximity correction to the detour index; extending the evaluation to larger goal counts or non-uniform edge costs.

*(No figures/tables.)*

---

> **Acknowledgements**
>
> The authors would like to acknowledge the use of various large language model (LLM)-based AI tools during the preparation of this work. These tools were employed solely to improve the clarity and readability of the written text and to assist in the development and debugging of the project's source code. All technical content, design decisions, and conclusions remain the sole responsibility of the authors.

---

## References

Own page, full bibliographic entries (not just the short in-text form used above). Suggested list:

- **MMS simulator:** mackorone, *mms — A Micromouse Simulator*, v1.2.0, MIT License, GitHub (2024). [`CITATIONS.bib`](../CITATIONS.bib)
- **Maze corpus:** J. Weisberg, *Micromouse Maze Collection*, tcp4me.com. [`CITATIONS.bib`](../CITATIONS.bib)
- **A\*:** P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107, 1968.
- **D\* / the freespace assumption:** A. Stentz, "Optimal and Efficient Path Planning for Partially-Known Environments," *Proc. IEEE International Conference on Robotics and Automation (ICRA)*, 1994.
- **D\*-Lite:** S. Koenig and M. Likhachev, "D\* Lite," *Proc. AAAI/IAAI*, 476–483, 2002; and S. Koenig and M. Likhachev, "Fast Replanning for Navigation in Unknown Terrain," *IEEE Transactions on Robotics*, 21(3), 354–363, 2005.
- **Detour index / route factor:** M. Barthélemy, "Spatial Networks," *Physics Reports*, 499(1–3), 1–101, 2011; M. T. Gastner and M. E. J. Newman, "The Spatial Structure of Networks," *European Physical Journal B*, 49(2), 247–252, 2006.
- **Related Micromouse-algorithm background** (consulted during design, cite if referenced in §2): the potential-value Tremaux-algorithm article and the Unity-based 3-D Micromouse simulator article listed in `docs/notes.md`'s reference material.
- *(Optional, if the difficulty-metric lineage from the original proposal is discussed:)* the closed-list-cardinality maze-difficulty article cited in `Rob_26_proposal.md`, noted as the inspiration later superseded by the detour index.
