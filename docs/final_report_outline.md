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
| 2. Background | ~1/2 page | Problem framing, algorithmic background, walls representation, one small figure |
| 3. Methodology | ~1.6 pages | System design, both algorithms, multi-goal handling, detour index |
| 4. Experimental Evaluation | ~2 pages | Setup + results — the centre of gravity of the report |
| 5. Conclusions and Future Work | ~1/3 page | Findings, limitations, next steps |
| **Total body** | **~5 pages** | |
| Acknowledgements | few lines, end of body | boilerplate (below) |
| References | 1 page, own page | full list at the end |

## Planned figures

| # | Figure | Used in |
|---|---|---|
| F1 | Screenshot of the `mms` simulator GUI during a typical run, annotated to identify its main interface elements | §2 |
| F2 | Detour-index score-map evolution: for one representative maze, the score map that `place_goals` maximizes at each step, k = 1..4 | §3.5 or §4.2 |
| F3 | Grouped bar chart: cumulative nodes expanded and cumulative planning time, A\* vs D\*-Lite, grouped by goal-count scenario (mean across mazes, std-dev error bars) | §4.3 |
| F4 | Regression plot: nodes expanded vs. residual distance to goal, per replanning event, faceted by goal-count scenario (k=1..4), linear trend with 95% band | §4.4 |
| F5 | Same regression, all scenarios pooled into one pair of trend lines | §4.4 |
| F6 *(pending)* | Memory occupancy of search structures over successive replanning events — one representative run per algorithm, expected bounded/resetting for A\* vs. monotonically increasing for D\*-Lite | §4.5 |

Keep to these six; a system-architecture diagram was considered for §3.1 but is optional and should be cut first if space runs short — the interface is adequately described in prose. F1 is a small inset (not full-width) so §2 stays within its half-page budget.

## Planned tables

| # | Table | Used in |
|---|---|---|
| T1 | Top-5 hardest mazes per goal-count scenario, ranked by mean/total detour of the placed goal set | §4.2 |
| T2 | Mean nodes expanded with 95% CI, by residual-distance bin and algorithm, flagging which bins fall inside the fitted trend region | §4.4 |

Absolute exploration-efficiency metrics (total moves, distinct cells visited) are not analyzed in this report — they belonged to the original proposal's wall-following/flood-fill/A\* comparison and are not the current focus; no table is planned for them.

---

## Report Body

### 1. Introduction
- The project addresses goal-directed maze exploration as an **online replanning problem**, framed in the classical Micromouse paradigm, where the agent knows the start and goal cells in advance but not the maze layout, and must interleave sensing, (re)planning, and acting to reach the goal while minimizing exploration effort.
- This is framed under the **freespace assumption** — every unexplored cell is optimistically treated as passable until a wall is sensed — with a replanning event triggered whenever new information invalidates the current plan; the report's central question is which replanning strategy handles this loop most effectively.
- Two replanning search strategies — A\* (full replanning) and D\*-Lite (incremental replanning) — are implemented against this shared framework and evaluated head-to-head, isolating the effect of *how* a plan is repaired when a new wall is discovered.
- Contributions (bullet list, kept tight):
  - A shared exploration-algorithm interface (`BaseAlgorithm`/`BaseAPI`) that made A\* and D\*-Lite drop-in implementations of the same sense-plan-act loop.
  - A headless extension of the MMS simulator interface enabling fully automated, GUI-free batch evaluation at scale.
  - Native multi-goal exploration, treated as a first-class capability rather than a special case of single-goal search.
  - The **detour index**, a metric for manufacturing exploration difficulty by goal placement rather than by maze selection, extended from a single reference point to nested multi-goal scenarios.
  - A 440-run controlled experimental comparison of A\* and D\*-Lite, evaluating D\*-Lite specifically as an *incremental* heuristic search algorithm that reuses prior search effort across replanning events, not merely as an alternative exploration heuristic.

*(No table; small annotated GUI screenshot lives in §2, not here.)*

### 2. Background
- **The Micromouse problem and the MMS simulator:** brief description of the Micromouse competition setting; the `mms` simulator's text-based `stdin`/`stdout` wall-sensing and movement protocol; why it was adopted (standard interface, GUI visualization, existing Python bindings).
- **Search under partial observability:** the freespace/optimistic assumption that underlies incremental replanning approaches to navigation in partially known environments; A\* as the classical heuristic best-first search baseline; D\*-Lite as a heuristic search algorithm designed explicitly to reuse previous search results when the environment (or the agent's knowledge of it) changes slightly, rather than resolving the whole problem again.
- **Walls representation:** the bitmask encoding of walls in each cell (one bit per cardinal direction) and the corresponding integer values used throughout the simulator and the internal maze map.

*(keep this section to short paragraphs — it exists to give the reader vocabulary for §3–4, not to survey the field.)*

**Figure F1**: a small annotated screenshot of the `mms` GUI during a typical run, identifying the maze grid, wall/cell decoration, and stats panel — gives the reader a mental picture of the environment described above.

### 3. Methodology
#### 3.1 A common interface for exploration algorithms
- `BaseAPI`: an abstract contract (wall sensing, movement, display, reset) that decouples an exploration algorithm from its I/O backend.
- `BaseAlgorithm`: the shared sense→plan→act→log loop, goal resolution (single goal, explicit multi-goal, random goals, or a default centre area), and metrics hooks common to every concrete algorithm.
- Consequence worth stating explicitly: adding a third algorithm to this project would mean implementing one class against this interface, with no changes to execution, logging, or evaluation tooling.

#### 3.2 Headless execution for automated evaluation
- A second, headless backend (`SimAPI`) implements the same `BaseAPI` contract entirely in-memory, without the MMS GUI process.
- This is what makes a large deterministic batch campaign (hundreds of runs) practical; the identical algorithm code path was cross-checked against the real MMS GUI to confirm behavioural equivalence between backends.

#### 3.3 Exploration algorithms
- **A\*:** replans from scratch after every newly discovered wall that invalidates the current plan; movement cost is 1 per traversable edge; multi-goal handled by greedily re-targeting the nearest remaining goal after each one is reached.
- **D\*-Lite:** maintains incremental search state (`g`/`rhs` values in a priority queue) across the entire episode; on a newly discovered wall, only the affected region of this state is repaired, rather than the whole plan being recomputed; multi-goal reached by reinitializing the search at the newly reached goal while retaining the reusable state built up over the rest of the maze.
- **Heuristic:** The experimental campaign (§4) fixes **both** algorithms to the Manhattan heuristic: it is admissible and consistent, costs O(1) per query rather than a fresh graph search (for wall-aware **shortest-known-distance**), and matches D\*-Lite's own incremental `km`-based heuristic update — so neither algorithm's node or time counts are confounded by heuristic-recomputation overhead, keeping the comparison isolated to *how* each algorithm repairs its plan.
- Flag explicitly: the reuse of prior search state is the property placed under direct experimental scrutiny in §4.3–4.4 — D\*-Lite is evaluated not just as an alternative path to the goal, but as an incremental-search algorithm whose central claimed advantage (cheaper repair vs. cheaper-from-scratch replanning) is directly measurable.

#### 3.4 Multi-goal exploration
- Goals may be an explicit list, a random sample, or an automatically placed set (§3.5); both algorithms visit them in a greedy nearest-goal order, though the search mechanics that produce that order differ between them (below).
- Framed as a capability of the shared interface (§3.1), not an algorithm-specific extension.
- **Structural difference between the two algorithms:** A\*'s search is rooted at the current position and terminates at the first goal cell popped from its open list, discarding all partial information about the other remaining goals once that replanning episode ends — every episode starts over. D\*-Lite's search is rooted at the *entire* remaining goal set simultaneously (every unreached goal is initialised with `rhs = 0`), so once the currently nearest goal is reached, the `g`/`rhs` state already built up toward the other, still-unreached goals remains valid and is reused directly in the next planning cycle rather than discarded — an amplification, specific to the multi-goal setting, of the general reuse advantage described in §3.3.
- **Default scenario:** with no goals specified, both algorithms fall back to the standard Micromouse competition convention — a 4-cell centre goal area, any one of which ends the run on first arrival — rather than a genuine multi-goal visitation; this is the project's k=1 baseline in §4.

#### 3.5 Modeling exploration difficulty: the detour index
- Definition: for a reference cell and a candidate cell, the detour index is the ratio of true in-maze (BFS) distance to straight-line (Manhattan) distance — a cell that "looks" close but is actually far behind walls scores highest, and is precisely the kind of cell that defeats a greedy planner working from a partial map.
- Single-goal placement: the goal is the free cell maximizing detour from the start.
- **Multi-goal extension:** each additional goal maximizes the *minimum* detour over the start and every previously placed goal, so a k-goal scenario stays jointly deceptive rather than just individually so; scenarios are nested (a k-goal scenario's first *j* goals equal the *j*-goal scenario's goals).
- This metric, not maze selection, is the difficulty axis of the whole experimental campaign — every maze in the corpus is used at every difficulty level (§4.1).
- One sentence flagging a known bias (goals cluster near the start because the ratio rewards small denominators) — full discussion deferred to §5.

**Figure F2** goes here (or is deferred to §4.2, whichever a first draft finds reads better — it illustrates both the metric's mechanism and the experimental corpus, so it can anchor either section).

### 4. Experimental Evaluation
#### 4.1 Setup
- Corpus: 55 standard Micromouse competition mazes, filtered to guarantee full connectivity from the start cell (so every goal is reachable).
- Four goal-count scenarios per maze: k=1 (default 4-cell centre area) and k=2,3,4 (detour-index placement, §3.5).
- Full factorial, headless batch campaign: 2 algorithms × 55 mazes × 4 scenarios = **440 runs**, each producing a JSON log of scalar metrics and per-event replanning records (nodes expanded, planning time, residual distance, memory occupancy).
- Determinism note: every metric except wall-clock time is exactly reproducible run to run: the campaign is a full census of the (algorithm × maze × scenario) space, not a sample, so the analysis is descriptive/paired rather than inferential (no p-values or resampling) — planning *time* is the only quantity with real measurement noise, summarized via median across repeats.
- Correctness, not a finding: both algorithms are complete searches under the freespace assumption, so on a fully connected maze reaching every goal is guaranteed; all 440 runs did so. This confirms the implementation rather than constituting an experimental result, and is not discussed further.

#### 4.2 Goal-placement difficulty across the maze corpus
- How detour-based placement concentrates difficulty differently across the corpus as k grows; which mazes are hardest at each scenario and why (walls forcing long detours around cells that are Manhattan-close).
- **Table T1**, **Figure F2** (if not already placed in §3.5).

#### 4.3 Replanning cost: nodes expanded and wall-clock planning time
- Central finding: D\*-Lite consistently expands **fewer** nodes per replanning event than A\* at every k, **and** accumulates **less** cumulative wall-clock planning time overall — a consistent win on both the algorithm-intrinsic measure and the implementation-level one, not a trade-off between them.
- Methodological point still worth keeping: nodes expanded is the algorithm-intrinsic, implementation-independent measure; planning time is useful corroboration but remains hardware/implementation-dependent in principle, even though in this campaign it points the same direction.
- **Figure F3.**

#### 4.4 Search-cost scaling: evidence for incremental reuse
- Nodes expanded vs. residual distance to the goal, per replanning event: A\*'s slope is steeper than D\*-Lite's at every k, and the gap widens as the residual distance grows.
- Interpretation: this is direct empirical support for D\*-Lite's central algorithmic claim (§3.3) — its cost scales with the *size of the affected region* after a change, not with the size of the whole remaining problem, whereas A\*'s from-scratch replan cost grows with the full remaining search space.
- **Figures F4, F5; Table T2.**

#### 4.5 Memory footprint of search structures *(pending)*
- Not yet produced — flagged here so the person drafting this subsection knows to generate it before submission (source data: the `memory_occupancy` field already present in every replanning-event record).
- Expected pattern to check for: A\*'s open/closed sets reset at every replan (bounded, sawtooth); D\*-Lite's `g`/`rhs`/priority-queue state persists and is expected to grow monotonically across the episode.
- **Figure F6.**

### 5. Conclusions and Future Work
- Recap the central result in one or two sentences: across the full corpus and every goal-count scenario, both algorithms reliably reach all goals (a correctness guarantee, not a finding — §4.1); on the metrics that matter, D\*-Lite's incremental repair consistently outperforms A\*'s from-scratch replanning, expending fewer nodes per event and less cumulative planning time overall, with the gap widening as replanning distance grows (§4.3–4.4). The memory-footprint comparison (§4.5) remains open at time of writing.
- Restate the supporting contributions in one line each (interface, headless execution, multi-goal, detour index) as reusable project infrastructure beyond this specific comparison.
- Limitations: the detour index's bias toward the maze start (goals cluster near the start because the ratio rewards small denominators — see the dedicated discussion in the repository if more depth is wanted than fits here); wall-clock planning time remains hardware/implementation-dependent in principle, even though it corroborated the node-count conclusion in this campaign; the memory-footprint analysis (§4.5) still pending at time of writing.
- Future work: additional algorithms via the shared interface (e.g. weighted or anytime variants); a start-proximity correction to the detour index.

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
- **Rob-26-MazeSolver repository:** F. Guarda and A. Moro, *Rob-26-MazeSolver*, v1.0.0, MIT License, GitHub (2024). *(entry not yet present in `CITATIONS.bib` — add it there when drafting.)*
