# Limitations of the detour metric for goal placement

Reminder for the report: the metric used to place goals
(`src/goal_placement.py`) has a known bias. It is not a computation bug but an
effect of its definition, and it should be stated explicitly.

## What it measures

For a cell `c` relative to a reference `ref` (the start or an already-placed goal):

    detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)

`d_BFS` is the real in-maze distance (walls respected), `d_Manhattan` is the
straight-line lower bound on a 4-connected grid. A detour ≈ 1 means the cell is
"honest"; a high detour means it **looks close but is far** — exactly the
deceptive cell we want, because it defeats a greedy planner with a partial map.
This is the *route factor / circuity* from spatial-network analysis
(Barthélemy 2011; Gastner & Newman 2006).

As a measure of *deception* the metric is correct and does its job: by
construction it selects the worst cells for the agent's heuristic.

## The problem: bias toward the start

Used as a **function to maximize** (`argmax` over all cells), normalizing by the
Manhattan distance rewards small denominators. As a result the chosen goal almost
always ends up **glued to the start**, where the Manhattan distance is smallest.
Goal 1 on a few mazes (start = `(0,0)`):

| maze       | goal 1 | Manhattan | real BFS | detour |
|------------|--------|-----------|----------|--------|
| 2015japan  | (2,1)  | 3         | 75       | 25.0   |
| 2017apec   | (3,2)  | 5         | 99       | 19.8   |
| 88us       | (1,4)  | 5         | 69       | 13.8   |
| 93apec     | (1,0)  | 1         | 9        | 9.0    |
| museum     | (1,0)  | 1         | 5        | 5.0    |

Two consequences:

1. **Goals cluster near the start** (not just goal 1: even with the combined
   minimum over previous references, for k≥2, they stay in the corner).
2. **In small/open mazes the difficulty is illusory.** In `museum` the "hardest"
   goal is `(1,0)`, only **5 real steps** away: a high detour purely because the
   Manhattan distance is 1, but no actual challenge. The difficulty ranking is
   reliable **only when the BFS distance is also substantial** (complex mazes such
   as 2015japan/apec, with BFS 75–99).

In short: the detour measures **relative** deception, not **absolute** difficulty.

## Why we keep it anyway

- As a measure of deception it is correct and serves the purpose: the agent's
  behaviour is clearly visible.
- For this specific application (adversarial goal placement in micromouse mazes)
  the literature is sparse: we are adapting an existing metric, not ignoring an
  established standard.
- Changing it would invalidate the logs already collected and force us to re-run
  the experiments, with no benefit proportional to the project's scope.

## Rejected alternative (in case it needs citing)

Replace the ratio with the **absolute excess**:

    score(ref, c) = d_BFS(ref, c) − d_Manhattan(ref, c)

i.e. the steps "wasted" relative to the lower bound. It removes the degeneracy
(on `museum`, `(1,0)` would score 4 and be discarded in favour of genuinely
distant cells), is a one-line change, and stays interpretable. Rejected only to
avoid invalidating the existing results.

## To remember for the report

State that the detour index selects **deceptive but not necessarily distant**
goals, causing them to cluster near the start and guaranteeing real difficulty
only in complex mazes; cite the absolute excess as a possible improvement not
adopted.
