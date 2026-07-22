# Tools

This directory contains standalone Python scripts for creating and curating maze files.

---

### Available Scripts

| Script | Description |
|--------|-------------|
| `tools/gen_maze.py` | Generate a perfect maze (no loops, no islands) in the mms ASCII text format used by this project. |
| `tools/filter_connected.py` | Scan a directory of mazes and delete any that aren't fully connected or fail to parse. |
| `tools/place_goals.py` | Manual inspection tool: print goal placement for a maze by maximizing the BFS/Manhattan detour index. The actual placement logic lives in `src/goal_placement.py` and is called automatically by headless runs — this script has no other consumer. |

---

### Usage

```bash
python3 tools/gen_maze.py --rows 16 --cols 16 --seed 1 -o out.txt
python3 tools/filter_connected.py --dir mazes/txt --dry-run
python3 tools/place_goals.py 2015japan --start 0 0 -k 4
```

See each script's module docstring for full options.

---

## Goal placement (`src/goal_placement.py`, inspected via `tools/place_goals.py`)

Placement is **automated**: headless runs (e.g. `experiments/01_experiment.py -k N`,
`experiments/run_batch.py`) call `src.goal_placement.scenario_goals()` directly
at load time, right after parsing the maze, for every k ≥ 2. Placement is
deterministic and takes milliseconds, so there is no pre-step to run and no
file to keep in sync — the same maze and k always produce the same goals.

`scenario_goals()` itself treats every k ≥ 1 identically (see "Placement
algorithm" below — there is no special case for k = 1 inside the module).
By convention, both `01_experiment.py` and `run_batch.py` still choose to
never call it for a "1 goal" run: they treat that case as the classic
default centre-*area* goal (`goals=None`, resolved by `BaseAlgorithm`
itself), not as an automated single-cell placement. That is a decision
made by those two callers, not a rule of this module.

`tools/place_goals.py` is a thin CLI over that module, kept around only so
you can inspect where goals land in a maze and why (it prints the detour
scores; it writes nothing to disk).

### The metric: detour index

For a reference cell `ref` and a free cell `c`:

```
detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)
```

`d_BFS` is the true shortest-path distance in the maze (BFS,
4-connectivity, respecting walls). This is the standard **detour index**
(also called *route factor* or, in transportation geography, *circuity*)
from spatial-network analysis: the ratio of network distance to
straight-line distance, ≥ 1, with 1 meaning "perfectly direct". The only
adaptation here is that the straight-line term is the **Manhattan** (L1)
distance rather than the usual Euclidean (L2) one — L1 is the correct
straight-line lower bound on a 4-connected grid, and it is exactly the
admissible heuristic a greedy / A\* planner would use, which is what makes
a high ratio "deceptive" to such a planner.

See **References** at the end of this section.

Intuition: the detour index measures how much a cell **lies about its
distance**. A cell with detour ≈ 1 is honest — it is about as far as it
looks. A high-detour cell looks close (small Manhattan distance) but is
actually far (long real path), e.g. a cell just behind a wall. A greedy
planner with a partial map is drawn toward exactly these deceptive goals,
so the detour index is our difficulty proxy.

### Placement algorithm

Input: maze, start cell (default `(0, 0)`, the simulator convention), K
goals (default 4).

1. BFS from start → `detour(start, c)` for every free cell.
2. **Goal 1** = argmax of `detour(start, c)`. This is also the entire
   result for `k = 1` — there is no special case; `k = 1` is simply the
   first step of the same algorithm, so it is identical to the first goal
   a `k ≥ 2` placement would choose.
3. **Goal k (k ≥ 2)**: BFS from start and from **every** previously placed
   goal. For each candidate cell `c` (free, reachable, not the start, not
   an already-placed goal):
   `score(c) = min(detour(ref, c) for ref in {start, goal_1, ..., goal_{k-1}})`.
   Goal k = argmax of `score(c)`.
4. Scenarios are **nested** for every `k ≥ 1`: level L = the first L
   goals — a `k = 1` scenario's goal is exactly the goal a `k ≥ 2`
   scenario would choose first.

No randomness anywhere. Ties break deterministically to the lowest
`(row, col)` = lowest `(y, x)`, where row 0 is the **bottom** row (MMS
origin bottom-left, the first index of `wall_matrix`). Unreachable cells
are excluded; the Manhattan distance is never 0 for a valid candidate
(only the reference cell itself has Manhattan 0, and it is excluded).

### I/O

**Input**: an ASCII `.txt` maze in the MMS map format (the same files the
simulator uses, parsed with `src/parser/maze_parser.py`).

**Output**: printed to stdout only — one line per goal with its cell and
detour value (plus the min-detour breakdown for k ≥ 2). There is no file
output; nothing in the repo reads this tool's output. All cells are
`[x, y]` with y=0 at the bottom.

For headless runs, the same information is instead recorded per-run in the
`scenario` block of the exported metrics log (see
`src/metrics/logger.py::MetricsLogger.set_scenario`):

```json
"scenario": {
  "maze_file": "mazes/maze_test.txt",
  "k": 3,
  "goals": [
    {"cell": [1, 3], "detour": 7.0},
    {"cell": [7, 6], "detour": 5.0},
    {"cell": [1, 0], "detour": 3.0}
  ]
}
```

`"scenario"` is `null` in logs from runs that didn't request an automated
scenario (e.g. explicit `--goals` or the default centre-area goal).

### Running the tool

```bash
# Inspect the 4-goal (default) detour placement with the simulator's start (0, 0):
python3 tools/place_goals.py 2015japan

# Custom start and goal count (maze name resolves against mazes/txt/; .txt optional):
python3 tools/place_goals.py maze_test.txt --start 0 0 -k 2

# Automated in a headless run — no pre-step needed:
python3 experiments/01_experiment.py -k 3
```

Each run also prints a ready-to-paste `--goal X Y` list for the GUI's
"Run command" field (see `run.py`), e.g. `--goal 3 3 --goal 0 3`.

Pasting that list is optional: `run.py --auto-goals MAZE [-k N]` performs the
same placement in-process, so the GUI can be pointed at a maze by name instead
of at hand-copied coordinates. Because both paths call `scenario_goals()` and
placement is deterministic, the goals are identical either way — the tool
stays the place to *see* a placement (it alone prints the detour scores),
while `--auto-goals` is the way to *run* one. See `docs/mms.md` for the flag's
maze-name resolution and its dimension check.

### Known limitations

These are deliberate design choices, kept simple on purpose:

- **(a)** There is **no constraint keeping deceptive goals at a low
  apparent distance**: a goal can maximize the detour ratio while being
  both far by Manhattan and much farther by path, rather than being a
  nearby-looking trap.
- **(b)** **The ratio amplifies small Manhattan distances**, so in
  practice the tool tends to favor nearby-looking traps over cells that
  are merely far and mildly deceptive: a cell 1 step away (Manhattan) with
  a 7-step real path scores 7.0, while a cell 20 steps away with a
  40-step real path scores only 2.0, even though both add 6 and 20 wasted
  steps respectively. If you need goals that are also far in absolute
  terms, the score would need an added absolute-distance term — the tool
  does not do this.

### References

The detour index / route factor / circuity is a standard quantity in
spatial-network analysis (defined there with Euclidean straight-line
distance; we substitute Manhattan distance for the 4-connected grid):

- M. Barthélemy, "Spatial networks," *Physics Reports* **499**(1–3),
  1–101 (2011). — Review that defines the *detour index* / *route factor*
  `Q(i, j) = d_route(i, j) / d_Euclidean(i, j) ≥ 1`. See §"Mixing space
  and topology".
- M. T. Gastner and M. E. J. Newman, "The spatial structure of networks,"
  *Eur. Phys. J. B* **49**(2), 247–252 (2006). — Uses the same *route
  factor* ratio to characterise how directly networks connect points.

In transportation geography the identical ratio is usually called
**circuity** (e.g. work by D. Levinson and collaborators on street-network
circuity). We are not aware of prior work using this ratio specifically to
*place* adversarial goals for a maze planner — that application is ours;
only the underlying metric is from the literature.
