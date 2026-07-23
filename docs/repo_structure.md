# Repository Structure

```
Rob-26-MazeSolver/
├── run.py
├── implementation.md
├── requirements.txt
├── README.md
├── LICENSE
├── CITATIONS.bib
│
├── src/
│   ├── __init__.py
│   ├── README.md
│   ├── constants.py
│   ├── maze_map.py
│   ├── robot.py
│   ├── goal_placement.py
│   │
│   ├── api/
│   │   ├── __init__.py
│   │   ├── README.md
│   │   ├── base_api.py
│   │   ├── mms_api.py
│   │   └── sim_api.py
│   ├── algorithms/
│   │   ├── __init__.py
│   │   ├── README.md
│   │   ├── base.py
│   │   ├── base_algorithm.py
│   │   ├── astar.py
│   │   └── dstar_lite.py
│   ├── parser/
│   │   ├── __init__.py
│   │   ├── README.md
│   │   └── maze_parser.py
│   └── metrics/
│       ├── __init__.py
│       ├── README.md
│       └── logger.py
│
├── mazes/
│   ├── README.md
│   ├── index.html
│   ├── maze_test.txt
│   ├── txt/              # 55 competition mazes, flat, ASCII .txt (Map format)
│   ├── img/              # PNG rendering of each maze in txt/
│   └── img2/              # secondary PNG renderings (alternate source)
│
├── results/
│   └── logs/               # gitignored; <goal-count>/<algo>/*.json
│
├── experiments/
│   ├── __init__.py
│   ├── 01_experiment.py
│   └── run_batch.py
│
├── tools/
│   ├── README.md
│   ├── gen_maze.py
│   ├── filter_connected.py
│   └── place_goals.py
│
├── scripts/
│   ├── README.md
│   └── clean.sh
│
├── tests/
│   ├── __init__.py
│   ├── test_maze_map.py
│   ├── test_robot.py
│   ├── test_maze_parser.py
│   ├── test_metrics_logger.py
│   ├── test_integration.py
│   ├── test_goal_placement.py
│   ├── test_auto_goals.py
│   ├── test_verbose_flag.py
│   └── test_no_stray_print.py
│
├── notebooks/
│   ├── goals_analysis.ipynb
│   ├── data_analysis.ipynb
│   └── report.md            # results write-up, not yet written
│
└── docs/
    ├── Rob_26_proposal.md / .pdf
    ├── Rob-26-MazeSolver_report.md / .pdf
    ├── implementation_roadmap.md
    ├── repo_structure.md
    ├── notes.md
    ├── mms.md
    ├── detour_metric_limitations.md
    ├── articles/             # gitignored reference PDFs
    └── res/                  # screenshots + generated analysis figures
```

`app/mms.app` (the MMS simulator binary itself) also lives at the repo root but is gitignored, along with `results/logs/` and `docs/articles/`.

---

## Directory and File Descriptions

### `run.py`
The MMS GUI entry point — the script path configured in MMS's "Run command" field. It always talks to the real simulator via `MmsAPI`, never `SimAPI`; headless/batch execution goes exclusively through `experiments/run_batch.py`. Parses `--algo {astar,dstar_lite}`, `--goal X Y` (repeatable), `--n-goals N --seed S`, `--auto-goals MAZE -k N` (in-process detour-index placement via `src/goal_placement.py`, resolved against `mazes/txt/` and dimension-checked against the loaded maze), `--heuristic {min_path,manhattan}` (A\* only, ignored by D\*-Lite), `--maze-name` (log-filename override), `--output-dir`, `--no-log`. `--goal`, `--n-goals`, and `--auto-goals` are mutually exclusive; with none given, the algorithm defaults to the maze's 4-cell centre area. Instantiates `MmsAPI`, `MazeMap`, `Robot`, the chosen algorithm and a `MetricsLogger`, calls `algorithm.run()`, then (unless `--no-log`) `logger.export_json()` — producing a log directly comparable to headless `SimAPI` runs. Also spawns a small Tkinter legend window in its own process (Tkinter needs to own its process's main thread) mapping the running algorithm's GUI colors/text codes to their meaning; a legend window left over from a previous MMS "Run" click is closed automatically.

### `implementation.md`
Detailed per-file specifications and design decisions (wall bitmask encoding, module layout, etc.) for the core infrastructure under `src/`, referenced from `docs/implementation_roadmap.md`.

---

### `/src`
All Python source code for the project. Shared primitives (`constants.py`, `maze_map.py`, `robot.py`, `goal_placement.py`) live directly under `src/`.

- **`constants.py`** — `Direction` `IntEnum` (N=0, E=1, S=2, W=3, clockwise rotation order), wall-bitmask constants (`WALL_N=1, WALL_E=2, WALL_S=4, WALL_W=8`, combinations 0–15), lookup tables (`DIR_TO_WALL`, `DIR_TO_DELTA`, `OPPOSITE_DIR`, `DIR_TO_STR`), `COLORS` and `COLOR_HEX` (hex values for the 15 supported MMS color codes, used to render the GUI legend).
- **`maze_map.py`** — `MazeMap(width, height)`: owns wall and visit matrices; `set_wall`/`clear_wall` (symmetric neighbour update), `mark_visit`, `is_explored`/`has_wall` queries, `export_walls`/`export_visits`, `distinct_cells_visited`, `total_visits`.
- **`robot.py`** — `Robot(x, y, heading)`: `turn_left()`/`turn_right()`/`move_forward()`, `wall_front/right/left/back_dir()`, `reset()`.
- **`goal_placement.py`** — `scenario_goals(wall_matrix, width, height, start, k)` / `place_goals()`: deterministic goal placement by **detour index** (`d_BFS(ref, c) / d_Manhattan(ref, c)`), the difficulty proxy used across the maze corpus (see `tools/README.md` for the full algorithm and `docs/detour_metric_limitations.md` for a known bias in the metric).

#### `/src/api/`
Houses the API layer that decouples algorithms from the I/O backend.

- **`base_api.py`** — Abstract base class (`BaseAPI`) declaring the full protocol as abstract methods: maze info (`maze_width`/`maze_height`), wall sensing (`wall_front/back/left/right`), movement (`move_forward`, `turn_right`/`turn_left`), display (`set_wall`/`clear_wall`, `set_color`/`clear_color`/`clear_all_color`, `set_text`/`clear_text`/`clear_all_text`), control (`was_reset`/`ack_reset`), and stats (`get_stat`). All algorithms are coded against this interface only.
- **`mms_api.py`** — Module-level functions (`mazeWidth`, `wallFront`, `moveForward`, `setWall`, `wasReset`, …) adapted from `mackorone/mms-python`'s `API.py`, writing commands to `sys.stdout`, reading responses from `sys.stdin`, and printing debug messages to `sys.stderr`; `MmsAPI(BaseAPI)` wraps them for the `BaseAPI` contract.
- **`sim_api.py`** — `SimAPI(BaseAPI)`: headless implementation backed by an in-memory `wall_matrix`. Answers wall-sensor queries via bitmask lookup, tracks robot position/heading internally, raises `MouseCrashedError` on an invalid `move_forward`; display methods are no-ops. Enables fully automated `pytest` and batch-run execution with no GUI.

#### `/src/algorithms/`
- **`base_algorithm.py`** — `BaseAlgorithm(ABC)`: constructor takes `api, maze_map, robot, logger`; goal resolution at construction time (`goals=[...]` → explicit list; `n_random_goals=k, random_seed=s` → `k` random free cells; neither → 4-cell centre area); `heuristic="min_path"|"manhattan"` (see below); `verbose` (default `True`) gates `[WALL]`/`[REPLAN]` stderr diagnostics without affecting the exported JSON log. Abstract `run()`. Shared utilities: `_sense_and_update`, `_execute_path`, `_compute_goal_heuristic` (wall-aware multi-source BFS from the goal set), `_compute_start_heuristic`, `_compute_heuristic` (dispatches on `self._heuristic`), `_report_walls`/`_report_replan` (consolidated stderr formatting). Polls `was_reset()`/`ack_reset()`/`robot.reset()` once per sense-plan-act iteration.
- **`astar.py`** — `AStarExplorer`: runs A\* from scratch on the current partial map under the freespace assumption (unexplored cells treated as passable), replans whenever a newly confirmed wall lies on the current plan, executes path steps via `turn_left`/`turn_right`/`move_forward`. Multi-goal via greedy nearest-goal visitation order. `--heuristic`: `min_path` (default, wall-aware BFS) or `manhattan` (straight-line, ignores wall knowledge). GUI: colors expanded/open-list cells, displays `f`/`h`-value text, clears on each replanning event.
- **`dstar_lite.py`** — `DStarLiteExplorer`: incremental D\*-Lite (`g`/`rhs` values, priority queue, `km` accumulator for agent-motion consistency); repairs the plan incrementally from newly discovered walls rather than replanning from scratch. Uses its own Manhattan-to-current-position heuristic internally (the `--heuristic` flag is a no-op for this algorithm — its `Key` invariant requires a heuristic consistent with a fixed reference point, a different mathematical role than A\*'s). GUI: `g-XXXr-YYY` cell text, consistent/inconsistent/trivial cell coloring.
- **`base.py`** — A standalone wall-follower script adapted from `mackorone/mms-python`'s `Main.py`, calling the raw `mms_api` module functions directly rather than going through `BaseAPI`/`BaseAlgorithm`. Independent of the shared algorithm hierarchy used by `AStarExplorer` and `DStarLiteExplorer`.

Both `AStarExplorer` and `DStarLiteExplorer` extend `BaseAlgorithm` and share the same constructor signature, `run()` entry point, and `MetricsLogger`/JSON-log schema, so they are interchangeable in `run.py` and `experiments/run_batch.py`.

#### `/src/parser/`
- **`maze_parser.py`** — `parse_maze(filepath) -> (wall_matrix, width, height)`: parses the ASCII `.txt` Map format (the only maze format used in the project). Cell `(x, y)` maps to fixed offsets into the character grid; returns `N=1, E=2, S=4, W=8` bitmasks.

#### `/src/metrics/`
- **`logger.py`** — `MetricsLogger(algorithm_name, maze_name)`: incremented via `start`/`log_move`/`log_turn`/`stop`. `set_matrices`, `set_scenario` (records the goal-placement `scenario` block — maze file, k, per-goal detour scores; `null` for non-automated goal runs), `set_goal_count`. Replanning tracking: `start_plan_timer`/`log_replanning_event(position, nodes_expanded, residual_distance, memory_occupancy)` plus derived properties `total_replanning_events`, `cumulative_planning_time`, `cumulative_nodes_expanded`. `export_json(output_dir="results/logs/")` writes to `<output_dir>/<goal-count>/<algo>/<algo>_<maze_name>_<YYYYMMDD_HHMMSS>.json` — bucketed first by goal count (`one_goal/`, `two_goals/`, `three_goals/`, `four_goals/`) and then by algorithm; colliding filenames within the same second get a `_1`, `_2`, … suffix instead of overwriting.

---

### `/mazes/`
All maze files live flat in [`txt/`](../mazes/txt/) — 55 `.txt` files in the ASCII Map format, sourced from `tcp4me.com/mmr/mazes/` — plus a standalone `maze_test.txt` at the `mazes/` root used by the test suite. There is no per-maze difficulty tiering: exploration difficulty is determined by **goal placement** (`src/goal_placement.py`), so every maze in the corpus is exercised at every goal-count scenario. `img/` and `img2/` hold PNG renderings of each maze (55 each, from two sources/renderers) for quick visual inspection; `index.html` is a standalone browsable maze viewer (linked from the top-level README).

### `/results/`
Generated run logs, gitignored (not committed) to avoid bloating the repository with data files.

- **`logs/`** — One JSON file per `(algorithm, maze, goal count)` run, filed under `<goal-count>/<algo>/`. Each file contains the full metrics set plus the serialised wall/visit matrices and (for automated placements) the `scenario` block. Generated figures are written to `docs/res/` instead (see `/notebooks/` below); there is no `results/plots/`.

### `/experiments/`
Headless (`SimAPI`) batch runners; not imported as library code.

- **`01_experiment.py`** — Runs both algorithms over a configurable maze list (default: every maze in `mazes/txt/`); `--goals X,Y ...` applies an explicit goal list to every maze, or `-k N` sweeps goal-count scenarios `k=1..N` per maze (`k=1` is always the default centre-area goal by this script's convention; `k≥2` uses `scenario_goals`). Prints a summary table and exports logs.
- **`run_batch.py`** — Full-corpus batch test: both algorithms × every maze in `mazes/txt/` × the four goal-count scenarios (1/2/3/4) = up to 440 runs, with `verbose=False` to avoid flooding stderr. Live `tqdm` progress bar plus an in-place running-average summary table; exits non-zero if any run fails or misses its goal.

### `/tools/`
Standalone scripts for creating and curating maze files and inspecting goal placement.

- **`gen_maze.py`** — Generates a perfect maze (no loops, no islands) in the MMS ASCII Map format.
- **`filter_connected.py`** — Scans a directory of mazes and deletes any that aren't fully connected or fail to parse; the connectivity guarantee is a hard dependency of the metrics subsystem (`residual_distance` in exported logs is only guaranteed finite because the corpus is fully connected).
- **`place_goals.py`** — Manual-inspection CLI over `src/goal_placement.py::scenario_goals`: prints the goals and detour scores it would place for a named maze. This is the same deterministic placement `run.py --auto-goals` and the batch experiments call automatically in-process — the tool exists only to *see* a placement (and print its detour scores); it writes nothing to disk and has no other consumer.

### `/scripts/`
- **`clean.sh`** — Removes Python cache artefacts (`__pycache__`, `.pytest_cache`, `*.pyc`/`*.pyo`, `.coverage`). The only script in this directory.

---

### `/tests/`
`pytest` suite; every test goes through `SimAPI` (no MMS process needed). There is no shared `conftest.py` — fixtures are defined locally within each test module.

- **`test_maze_map.py`** — Unit tests for wall setting/getting, visit counting, neighbour queries, and matrix export.
- **`test_robot.py`** — Unit tests for heading arithmetic, position updates, and wall-direction translation.
- **`test_maze_parser.py`** — Parses a known `.txt` maze; asserts dimensions, perimeter walls, and a known interior cell's bitmask.
- **`test_metrics_logger.py`** — Move/turn counting, per-event replanning-record correctness, `export_json` schema validation.
- **`test_integration.py`** — End-to-end: both `AStarExplorer` and `DStarLiteExplorer` reach all goals on handcrafted mazes via `SimAPI`; replanning-event counts are checked for consistency between the two algorithms on the same maze.
- **`test_goal_placement.py`** — `scenario_goals`/`place_goals` correctness: determinism, nesting across increasing k, tie-breaking rule.
- **`test_auto_goals.py`** — `run.py --auto-goals` maze-name resolution, the dimension-mismatch guard, and mutual exclusivity with `--goal`/`--n-goals`.
- **`test_verbose_flag.py`** — `verbose=False` suppresses `[WALL]`/`[REPLAN]` stderr output without affecting the exported JSON log.
- **`test_no_stray_print.py`** — Guards that no `print()` call exists under `src/algorithms/` or `src/api/mms_api.py` outside the intended stdin/stdout protocol I/O — MMS communicates over raw stdin/stdout, so a stray `print()` would desync the protocol; this class of bug is invisible to `SimAPI`-based tests otherwise.

---

### `/notebooks/`
- **`goals_analysis.ipynb`** — Renders the detour-score heatmap `place_goals` maximizes at each step, for k=1..4 on a chosen maze; saves `docs/res/goal_heatmap_evolution.svg`. Source data is computed directly from `mazes/txt/` via `src/goal_placement.py` — no run logs needed.
- **`data_analysis.ipynb`** — Reads every JSON log under `results/logs/`; produces the A\*-vs-D\*-Lite replanning-cost comparison (bar chart) and the nodes-expanded-vs-residual-distance regression figures, split by goal count and aggregated. Saves all figures to `docs/res/`.
- **`report.md`** — Not yet written. Will document the aggregated experimental results and any anomalies (e.g. D\*-Lite memory growth on high-complexity mazes) once the full analysis is complete.

### `/docs/`
Project documentation.

- **`Rob_26_proposal.md`** — Original project proposal (read-only reference).
- **`Rob-26-MazeSolver_report.md`** — The final report, structured per the proposal index; not yet written.
- **`implementation_roadmap.md`** — Phase-by-phase implementation roadmap.
- **`repo_structure.md`** — This file.
- **`notes.md`** — Course logistics and background reference material (assigned readings, videos, submission rules).
- **`mms.md`** — Complete MMS simulator setup and integration guide: installation, algorithm configuration, `run.py`'s full CLI (including `--auto-goals`/`--heuristic`/`--no-log`), GUI usage walkthrough, troubleshooting table.
- **`detour_metric_limitations.md`** — Documents a known bias of the detour-index goal-placement metric (it rewards small Manhattan denominators, so goals cluster near the start; the ratio measures *relative* deception, not *absolute* difficulty) — a deliberate, accepted limitation to state explicitly in the final report.
- **`articles/`** — Reference PDFs (gitignored, not committed).
- **`res/`** — Screenshots referenced by `mms.md` (`mms_gui.png`, `new_algorithm_dialog.png`, `maze_selection.png`, `wall_configurations_dict.png`, `logo_unibs.png/.svg`) and generated analysis figures referenced by the report (`goal_heatmap_evolution.svg`, `replanning_cost_bars.svg`, `nodes_vs_residual_distance_by_k.svg`, `nodes_vs_residual_distance_aggregate.svg`).

### `requirements.txt`
Python package dependencies: `numpy`, `matplotlib`, `pytest`, `pandas`, `seaborn`, `tqdm` — covering the core algorithms, log analysis (`notebooks/`), and batch-run progress reporting (`experiments/run_batch.py`).
