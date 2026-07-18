# `src/metrics` — Metrics Collection and Export

Collects per-run algorithm performance data and writes timestamped JSON logs to `results/logs/<goal-count>/<algorithm>/`.

## Module

| Module | Status | Description |
|--------|--------|-------------|
| [`logger.py`](#loggerpy) | ✓ implemented | `MetricsLogger` — records and exports run statistics |

---

## `logger.py`

### `MetricsLogger`

```python
MetricsLogger(algorithm_name: str, maze_name: str)
```

Accumulates metrics during a single algorithm run and exports them to JSON on completion.

### Recording interface

Call these methods from inside the algorithm's run loop:

| Method | Description |
|--------|-------------|
| `start()` | Record run start time (`time.monotonic()`) |
| `log_move()` | Increment forward-move counter by 1 |
| `log_turn()` | Increment turn counter by 1 |
| `stop()` | Record run end time |
| `set_matrices(walls, visits)` | Snapshot `MazeMap.export_walls()` / `export_visits()` at run end |
| `set_goal_count(n)` | Record how many goals the run targets; selects the export bucket |

### Computed metrics (read-only properties)

| Property | Type | Description |
|----------|------|-------------|
| `forward_moves` | `int` | Total forward moves |
| `turns` | `int` | Total turns (each 90° rotation = 1) |
| `total_moves` | `int` | `forward_moves + turns` |
| `distinct_cells_visited` | `int` | Cells with visit count ≥ 1 |
| `total_visits` | `int` | Sum of all visit counts (includes revisits) |
| `execution_time` | `float \| None` | Elapsed seconds; `None` if `stop()` not yet called |

### `export_json(output_dir)`

```python
export_json(output_dir: str = "results/logs/") -> str
```

Writes a JSON file to `output_dir/{goal-count}/{algorithm}/` (created if absent) and returns the full file path.

Logs are bucketed on two levels. The outer level is the number of goals the run targeted — `one_goal/`, `two_goals/`, `three_goals/`, `four_goals/`, and so on up to `ten_goals/`, with counts above ten falling back to a numeric name (`12_goals/`). The inner level is the algorithm, so with the default `output_dir` a four-goal A* run lands in `results/logs/four_goals/astar/` and its D* Lite counterpart in `results/logs/four_goals/dstar_lite/`.

Goal count comes first because it is the experimental variable: the comparison that matters is A* against D* Lite at the same difficulty, and putting the two algorithms side by side inside one bucket makes that pairing a directory listing rather than a filename-parsing exercise. The algorithm subdirectory name comes from the `algorithm_name` passed to the constructor, with any character that is not alphanumeric, `-` or `_` replaced by `_`, and lowercased.

The count itself is whatever was passed to `set_goal_count()`, or the `k` given to `set_scenario()` if only that was called. `set_goal_count()` should be fed from `BaseAlgorithm.goal_count`, which reports the requested goal count rather than the resolved cell count — the default micromouse centre area resolves to four adjacent cells but only one of them has to be reached, so it counts as `one_goal`. If neither method is called the run lands in `unknown_goals/`, which is a signal that a caller forgot to wire the count through rather than a legitimate bucket.

Consequence for analysis code: the tree is now two levels deep, so use `glob("results/logs/*/*/*.json")` or, more robustly, `Path("results/logs").rglob("*.json")`. The count is also duplicated into the `goal_count` payload key, so a loader never has to parse directory names to recover it.

**Filename convention:** `{goal-count}/{algorithm}/{algorithm}_{maze}_{YYYYMMDD_HHMMSS}.json`

Timestamps have one-second granularity, which a batch loop can easily collide with (the same algorithm, maze and goal count exported twice within the same second). When the target filename already exists, `_1`, `_2`, … is appended rather than overwriting, so no run is silently lost.

**JSON payload keys:**

| Key | Type | Description |
|-----|------|-------------|
| `algorithm` | `str` | Algorithm name |
| `maze` | `str` | Maze name |
| `goal_count` | `int \| null` | Goals targeted; `null` if never set (see `unknown_goals/`) |
| `timestamp` | `str` | `YYYYMMDD_HHMMSS` |
| `total_moves` | `int` | Forward moves + turns |
| `forward_moves` | `int` | Forward moves only |
| `turns` | `int` | Turn count |
| `distinct_cells_visited` | `int` | Unique cells reached |
| `total_visits` | `int` | Total cell visits |
| `execution_time_s` | `float \| null` | Run duration in seconds |
| `wall_matrix` | `list[list[int]]` | Wall bitmask matrix at run end |
| `visit_matrix` | `list[list[int]]` | Visit-count matrix at run end |

### Typical usage

```python
from src.metrics import MetricsLogger
from src.maze_map import MazeMap

logger = MetricsLogger("astar", "maze_test")
logger.set_goal_count(algorithm.goal_count)   # bucket: results/logs/one_goal/...
logger.start()

# inside the algorithm loop:
api.turn_right();  logger.log_turn()
api.move_forward(); logger.log_move()

logger.stop()
logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
path = logger.export_json()   # → "results/logs/one_goal/astar/astar_maze_test_20260704_153000.json"
```
