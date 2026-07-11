# `src/metrics` — Metrics Collection and Export

Collects per-run algorithm performance data and writes timestamped JSON logs to `results/logs/`.

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

Writes a JSON file to `output_dir` (created if absent) and returns the full file path.

**Filename convention:** `{algorithm}_{maze}_{YYYYMMDD_HHMMSS}.json`

**JSON payload keys:**

| Key | Type | Description |
|-----|------|-------------|
| `algorithm` | `str` | Algorithm name |
| `maze` | `str` | Maze name |
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

logger = MetricsLogger("flood_fill", "maze_test")
logger.start()

# inside the algorithm loop:
api.turn_right();  logger.log_turn()
api.move_forward(); logger.log_move()

logger.stop()
logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
path = logger.export_json()   # → "results/logs/flood_fill_maze_test_20260704_153000.json"
```
