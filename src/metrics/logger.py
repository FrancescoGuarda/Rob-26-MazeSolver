"""
MetricsLogger: per-run performance metrics collector and exporter.

Accumulates forward moves, turns, visit data, and elapsed time during a run,
then exports results to a timestamped JSON file in
results/logs/<goal-count>/<algorithm>/.
"""
from __future__ import annotations

import json
import os
import time
from datetime import datetime


class MetricsLogger:
    """Collects and exports per-run algorithm performance metrics.

    Tracks efficiency (total moves = forward moves + turns), efficacy
    (distinct cells visited, total visits), and execution time.
    Stores wall and visit matrices for offline analysis.

    Usage::

        logger = MetricsLogger("astar", "maze_test")
        logger.start()
        # ... algorithm runs, calling logger.log_move() / logger.log_turn() ...
        logger.stop()
        logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
        path = logger.export_json()
    """

    def __init__(self, algorithm_name: str, maze_name: str) -> None:
        self._algo = algorithm_name
        self._maze = maze_name
        self._forward_moves: int = 0
        self._turns: int = 0
        self._start_time: float | None = None
        self._end_time: float | None = None
        self._wall_matrix: list[list[int]] | None = None
        self._visit_matrix: list[list[int]] | None = None
        # Replanning-event tracking (Phase 3 extension)
        self._replanning_events: list[dict] = []
        self._plan_timer_start: float | None = None
        self._scenario: dict | None = None
        self._goal_count: int | None = None

    # ------------------------------------------------------------------
    # Recording interface
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Record the run start time."""
        self._start_time = time.monotonic()

    def log_move(self) -> None:
        """Increment the forward-move counter by one."""
        self._forward_moves += 1

    def log_turn(self) -> None:
        """Increment the turn counter by one."""
        self._turns += 1

    def stop(self) -> None:
        """Record the run end time."""
        self._end_time = time.monotonic()

    def set_matrices(
        self,
        wall_matrix: list[list[int]],
        visit_matrix: list[list[int]],
    ) -> None:
        """Snapshot wall and visit matrices (use MazeMap.export_walls/visits)."""
        self._wall_matrix = wall_matrix
        self._visit_matrix = visit_matrix

    def set_scenario(
        self,
        maze_file: str,
        k: int,
        goals: list[tuple[tuple[int, int], float]],
    ) -> None:
        """Record scenario metadata: maze path, goal count, (cell, detour) pairs.

        Opaque to the logger — it stores whatever it is given without
        knowing what a detour value means. Call before export_json(); if
        never called, the exported "scenario" field is null.
        """
        self._goal_count = k
        self._scenario = {
            "maze_file": maze_file,
            "k": k,
            "goals": [
                {"cell": list(cell), "detour": detour} for cell, detour in goals
            ],
        }

    def set_goal_count(self, n_goals: int) -> None:
        """Record how many goals the run targets (drives the export subdirectory).

        Call before export_json(); if never called, logs land in an
        "unknown_goals" bucket.
        """
        self._goal_count = n_goals

    def start_plan_timer(self) -> None:
        """Record the start time of a planning / replanning call."""
        self._plan_timer_start = time.monotonic()

    def log_replanning_event(
        self,
        position: tuple[int, int],
        nodes_expanded: int,
        residual_distance: int | float,
        memory_occupancy: int,
    ) -> None:
        """Append one replanning-event record.

        Must be called after start_plan_timer() so that planning_time_s can be
        computed.  cost_ratio is set to None when residual_distance == 0.
        """
        planning_time = (
            time.monotonic() - self._plan_timer_start
            if self._plan_timer_start is not None
            else 0.0
        )
        cost_ratio: float | None = (
            nodes_expanded / residual_distance
            if residual_distance and residual_distance > 0
            else None
        )
        self._replanning_events.append({
            "event_id":          len(self._replanning_events),
            "position":          list(position),
            "planning_time_s":   planning_time,
            "nodes_expanded":    nodes_expanded,
            "residual_distance": int(residual_distance) if residual_distance != float('inf') else -1,
            "cost_ratio":        cost_ratio,
            "memory_occupancy":  memory_occupancy,
        })
        self._plan_timer_start = None  # reset timer

    # ------------------------------------------------------------------
    # Computed metrics (read-only properties)
    # ------------------------------------------------------------------

    @property
    def forward_moves(self) -> int:
        """Total number of forward moves."""
        return self._forward_moves

    @property
    def turns(self) -> int:
        """Total number of turns."""
        return self._turns

    @property
    def total_moves(self) -> int:
        """Total moves: forward moves + turns."""
        return self._forward_moves + self._turns

    @property
    def distinct_cells_visited(self) -> int:
        """Number of distinct cells visited at least once."""
        if self._visit_matrix is None:
            return 0
        return sum(1 for row in self._visit_matrix for v in row if v > 0)

    @property
    def total_visits(self) -> int:
        """Total number of cell visits (including revisits)."""
        if self._visit_matrix is None:
            return 0
        return sum(v for row in self._visit_matrix for v in row)

    @property
    def execution_time(self) -> float | None:
        """Elapsed time in seconds, or None if the run has not been stopped."""
        if self._start_time is None or self._end_time is None:
            return None
        return self._end_time - self._start_time

    # ------------------------------------------------------------------
    # Replanning-event metrics (Phase 3 extension)
    # ------------------------------------------------------------------

    @property
    def replanning_events(self) -> list[dict]:
        """Full list of per-replanning-event records."""
        return list(self._replanning_events)

    @property
    def total_replanning_events(self) -> int:
        """Number of replanning events recorded."""
        return len(self._replanning_events)

    @property
    def cumulative_planning_time(self) -> float:
        """Sum of planning_time_s over all replanning events (seconds)."""
        return sum(e["planning_time_s"] for e in self._replanning_events)

    @property
    def cumulative_nodes_expanded(self) -> int:
        """Sum of nodes_expanded over all replanning events."""
        return sum(e["nodes_expanded"] for e in self._replanning_events)

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    # Spelled-out names for the common goal counts; larger runs fall back to
    # a numeric bucket ("12_goals"), which still sorts and globs cleanly.
    _GOAL_WORDS = {
        1: "one", 2: "two", 3: "three", 4: "four", 5: "five",
        6: "six", 7: "seven", 8: "eight", 9: "nine", 10: "ten",
    }

    def _goal_dirname(self) -> str:
        """Name of the goal-count bucket directory, e.g. "one_goal", "four_goals"."""
        n = self._goal_count
        if n is None or n < 1:
            return "unknown_goals"
        word = self._GOAL_WORDS.get(n, str(n))
        return f"{word}_goal" if n == 1 else f"{word}_goals"

    def export_json(self, output_dir: str = "results/logs/") -> str:
        """Write metrics to a timestamped JSON file, bucketed by goal count and algorithm.

        The file is written to ``<output_dir>/<goal-count>/<algorithm_name>/``
        (e.g. ``results/logs/one_goal/astar/``), created if it does not exist.

        Returns:
            The full path of the written file.
        """
        # Two levels: goal count first, then algorithm, so a scenario's astar
        # and dstar_lite runs sit side by side under the same difficulty bucket.
        safe_algo = "".join(
            c if c.isalnum() or c in "-_" else "_" for c in self._algo
        ).strip("_").lower() or "unknown"
        run_dir = os.path.join(output_dir, self._goal_dirname(), safe_algo)
        os.makedirs(run_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Second-granularity timestamps collide when a batch runs the same
        # algorithm/maze/goal-count twice within a second; suffix instead of
        # overwriting so no run is silently lost.
        filepath = os.path.join(run_dir, f"{safe_algo}_{self._maze}_{timestamp}.json")
        dup = 1
        while os.path.exists(filepath):
            filepath = os.path.join(
                run_dir, f"{safe_algo}_{self._maze}_{timestamp}_{dup}.json"
            )
            dup += 1

        payload = {
            "algorithm": self._algo,
            "maze": self._maze,
            "goal_count": self._goal_count,
            "timestamp": timestamp,
            "total_moves": self.total_moves,
            "forward_moves": self._forward_moves,
            "turns": self._turns,
            "distinct_cells_visited": self.distinct_cells_visited,
            "total_visits": self.total_visits,
            "execution_time_s": self.execution_time,
            "wall_matrix": self._wall_matrix,
            "visit_matrix": self._visit_matrix,
            # Replanning-event fields (Phase 3 extension)
            "total_replanning_events": self.total_replanning_events,
            "cumulative_planning_time_s": self.cumulative_planning_time,
            "cumulative_nodes_expanded": self.cumulative_nodes_expanded,
            "replanning_events": self._replanning_events,
            "scenario": self._scenario,
        }

        with open(filepath, 'w') as fh:
            json.dump(payload, fh, indent=2)

        return filepath
