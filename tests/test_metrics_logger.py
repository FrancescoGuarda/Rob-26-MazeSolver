"""Unit tests for the extended MetricsLogger (Phase 3 additions)."""
from __future__ import annotations

import json
import os
import tempfile

import pytest

from src.metrics.logger import MetricsLogger


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_logger(algo: str = "test_algo", maze: str = "test_maze") -> MetricsLogger:
    return MetricsLogger(algo, maze)


def simple_visit_matrix(n: int = 4) -> list[list[int]]:
    m = [[0] * n for _ in range(n)]
    m[0][0] = 2
    m[1][1] = 1
    return m


# ---------------------------------------------------------------------------
# Phase 2 metrics (must remain unchanged)
# ---------------------------------------------------------------------------

def test_phase2_metrics_unaffected():
    logger = make_logger()
    logger.start()
    logger.log_move()
    logger.log_move()
    logger.log_turn()
    logger.stop()
    logger.set_matrices([[0]], [[1]])

    assert logger.forward_moves == 2
    assert logger.turns == 1
    assert logger.total_moves == 3
    assert logger.execution_time is not None
    assert logger.execution_time >= 0.0


# ---------------------------------------------------------------------------
# start_plan_timer / log_replanning_event
# ---------------------------------------------------------------------------

def test_no_events_initially():
    logger = make_logger()
    assert logger.total_replanning_events == 0
    assert logger.replanning_events == []
    assert logger.cumulative_planning_time == 0.0
    assert logger.cumulative_nodes_expanded == 0


def test_log_single_event():
    logger = make_logger()
    logger.start_plan_timer()
    logger.log_replanning_event((1, 2), nodes_expanded=10, residual_distance=5, memory_occupancy=15)

    assert logger.total_replanning_events == 1
    events = logger.replanning_events
    e = events[0]
    assert e["event_id"] == 0
    assert e["position"] == [1, 2]
    assert e["nodes_expanded"] == 10
    assert e["residual_distance"] == 5
    assert e["memory_occupancy"] == 15
    assert isinstance(e["planning_time_s"], float)
    assert e["planning_time_s"] >= 0.0
    assert e["cost_ratio"] == pytest.approx(10 / 5)


def test_log_multiple_events():
    logger = make_logger()
    for i in range(3):
        logger.start_plan_timer()
        logger.log_replanning_event((i, 0), nodes_expanded=i + 1, residual_distance=2, memory_occupancy=i + 5)

    assert logger.total_replanning_events == 3
    assert logger.cumulative_nodes_expanded == 1 + 2 + 3
    assert logger.cumulative_planning_time >= 0.0
    events = logger.replanning_events
    for idx, e in enumerate(events):
        assert e["event_id"] == idx


def test_cost_ratio_zero_residual():
    """cost_ratio should be None when residual_distance is 0 (at-goal replanning)."""
    logger = make_logger()
    logger.start_plan_timer()
    logger.log_replanning_event((0, 0), nodes_expanded=5, residual_distance=0, memory_occupancy=10)
    assert logger.replanning_events[0]["cost_ratio"] is None


def test_plan_timer_reset_after_event():
    """A second log_replanning_event without start_plan_timer still produces a valid record."""
    logger = make_logger()
    logger.start_plan_timer()
    logger.log_replanning_event((0, 0), 1, 1, 1)
    # No start_plan_timer call before second event — should not raise
    logger.log_replanning_event((1, 1), 2, 2, 2)
    assert logger.total_replanning_events == 2


# ---------------------------------------------------------------------------
# export_json schema
# ---------------------------------------------------------------------------

def test_export_json_contains_replanning_keys():
    logger = make_logger()
    logger.start()
    logger.log_move()
    logger.stop()
    logger.set_matrices([[4]], [[1]])
    logger.start_plan_timer()
    logger.log_replanning_event((0, 0), 3, 2, 5)

    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        with open(path) as fh:
            data = json.load(fh)

    # Phase 2 keys present
    for key in ("algorithm", "maze", "total_moves", "forward_moves", "turns",
                "distinct_cells_visited", "total_visits", "execution_time_s",
                "wall_matrix", "visit_matrix"):
        assert key in data, f"Missing Phase 2 key: {key}"

    # Phase 3 extension keys present
    for key in ("total_replanning_events", "cumulative_planning_time_s",
                "cumulative_nodes_expanded", "replanning_events"):
        assert key in data, f"Missing Phase 3 key: {key}"

    assert data["total_replanning_events"] == 1
    assert isinstance(data["replanning_events"], list)
    assert len(data["replanning_events"]) == 1

    event = data["replanning_events"][0]
    for field in ("event_id", "position", "planning_time_s", "nodes_expanded",
                  "residual_distance", "cost_ratio", "memory_occupancy"):
        assert field in event, f"Missing event field: {field}"


def test_export_json_zero_events():
    logger = make_logger()
    logger.start(); logger.stop()
    logger.set_matrices([[0]], [[0]])
    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        with open(path) as fh:
            data = json.load(fh)
    assert data["total_replanning_events"] == 0
    assert data["replanning_events"] == []
    assert data["cumulative_planning_time_s"] == 0.0
    assert data["cumulative_nodes_expanded"] == 0


# ---------------------------------------------------------------------------
# replanning_events returns a copy
# ---------------------------------------------------------------------------

def test_replanning_events_is_copy():
    logger = make_logger()
    logger.start_plan_timer()
    logger.log_replanning_event((0, 0), 1, 1, 1)
    events = logger.replanning_events
    events.clear()
    assert logger.total_replanning_events == 1


# ---------------------------------------------------------------------------
# Goal-count export bucketing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n_goals,expected_dir", [
    (1, "one_goal"),
    (2, "two_goals"),
    (3, "three_goals"),
    (4, "four_goals"),
    (10, "ten_goals"),
    (12, "12_goals"),
])
def test_export_json_goal_count_directory(n_goals, expected_dir):
    logger = make_logger(algo="astar")
    logger.set_goal_count(n_goals)
    logger.start(); logger.stop()
    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        rel = os.path.relpath(path, tmp)
        with open(path) as fh:
            data = json.load(fh)
    assert os.path.dirname(rel) == os.path.join(expected_dir, "astar")
    assert data["goal_count"] == n_goals


def test_export_json_without_goal_count_uses_unknown_bucket():
    logger = make_logger(algo="astar")
    logger.start(); logger.stop()
    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        rel = os.path.relpath(path, tmp)
        with open(path) as fh:
            data = json.load(fh)
    assert os.path.dirname(rel) == os.path.join("unknown_goals", "astar")
    assert data["goal_count"] is None


def test_set_scenario_sets_goal_count():
    logger = make_logger()
    logger.set_scenario("mazes/txt/88us.txt", 3, [((7, 7), 0.5)])
    logger.start(); logger.stop()
    with tempfile.TemporaryDirectory() as tmp:
        path = logger.export_json(output_dir=tmp)
        assert os.path.basename(os.path.dirname(os.path.dirname(path))) == "three_goals"


def test_export_json_does_not_overwrite_same_second_run():
    """Two runs of the same algo/maze/goal count must produce two files."""
    paths = []
    with tempfile.TemporaryDirectory() as tmp:
        for _ in range(3):
            logger = make_logger()
            logger.set_goal_count(1)
            logger.start(); logger.stop()
            paths.append(logger.export_json(output_dir=tmp))
        assert len(set(paths)) == 3
        assert all(os.path.exists(p) for p in paths)
