"""
Goal placement: place K goals in a maze by detour index.

For a reference cell ``ref`` and a free cell ``c`` the detour index is

    detour(ref, c) = d_BFS(ref, c) / d_Manhattan(ref, c)

where d_BFS is the true shortest-path distance in the maze (BFS,
4-connectivity). A cell with detour ~1 is "honest" about its distance;
a high-detour cell looks close but is far (e.g. just behind a wall).
Such deceptive cells defeat a greedy planner with a partial map, so the
detour index is the difficulty proxy for goal placement.

The detour index (a.k.a. route factor / circuity) is a standard metric in
spatial-network analysis, normally defined with Euclidean straight-line
distance; here it uses Manhattan distance, the correct straight-line lower
bound on a 4-connected grid. See tools/README.md ("References") for
citations (Barthélemy 2011; Gastner & Newman 2006).

Placement algorithm (deterministic, no randomness):
  1. Goal 1 = argmax over free cells of detour(start, c).
  2. Goal k (k >= 2) = argmax of min(detour(ref, c) for ref in {start, goal_1,
     ..., goal_{k-1}}) — every previously placed goal is used as a reference,
     not just the last one.
  3. Ties break to the lowest (row, col) = lowest (y, x), y=0 at bottom.
  4. Scenarios (k >= 2) are nested: level L = the first L goals.

Scenario rule (``scenario_goals``): k = 1 is the classic micromouse
scenario — the goal is fixed at the maze's centre cell, not chosen by
detour search — and k >= 2 uses the placement algorithm above. This means
the k = 1 goal is deliberately NOT the first element of the k >= 2 nested
sequence.
"""
from __future__ import annotations

import math
import sys
from collections import deque
from dataclasses import dataclass

from src.constants import DIR_TO_DELTA, DIR_TO_WALL, Direction

# Cell coordinates are (x, y) with y=0 at the bottom (MMS origin bottom-left),
# matching wall_matrix[y][x] everywhere in this repo.
Cell = tuple[int, int]


@dataclass
class PlacementStep:
    """Result of placing one goal.

    Attributes:
        goal: The chosen cell (x, y).
        score: Value of the score map at the chosen cell (the minimum
            detour across all references active at this step).
        detour_from_start: detour(start, goal).
        detour_from_goals: detour(goal_i, goal) for each previously placed
            goal_i, in placement order. Empty for goal 1.
    """
    goal: Cell
    score: float
    detour_from_start: float
    detour_from_goals: list[float]


def bfs_distance_map(
    wall_matrix: list[list[int]], width: int, height: int, source: Cell,
) -> list[list[float]]:
    """BFS shortest-path distances from *source* over the fully-known maze.

    Args:
        wall_matrix: wall_matrix[y][x] N/E/S/W bitmask (fully-known maze).
        width: Number of columns.
        height: Number of rows.
        source: Source cell (x, y).

    Returns:
        dist[y][x] = shortest path length in cells (4-connectivity,
        respecting walls); math.inf for unreachable cells.
    """
    dist: list[list[float]] = [[math.inf] * width for _ in range(height)]
    sx, sy = source
    dist[sy][sx] = 0.0
    queue: deque[Cell] = deque([source])
    while queue:
        x, y = queue.popleft()
        mask = wall_matrix[y][x]
        for direction in Direction:
            if mask & DIR_TO_WALL[direction]:
                continue
            dx, dy = DIR_TO_DELTA[direction]
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and math.isinf(dist[ny][nx]):
                dist[ny][nx] = dist[y][x] + 1
                queue.append((nx, ny))
    return dist


def manhattan(a: Cell, b: Cell) -> int:
    """Manhattan distance between two cells."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def detour_map(
    dist: list[list[float]], ref: Cell, width: int, height: int,
) -> list[list[float]]:
    """Detour index of every cell relative to *ref*.

    detour[y][x] = dist[y][x] / manhattan(ref, (x, y)).

    NaN at *ref* itself (Manhattan distance 0 — never a valid candidate)
    and at unreachable cells (dist == inf), so both are skipped by argmax.

    Args:
        dist: BFS distance map from *ref* (as returned by bfs_distance_map).
        ref: The reference cell the distances were computed from.
        width: Number of columns.
        height: Number of rows.

    Returns:
        detour[y][x] as described above.
    """
    detour: list[list[float]] = [[math.nan] * width for _ in range(height)]
    for y in range(height):
        for x in range(width):
            if (x, y) == ref or math.isinf(dist[y][x]):
                continue
            md = manhattan(ref, (x, y))
            assert md > 0, f"Manhattan distance is 0 for candidate {(x, y)} vs ref {ref}"
            detour[y][x] = dist[y][x] / md
    return detour


def argmax_cell(
    score_map: list[list[float]], width: int, height: int, excluded: set[Cell],
) -> tuple[Cell, float] | None:
    """Cell with the maximum finite score, skipping NaN and *excluded* cells.

    Deterministic tie-break: lowest (row, col) = lowest (y, x), where row 0
    is the bottom row (the first index of wall_matrix). Implemented by
    scanning in (y, x) order with a strict '>' comparison, so the
    first-visited cell wins ties.

    Returns:
        ((x, y), score) of the winner, or None if no valid candidate exists.
    """
    best: tuple[Cell, float] | None = None
    for y in range(height):
        for x in range(width):
            value = score_map[y][x]
            if math.isnan(value) or (x, y) in excluded:
                continue
            if best is None or value > best[1]:
                best = ((x, y), value)
    return best


def _combine_min(maps: list[list[list[float]]], width: int, height: int) -> list[list[float]]:
    """Elementwise minimum across several maps, NaN if any map is NaN there.

    Python's bare min() is asymmetric with NaN (e.g. min(5.0, nan) == 5.0),
    so NaN is propagated explicitly here rather than relying on min().
    """
    combined: list[list[float]] = [[math.nan] * width for _ in range(height)]
    for y in range(height):
        for x in range(width):
            values = [m[y][x] for m in maps]
            if any(math.isnan(v) for v in values):
                continue
            combined[y][x] = min(values)
    return combined


def place_goals(
    wall_matrix: list[list[int]], width: int, height: int, start: Cell, k: int,
) -> list[PlacementStep]:
    """Place up to *k* goals by maximizing the detour index.

    Goal 1 maximizes detour(start, c). Goal k >= 2 maximizes
    min(detour(ref, c) for ref in {start, goal_1, ..., goal_{k-1}}) —
    every previously placed goal is used as a reference, not just the
    last one. Candidates are free reachable cells excluding the start and
    already-placed goals. Stops early (with a warning on stderr) if
    candidates run out before k goals are placed.

    Args:
        wall_matrix: wall_matrix[y][x] N/E/S/W bitmask (fully-known maze).
        width: Number of columns.
        height: Number of rows.
        start: Start cell (x, y).
        k: Number of goals to place (>= 1).

    Returns:
        One PlacementStep per placed goal, in placement order.
    """
    dist_start = bfs_distance_map(wall_matrix, width, height, start)
    det_start = detour_map(dist_start, start, width, height)
    det_maps: list[list[list[float]]] = [det_start]  # index 0 = start, i = goal i

    steps: list[PlacementStep] = []
    excluded: set[Cell] = {start}

    for _ in range(k):
        score_map = det_start if len(det_maps) == 1 else _combine_min(det_maps, width, height)

        result = argmax_cell(score_map, width, height, excluded)
        if result is None:
            print(
                f"warning: only {len(steps)} of {k} goals placed "
                f"(no candidate cells left)",
                file=sys.stderr,
            )
            break

        goal, score = result
        gx, gy = goal
        steps.append(PlacementStep(
            goal=goal,
            score=score,
            detour_from_start=det_start[gy][gx],
            detour_from_goals=[m[gy][gx] for m in det_maps[1:]],
        ))
        excluded.add(goal)

        dist_goal = bfs_distance_map(wall_matrix, width, height, goal)
        det_maps.append(detour_map(dist_goal, goal, width, height))

    return steps


def scenario_goals(
    wall_matrix: list[list[int]], width: int, height: int, start: Cell, k: int,
) -> list[tuple[Cell, float]]:
    """Goals for the k-goal scenario of a maze, as (cell, detour) pairs.

    k == 1: classic micromouse scenario — the fixed centre cell
        ``(width // 2 - 1, height // 2 - 1)``, with its real detour value
        (BFS distance from start / Manhattan distance). Deliberately NOT
        the same as the first element place_goals() would choose for
        k >= 2 — see module docstring.
    k >= 2: detour-index placement via place_goals(); detour is the
        placement score (the min-detour across all active references).

    Args:
        wall_matrix: wall_matrix[y][x] N/E/S/W bitmask (fully-known maze).
        width: Number of columns.
        height: Number of rows.
        start: Start cell (x, y).
        k: Number of goals in the scenario (>= 1).

    Returns:
        List of (cell, detour) pairs, one per goal, in placement order.
        May be shorter than k if place_goals() runs out of candidates.

    Raises:
        ValueError: k < 1; or (k == 1) the centre cell is out of bounds,
            coincides with start, or is unreachable from start.
    """
    if k < 1:
        raise ValueError(f"k must be >= 1, got {k}")

    if k == 1:
        centre: Cell = (width // 2 - 1, height // 2 - 1)
        if not (0 <= centre[0] < width and 0 <= centre[1] < height):
            raise ValueError(
                f"fixed goal {centre} out of bounds for {width}x{height} maze"
            )
        if centre == start:
            raise ValueError(f"fixed goal {centre} coincides with start {start}")
        dist_start = bfs_distance_map(wall_matrix, width, height, start)
        if math.isinf(dist_start[centre[1]][centre[0]]):
            raise ValueError(f"fixed goal {centre} is unreachable from start {start}")
        detour = dist_start[centre[1]][centre[0]] / manhattan(start, centre)
        return [(centre, detour)]

    steps = place_goals(wall_matrix, width, height, start, k)
    return [(s.goal, s.score) for s in steps]
