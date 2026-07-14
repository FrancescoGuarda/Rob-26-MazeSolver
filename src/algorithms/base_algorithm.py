"""
BaseAlgorithm: abstract base class for all maze-solving algorithms.

Provides shared infrastructure:
  - Goal resolution (centre, explicit list, random generation)
  - Wall sensing and MazeMap update after each move
  - Robot movement execution (turn + move_forward)
  - BFS-based heuristic computation (goal-side and start-side)
"""
from __future__ import annotations

import random
from abc import ABC, abstractmethod
from collections import deque

from src.api.base_api import BaseAPI
from src.constants import DIR_TO_DELTA, DIR_TO_STR, Direction
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot

# Maps (dx, dy) movement delta → Direction (inverse of DIR_TO_DELTA)
_DELTA_TO_DIR: dict[tuple[int, int], Direction] = {
    v: k for k, v in DIR_TO_DELTA.items()
}


class BaseAlgorithm(ABC):
    """Abstract base for AStarExplorer and DStarLiteExplorer.

    Constructor resolves goal coordinates once, at construction time.
    Subclasses implement ``run()`` with the full planning/execution loop.
    """

    def __init__(
        self,
        api: BaseAPI,
        maze_map: MazeMap,
        robot: Robot,
        logger: MetricsLogger,
        goals: list[tuple[int, int]] | None = None,
        n_random_goals: int | None = None,
        random_seed: int | None = None,
    ) -> None:
        self._api = api
        self._maze_map = maze_map
        self._robot = robot
        self._logger = logger
        self._start_pos: tuple[int, int] = robot.position
        self._width: int = api.maze_width()
        self._height: int = api.maze_height()
        self._goals: list[tuple[int, int]] = self._resolve_goals(
            goals, n_random_goals, random_seed
        )

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def run(self) -> None:
        """Execute the full exploration loop until all goals are reached."""

    # ------------------------------------------------------------------
    # Goal resolution
    # ------------------------------------------------------------------

    def _resolve_goals(
        self,
        goals: list[tuple[int, int]] | None,
        n_random_goals: int | None,
        random_seed: int | None,
    ) -> list[tuple[int, int]]:
        W, H = self._width, self._height

        if goals is not None:
            return list(goals)

        if n_random_goals is not None:
            rng = random.Random(random_seed)
            start = self._robot.position
            candidates = [
                (x, y)
                for x in range(W)
                for y in range(H)
                if (x, y) != start
            ]
            rng.shuffle(candidates)
            return candidates[:n_random_goals]

        # Default: 4-cell centre area (works for any even-sized maze)
        cx, cy = W // 2, H // 2
        centre = [
            (cx - 1, cy - 1),
            (cx,     cy - 1),
            (cx - 1, cy    ),
            (cx,     cy    ),
        ]
        return [c for c in centre if self._in_bounds(c[0], c[1])]

    # ------------------------------------------------------------------
    # Shared utilities
    # ------------------------------------------------------------------

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self._width and 0 <= y < self._height

    def _check_reset(self, robot: Robot, api: BaseAPI) -> bool:
        """Poll the MMS reset button once; resync robot state if pressed.

        The mouse's start cell is a fixed convention (maze origin, facing
        North) shared identically by the real MMS mouse and ``SimAPI``, so
        both ``ack_reset()`` (backend position tracking) and ``robot.reset()``
        (algorithm-side position/heading tracking) take no arguments.
        Returns True if a reset occurred, so callers know to discard any
        in-progress plan/search state before continuing.
        """
        if not api.was_reset():
            return False
        api.ack_reset()
        robot.reset()
        return True

    def _sense_and_update(
        self,
        maze_map: MazeMap,
        robot: Robot,
        api: BaseAPI,
    ) -> list[tuple[Direction, bool]]:
        """Sense all four walls of the current cell and update maze_map.

        Returns a list of ``(absolute_direction, is_new_wall)`` for all four
        directions.  ``is_new_wall`` is ``True`` only if the wall was not
        previously recorded in ``maze_map``.
        """
        x, y = robot.position
        results: list[tuple[Direction, bool]] = []

        readings = [
            (api.wall_front(), robot.wall_front_dir()),
            (api.wall_right(), robot.wall_right_dir()),
            (api.wall_back(),  robot.wall_back_dir()),
            (api.wall_left(),  robot.wall_left_dir()),
        ]

        for has_wall, direction in readings:
            already_known = maze_map.has_wall(x, y, direction)
            if has_wall and not already_known:
                maze_map.set_wall(x, y, direction)
                results.append((direction, True))
            else:
                results.append((direction, False))

        return results

    def _move_to(
        self,
        next_cell: tuple[int, int],
        robot: Robot,
        api: BaseAPI,
        logger: MetricsLogger,
        maze_map: MazeMap,
    ) -> None:
        """Rotate robot to face next_cell and execute one move_forward.

        Applies the minimum-turn strategy: 0, 1 right, 1 left, or 2 right turns.
        Updates ``robot``, calls API, increments logger counters, and marks
        the arrival cell as visited in ``maze_map``.
        """
        x, y = robot.position
        tx, ty = next_cell
        target_dir = _DELTA_TO_DIR[(tx - x, ty - y)]

        diff = (int(target_dir) - int(robot.heading)) % 4
        if diff == 1:
            api.turn_right(); robot.turn_right(); logger.log_turn()
        elif diff == 2:
            api.turn_right(); robot.turn_right(); logger.log_turn()
            api.turn_right(); robot.turn_right(); logger.log_turn()
        elif diff == 3:
            api.turn_left(); robot.turn_left(); logger.log_turn()
        # diff == 0: already facing the right direction

        api.move_forward()
        robot.move_forward()
        logger.log_move()
        maze_map.mark_visit(robot.x, robot.y)

    def _compute_goal_heuristic(
        self,
        maze_map: MazeMap,
        goals: list[tuple[int, int]],
    ) -> dict[tuple[int, int], int | float]:
        """Multi-source BFS backward from *goals* on the current partial map.

        Freespace assumption: any edge without a confirmed wall is treated as
        passable.  Returns ``h[cell]`` = BFS distance from ``cell`` to the
        nearest goal.  Cells unreachable under current knowledge return
        ``float('inf')``.
        """
        INF: float = float('inf')
        h: dict[tuple[int, int], int | float] = {
            (x, y): INF
            for y in range(maze_map.height)
            for x in range(maze_map.width)
        }

        queue: deque[tuple[int, int]] = deque()
        for g in goals:
            if self._in_bounds(g[0], g[1]):
                h[g] = 0
                queue.append(g)

        while queue:
            cx, cy = queue.popleft()
            d = h[(cx, cy)]
            for direction in Direction:
                if maze_map.has_wall(cx, cy, direction):
                    continue  # confirmed wall → impassable
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                if h[(nx, ny)] > d + 1:
                    h[(nx, ny)] = d + 1
                    queue.append((nx, ny))

        return h

    def _compute_start_heuristic(
        self,
        maze_map: MazeMap,
        start: tuple[int, int],
    ) -> dict[tuple[int, int], int | float]:
        """BFS forward from *start* on the current partial map.

        Returns ``h[cell]`` = BFS distance from ``cell`` to ``start``.
        Used by D*-Lite at initialisation (km adjustment handles robot movement).
        On an empty map (all cells free) this equals Manhattan distance.
        """
        INF: float = float('inf')
        h: dict[tuple[int, int], int | float] = {
            (x, y): INF
            for y in range(maze_map.height)
            for x in range(maze_map.width)
        }
        h[start] = 0
        queue: deque[tuple[int, int]] = deque([start])

        while queue:
            cx, cy = queue.popleft()
            d = h[(cx, cy)]
            for direction in Direction:
                if maze_map.has_wall(cx, cy, direction):
                    continue
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                if h[(nx, ny)] > d + 1:
                    h[(nx, ny)] = d + 1
                    queue.append((nx, ny))

        return h
