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
import sys
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

    # GUI legend: overridden per subclass; each entry is (symbol, meaning).
    # Read off the class (not an instance) by run.py to build the legend window.
    LEGEND: list[tuple[str, str]] = []

    def __init__(
        self,
        api: BaseAPI,
        maze_map: MazeMap,
        robot: Robot,
        logger: MetricsLogger,
        goals: list[tuple[int, int]] | None = None,
        n_random_goals: int | None = None,
        random_seed: int | None = None,
        heuristic: str = "min_path",
    ) -> None:
        self._api = api
        self._maze_map = maze_map
        self._robot = robot
        self._logger = logger
        self._start_pos: tuple[int, int] = robot.position
        self._width: int = api.maze_width()
        self._height: int = api.maze_height()
        # "min_path" (default): wall-aware BFS distance, via _compute_goal_heuristic.
        # "manhattan": straight-line distance, via _compute_heuristic. Currently only
        # consulted by AStarExplorer — DStarLiteExplorer always uses its own
        # Manhattan-to-s_start heuristic regardless of this value (see _h() in
        # dstar_lite.py; a different mathematical role, tied to its Key/km invariant).
        self._heuristic: str = heuristic
        # True only when neither an explicit goal list nor a random-goal count
        # was supplied, i.e. the default centre-area goal kicked in. Captured
        # from which argument was given, not from the resolved coordinates, so
        # an explicit list identical to the centre area is still treated as a
        # real multi-goal request (see run() early-termination logic).
        self._is_default_goal: bool = goals is None and n_random_goals is None
        self._goals: list[tuple[int, int]] = self._resolve_goals(
            goals, n_random_goals, random_seed
        )
        # Goals reached so far in the current run, in reach order. Consulted
        # by _gui_show_termination and by both subclasses' own live-display
        # goal-color-persistence logic (redrawn on every replan so 'g'
        # markers survive clear_all_color()).
        self._reached_goals: list[tuple[int, int]] = []

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

    def _report_event(self, message: str) -> None:
        """Print a diagnostic line to stderr.

        stdout is reserved exclusively for the MMS stdin/stdout protocol, so
        all human-readable diagnostics go to stderr (a no-op for the protocol
        under both MmsAPI and SimAPI).
        """
        print(message, file=sys.stderr)

    def _report_walls(self, maze_map: MazeMap, x: int, y: int) -> None:
        """One consolidated stderr line for all four walls at (x, y).

        Reports the cell's complete now-known wall status (not just this
        sensing pass's deltas), in Direction's canonical N, E, S, W order,
        with '_' marking an absent wall. Callers must gate this on "at least
        one new wall found this pass" themselves, so verbosity actually drops
        relative to the old one-line-per-wall format.
        """
        parts = [
            DIR_TO_STR[direction] if maze_map.has_wall(x, y, direction) else '_'
            for direction in Direction
        ]
        self._report_event(f"[WALL] ({x}, {y}) {' '.join(parts)}")

    def _report_replan(self, record: dict) -> None:
        """One stderr line for a replanning-event record (see MetricsLogger).

        cost_ratio and time_ms are rounded to 2 decimals; cost_ratio is
        printed as the literal string 'None' when the logger recorded it as
        None (residual_distance == 0).
        """
        cost_ratio = record['cost_ratio']
        cost_ratio_str = f"{cost_ratio:.2f}" if cost_ratio is not None else "None"
        self._report_event(
            f"[REPLAN] pos=({record['position'][0]}, {record['position'][1]}) "
            f"expanded={record['nodes_expanded']} "
            f"residual={record['residual_distance']} "
            f"cost_ratio={cost_ratio_str} "
            f"time_ms={record['planning_time_s'] * 1000:.2f}"
        )

    def _gui_show_termination(self, api: BaseAPI) -> None:
        """Final GUI frame: clear all non-goal decoration, mark goal cells only.

        Called once, unconditionally, at the end of run() in both explorers
        (regardless of whether the loop exited because the goal condition was
        satisfied or because it got stuck — the maze corpus used by this
        project is verified solvable by both algorithms, so the stuck path is
        not expected to execute in practice).

        Reached goals show 'g' (Green); in a true multi-goal run (an explicit
        goals/n_random_goals request of >= 2 cells) they're additionally
        labelled with their 1-based reach order. The default centre-area case
        is not a multi-goal run for this purpose — only one of its 4 cells is
        ever reached by design — so its unreached cells stay 'G' (Dark Green)
        and no order text is shown for the one reached cell either.
        """
        api.clear_all_color()
        api.clear_all_text()
        multi_goal = not self._is_default_goal and len(self._goals) >= 2
        for i, (gx, gy) in enumerate(self._reached_goals, start=1):
            api.set_color(gx, gy, 'g')
            if multi_goal:
                api.set_text(gx, gy, str(i))
        if self._is_default_goal:
            reached = set(self._reached_goals)
            for gx, gy in self._goals:
                if (gx, gy) not in reached:
                    api.set_color(gx, gy, 'G')

    def _display_maze_outline(self, api: BaseAPI) -> None:
        """Draw the maze's outer perimeter walls (cosmetic only).

        This affects only what is drawn on screen before exploration starts;
        MazeMap's own knowledge of the border is still built up by sensing,
        unchanged. A no-op under SimAPI.
        """
        W, H = self._width, self._height
        for x in range(W):
            api.set_wall(x, 0, 's')
            api.set_wall(x, H - 1, 'n')
        for y in range(H):
            api.set_wall(0, y, 'w')
            api.set_wall(W - 1, y, 'e')

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

    def _compute_heuristic(
        self,
        maze_map: MazeMap,
        goals: list[tuple[int, int]],
    ) -> dict[tuple[int, int], int | float]:
        """Dispatch on self._heuristic for the goal-directed planning heuristic.

        ``"min_path"`` (default): wall-aware BFS backward from *goals*, i.e.
        ``_compute_goal_heuristic``. ``"manhattan"``: straight-line ``|dx|+|dy|``
        distance to the nearest goal, ignoring wall knowledge entirely.
        """
        if self._heuristic == "manhattan":
            return {
                (x, y): min(
                    (abs(x - gx) + abs(y - gy) for gx, gy in goals),
                    default=float('inf'),
                )
                for y in range(maze_map.height)
                for x in range(maze_map.width)
            }
        return self._compute_goal_heuristic(maze_map, goals)

    def _compute_start_heuristic(
        self,
        maze_map: MazeMap,
        start: tuple[int, int],
    ) -> dict[tuple[int, int], int | float]:
        """BFS forward from *start* on the current partial map.

        Returns ``h[cell]`` = BFS distance from ``cell`` to ``start``.
        Currently unused: DStarLiteExplorer computes its own Manhattan-distance
        heuristic inline (``_h()`` in dstar_lite.py) rather than calling this.
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
