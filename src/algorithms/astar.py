"""
AStarExplorer: online A* with replanning-from-scratch on wall discovery.

Planning model
--------------
* Freespace assumption: every cell whose wall bitmask is 0 (unexplored) is
  treated as passable in all directions.
* Heuristic: multi-source BFS backward from the current goal set on the
  partial map.  Admissible because a partial map can only underestimate.
  Recomputed before every replanning event.
* Replanning trigger: after each move_forward, all four walls of the current
  cell are sensed.  If any newly confirmed wall lies on the remaining planned
  path, A* is rerun from the current position.

Multi-goal
----------
When a goal cell is reached, it is removed from the remaining set.
The BFS heuristic is recomputed over the updated goal set before the next
planning call.  Greedy nearest-goal ordering emerges naturally from the BFS.

GUI (MMS simulator, no-ops in headless mode)
---------------------------------------------
* Goal cells: 'G' (Dark Green) until reached, 'g' (Green) on arrival.
* Cell text: f-value of each expanded cell.  Cleared and redisplayed on
  every replanning event.
* New walls: displayed via set_wall after each sensing step.
"""
from __future__ import annotations

import heapq

from src.algorithms.base_algorithm import BaseAlgorithm, _DELTA_TO_DIR
from src.api.base_api import BaseAPI
from src.constants import DIR_TO_DELTA, DIR_TO_STR, Direction
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot


class AStarExplorer(BaseAlgorithm):
    """Online A* explorer: replans from scratch on every wall discovery."""

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
        super().__init__(api, maze_map, robot, logger, goals, n_random_goals, random_seed)

    # ------------------------------------------------------------------
    # Internal A* search
    # ------------------------------------------------------------------

    def _a_star(
        self,
        start: tuple[int, int],
        goal_set: set[tuple[int, int]],
        maze_map: MazeMap,
        h: dict[tuple[int, int], int | float],
    ) -> tuple[
        list[tuple[int, int]],
        int,
        dict[tuple[int, int], int | float],
        set[tuple[int, int]],
        set[tuple[int, int]],
    ]:
        """Standard A* on the current partial map.

        Returns ``(path, nodes_expanded, f_values, open_set, closed_set)``
        where ``path`` is ``[start, ..., goal]`` inclusive, ``f_values`` maps
        every expanded or open cell to its f-value, and ``open_set`` /
        ``closed_set`` are the final open-list / expanded-cell sets (for GUI
        display and the memory metric — ``len(open_set) + len(closed_set)``).
        Returns ``([], 0, {}, set(), set())`` if no path exists.

        ``nodes_expanded`` counts nodes removed from the open list (non-stale).
        """
        INF: float = float('inf')

        g_score: dict[tuple[int, int], int | float] = {start: 0}
        came_from: dict[tuple[int, int], tuple[int, int]] = {}

        # heap entries: (f, g, cell)
        open_heap: list[tuple[float | int, float | int, tuple[int, int]]] = [
            (h.get(start, INF), 0, start)
        ]
        open_set: set[tuple[int, int]] = {start}
        closed_set: set[tuple[int, int]] = set()
        nodes_expanded = 0

        while open_heap:
            f, g, current = heapq.heappop(open_heap)

            if current not in open_set:
                continue  # stale heap entry
            open_set.discard(current)

            if current in goal_set:
                # Reconstruct path from start to current
                closed_set.add(current)
                nodes_expanded += 1
                path: list[tuple[int, int]] = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start)
                path.reverse()
                f_values = {
                    c: g_score[c] + h.get(c, INF) for c in closed_set | open_set
                }
                return path, nodes_expanded, f_values, open_set, closed_set

            closed_set.add(current)
            nodes_expanded += 1

            cx, cy = current
            for direction in Direction:
                if maze_map.has_wall(cx, cy, direction):
                    continue  # confirmed wall
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                neighbour = (nx, ny)
                if neighbour in closed_set:
                    continue
                tentative_g: int | float = g + 1
                if tentative_g < g_score.get(neighbour, INF):
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    f_new = tentative_g + h.get(neighbour, INF)
                    heapq.heappush(open_heap, (f_new, tentative_g, neighbour))
                    open_set.add(neighbour)

        f_values = {c: g_score[c] + h.get(c, INF) for c in closed_set | open_set}
        return [], nodes_expanded, f_values, open_set, closed_set

    def _path_has_blocked_edge(
        self,
        path: list[tuple[int, int]],
        maze_map: MazeMap,
    ) -> bool:
        """Return True if any consecutive edge in *path* is now confirmed blocked."""
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            d = _DELTA_TO_DIR.get((x2 - x1, y2 - y1))
            if d is not None and maze_map.has_wall(x1, y1, d):
                return True
        return False

    @staticmethod
    def _format_f(value: int | float) -> str:
        """Format an f-value for the 10-char cell-text budget: 'f-XXX' or 'f-inf'."""
        if value == float('inf'):
            return "f-inf"
        return f"f-{int(value):03d}"

    def _gui_show_search(
        self,
        f_values: dict[tuple[int, int], int | float],
        open_set: set[tuple[int, int]],
        closed_set: set[tuple[int, int]],
        remaining_goals: list[tuple[int, int]],
        api: BaseAPI,
    ) -> None:
        """Display live search progress: expanded cells (blue), open cells
        (dark red), each with its f-value; goal markers are reapplied on top
        since they were just wiped by clear_all_color()."""
        api.clear_all_color()
        api.clear_all_text()
        for cx, cy in closed_set:
            api.set_color(cx, cy, 'b')
            api.set_text(cx, cy, self._format_f(f_values.get((cx, cy), float('inf'))))
        for cx, cy in open_set:
            api.set_color(cx, cy, 'R')
            api.set_text(cx, cy, self._format_f(f_values.get((cx, cy), float('inf'))))
        for gx, gy in remaining_goals:
            api.set_color(gx, gy, 'G')

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """Online A* exploration loop."""
        maze_map = self._maze_map
        robot = self._robot
        api = self._api
        logger = self._logger

        logger.start()
        maze_map.mark_visit(robot.x, robot.y)

        # Sense start cell before planning
        self._sense_and_update(maze_map, robot, api)

        remaining_goals = list(self._goals)

        # GUI: mark all goals
        for gx, gy in remaining_goals:
            api.set_color(gx, gy, 'G')

        # GUI: initial state — empty text except the start cell's f = h(start)
        h_init = self._compute_goal_heuristic(maze_map, remaining_goals)
        api.clear_all_text()
        api.set_text(robot.x, robot.y, self._format_f(h_init.get(robot.position, float('inf'))))

        # Handle edge case: already at a goal
        goal_set = set(remaining_goals)
        while robot.position in goal_set:
            remaining_goals.remove(robot.position)
            api.set_color(robot.x, robot.y, 'g')
            goal_set = set(remaining_goals)

        while remaining_goals:
            goal_set = set(remaining_goals)

            # Plan
            h = self._compute_goal_heuristic(maze_map, remaining_goals)
            path, _, f_values, open_set, closed_set = self._a_star(
                robot.position, goal_set, maze_map, h
            )

            if len(path) < 2:
                break  # No path reachable (shouldn't happen with freespace assumption)

            self._gui_show_search(f_values, open_set, closed_set, remaining_goals, api)

            # Execute path step by step; path[0] = current position
            step_idx = 1
            while step_idx < len(path):
                next_cell = path[step_idx]

                # Move and sense
                self._move_to(next_cell, robot, api, logger, maze_map)
                new_wall_events = self._sense_and_update(maze_map, robot, api)

                # Reset handling: abort the current path, replan from the restored start
                if self._check_reset(robot, api):
                    maze_map.mark_visit(robot.x, robot.y)
                    self._sense_and_update(maze_map, robot, api)
                    break  # replan in next outer iteration

                # GUI: display newly confirmed walls
                for direction, is_new in new_wall_events:
                    if is_new:
                        api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])

                # Check: goal reached?
                if robot.position in goal_set:
                    remaining_goals.remove(robot.position)
                    api.set_color(robot.x, robot.y, 'g')
                    break  # replan in next outer iteration

                # Check: do any new walls block the remaining path?
                has_new = any(is_new for _, is_new in new_wall_events)
                if has_new:
                    remaining_path = [robot.position] + path[step_idx + 1:]
                    if self._path_has_blocked_edge(remaining_path, maze_map):
                        # ---- Replanning event ----
                        logger.start_plan_timer()
                        h_new = self._compute_goal_heuristic(maze_map, remaining_goals)
                        new_path, n_exp, f_values, open_set, closed_set = self._a_star(
                            robot.position, set(remaining_goals), maze_map, h_new
                        )
                        residual = h_new.get(robot.position, 0)
                        memory = len(open_set) + len(closed_set)
                        logger.log_replanning_event(
                            robot.position, n_exp, residual, memory
                        )
                        self._gui_show_search(
                            f_values, open_set, closed_set, remaining_goals, api
                        )
                        path = new_path
                        step_idx = 1
                        continue  # restart inner loop with new path

                step_idx += 1

        logger.stop()
        logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
