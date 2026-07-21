"""
AStarExplorer: online A* with replanning-from-scratch on wall discovery.

Planning model
--------------
* Freespace assumption: every cell whose wall bitmask is 0 (unexplored) is
  treated as passable in all directions.
* Heuristic: selectable via self._heuristic ('min_path', default: multi-source
  BFS backward from the current goal set on the partial map, admissible
  because a partial map can only underestimate; or 'manhattan': straight-line
  distance to the nearest goal, ignoring walls). Recomputed before every
  replanning event via _compute_heuristic.
* Replanning trigger: after each move_forward, all four walls of the current
  cell are sensed.  If any newly confirmed wall lies on the remaining planned
  path, A* is rerun from the current position.

Multi-goal
----------
When a goal cell is reached, it is removed from the remaining set.
The BFS heuristic is recomputed over the updated goal set before the next
planning call.  Greedy nearest-goal ordering emerges naturally from the BFS.

Default goal
------------
When neither an explicit goal list nor a random-goal count is given, the goal
is the maze's 4-cell centre *area* and the run terminates as soon as the first
of those cells is reached (self._is_default_goal).

GUI (MMS simulator, no-ops in headless mode)
---------------------------------------------
* Expanded cells: 'b' (Blue); open-list cells: 'R' (Dark Red); each labelled
  with its f/h values ('f-XXXh-YYY').
* Planned path: 'c' (Cyan).
* Goal cells: 'G' (Dark Green) until reached, 'g' (Green) on arrival; reached
  goals are redrawn on every replan so they survive clear_all_color().
* Perimeter outline drawn once at startup; new walls displayed via set_wall
  after each sensing step.
* Colours/text cleared and fully redrawn on every plan and replan.
* On termination: non-goal cells cleared; goal cells left marked (see
  _gui_show_termination in base_algorithm.py).

stderr diagnostics
------------------
Wall discoveries ('[WALL] (x, y) n e s w' — one consolidated line per sensing
event) and replanning events ('[REPLAN] ...', cost_ratio/time_ms rounded to
2 decimals) are reported to stderr via _report_walls/_report_replan (never
stdout — the MMS protocol channel).
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

    LEGEND = [
        ("b  (Blue)",      "Expanded (closed list)"),
        ("R  (Dark Red)",  "Open list"),
        ("c  (Cyan)",      "Planned path / traversed path"),
        ("G  (Dark Green)", "Goal cell, not yet reached"),
        ("g  (Green)",     "Goal cell, reached"),
        ("f-XXX",          "f-value of an expanded/open cell"),
        ("f-XXXh-YYY",     "f-value and h-value of an expanded/open cell"),
    ]

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
        super().__init__(
            api, maze_map, robot, logger, goals, n_random_goals, random_seed, heuristic,
        )

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

        # heap entries: (f, h, cell) — h is the tie-break key (already computed
        # to derive f), aligning exploration order with D*-Lite's k2 = min(g, rhs)
        # tie-break. Admissibility (hence optimality) is tie-break-independent.
        open_heap: list[tuple[float | int, float | int, tuple[int, int]]] = [
            (h.get(start, INF), h.get(start, INF), start)
        ]
        open_set: set[tuple[int, int]] = {start}
        closed_set: set[tuple[int, int]] = set()
        nodes_expanded = 0

        while open_heap:
            f, _tie, current = heapq.heappop(open_heap)

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
                tentative_g: int | float = g_score[current] + 1
                if tentative_g < g_score.get(neighbour, INF):
                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g
                    h_neighbour = h.get(neighbour, INF)
                    f_new = tentative_g + h_neighbour
                    heapq.heappush(open_heap, (f_new, h_neighbour, neighbour))
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
        """Format an f-value into a 3-char left-justified slot: 'f-5  ', 'f-inf'.

        3 characters is the correct budget: on a 16x16 maze a pathological
        shortest path can exceed 100 cells, so 2 digits is not always enough.
        """
        text = "inf" if value == float('inf') else str(int(value))
        return f"f-{text:<3}"

    @staticmethod
    def _format_fh(f_value: int | float, h_value: int | float) -> str:
        """Format f- and h-values into two 3-char slots: 'f-XXXh-YYY' (10 chars)."""
        f_text = "inf" if f_value == float('inf') else str(int(f_value))
        h_text = "inf" if h_value == float('inf') else str(int(h_value))
        return f"f-{f_text:<3}h-{h_text:<3}"

    def _gui_show_search(
        self,
        f_values: dict[tuple[int, int], int | float],
        h_values: dict[tuple[int, int], int | float],
        open_set: set[tuple[int, int]],
        closed_set: set[tuple[int, int]],
        remaining_goals: list[tuple[int, int]],
        reached_goals: list[tuple[int, int]],
        path: list[tuple[int, int]],
        api: BaseAPI,
    ) -> None:
        """Display live search progress after a full clear.

        Draw order (later wins on overlap): expanded cells (blue) and open
        cells (dark red) with their f/h text first; then the planned path
        (cyan); then goal markers last, so goals remain visually
        distinguishable — remaining goals dark green, previously reached
        goals green (reapplied because clear_all_color() wiped them).
        """
        INF = float('inf')
        api.clear_all_color()
        api.clear_all_text()
        for cx, cy in closed_set:
            api.set_color(cx, cy, 'b')
            api.set_text(cx, cy, self._format_fh(
                f_values.get((cx, cy), INF), h_values.get((cx, cy), INF)))
        for cx, cy in open_set:
            api.set_color(cx, cy, 'R')
            api.set_text(cx, cy, self._format_fh(
                f_values.get((cx, cy), INF), h_values.get((cx, cy), INF)))
        # Planned path (excluding the robot's current cell, path[0]).
        for cx, cy in path[1:]:
            api.set_color(cx, cy, 'c')
        # Goal markers drawn last so they win over cyan on goal cells.
        for gx, gy in remaining_goals:
            api.set_color(gx, gy, 'G')
        for gx, gy in reached_goals:
            api.set_color(gx, gy, 'g')

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

        # GUI: draw the maze perimeter before anything else (cosmetic).
        self._display_maze_outline(api)

        maze_map.mark_visit(robot.x, robot.y)

        # Sense start cell before planning; display and report any walls found.
        initial_walls = self._sense_and_update(maze_map, robot, api)
        for direction, is_new in initial_walls:
            if is_new:
                api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])
        if any(is_new for _, is_new in initial_walls):
            self._report_walls(maze_map, robot.x, robot.y)

        remaining_goals = list(self._goals)

        # GUI: mark all goals
        for gx, gy in remaining_goals:
            api.set_color(gx, gy, 'G')

        # GUI: initial state — empty text except the start cell's f = h = h(start)
        h_init = self._compute_heuristic(maze_map, remaining_goals)
        api.clear_all_text()
        h_start = h_init.get(robot.position, float('inf'))
        api.set_text(robot.x, robot.y, self._format_fh(h_start, h_start))

        # Handle edge case: already at a goal
        goal_set = set(remaining_goals)
        while robot.position in goal_set:
            remaining_goals.remove(robot.position)
            api.set_color(robot.x, robot.y, 'g')
            self._reached_goals.append(robot.position)
            if self._is_default_goal:
                # Default goal is a single 4-cell *area*: stop at the first reached.
                remaining_goals.clear()
            goal_set = set(remaining_goals)

        while remaining_goals:
            goal_set = set(remaining_goals)

            # Plan
            h = self._compute_heuristic(maze_map, remaining_goals)
            path, _, f_values, open_set, closed_set = self._a_star(
                robot.position, goal_set, maze_map, h
            )

            if len(path) < 2:
                break  # No path reachable (shouldn't happen with freespace assumption)

            self._gui_show_search(
                f_values, h, open_set, closed_set,
                remaining_goals, self._reached_goals, path, api,
            )

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

                # GUI: display newly confirmed walls; report to stderr as one line
                for direction, is_new in new_wall_events:
                    if is_new:
                        api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])
                if any(is_new for _, is_new in new_wall_events):
                    self._report_walls(maze_map, robot.x, robot.y)

                # Check: goal reached?
                if robot.position in goal_set:
                    remaining_goals.remove(robot.position)
                    api.set_color(robot.x, robot.y, 'g')
                    self._reached_goals.append(robot.position)
                    if self._is_default_goal:
                        remaining_goals.clear()
                    break  # replan in next outer iteration

                # Check: do any new walls block the remaining path?
                has_new = any(is_new for _, is_new in new_wall_events)
                if has_new:
                    remaining_path = [robot.position] + path[step_idx + 1:]
                    if self._path_has_blocked_edge(remaining_path, maze_map):
                        # ---- Replanning event ----
                        logger.start_plan_timer()
                        h_new = self._compute_heuristic(maze_map, remaining_goals)
                        new_path, n_exp, f_values, open_set, closed_set = self._a_star(
                            robot.position, set(remaining_goals), maze_map, h_new
                        )
                        logger.stop_plan_timer()

                        memory = len(open_set) + len(closed_set)
                        # residual_distance is always the wall-aware BFS distance,
                        # regardless of self._heuristic (see MetricsLogger schema
                        # doc, §4.1): reuse h_new when it already is that map
                        # (heuristic == "min_path"), else compute it explicitly.
                        residual_map = (
                            h_new if self._heuristic == "min_path"
                            else self._compute_goal_heuristic(maze_map, remaining_goals)
                        )
                        residual = residual_map.get(robot.position, 0)
                        logger.log_replanning_event(
                            robot.position, n_exp, residual, memory
                        )
                        self._report_replan(logger.replanning_events[-1])
                        self._gui_show_search(
                            f_values, h_new, open_set, closed_set,
                            remaining_goals, self._reached_goals, new_path, api,
                        )
                        path = new_path
                        step_idx = 1
                        continue  # restart inner loop with new path

                step_idx += 1

        self._gui_show_termination(api)
        logger.stop()
        logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
