"""
DStarLiteExplorer: D*-Lite with incremental replanning on wall discovery.

Algorithm reference
-------------------
Koenig & Likhachev, "D* Lite", AAAI 2002.

Planning model
--------------
* Backward search: computes g(s) = shortest known cost from s to the nearest
  remaining goal (goal cells have rhs = 0).
* Freespace assumption: edges with no confirmed wall treated as passable
  (cost 1).  Confirmed walls have cost +∞.
* Heuristic: Manhattan distance from each cell to the robot's current
  position (consistent, never recomputed).  The km accumulator absorbs
  incremental changes as the robot moves.
* Replanning: when a wall is discovered, only the affected inconsistent nodes
  are re-processed via ComputeShortestPath — not the entire search tree.

Key definitions
---------------
g(s)   – current best-known cost from s to a goal (backward).
rhs(s) – one-step lookahead: min over passable neighbours n of (1 + g(n)).
         Goal cells always have rhs = 0.
A node is inconsistent when g(s) ≠ rhs(s).
Key(s) = (min(g,rhs) + h(s, s_start) + km,  min(g,rhs))

nodes_expanded (logged)
  Counts only extractions from the inconsistency queue whose stored key
  matches the current CalculateKey at extraction time (non-stale).
  Stale-key re-insertions are NOT counted.

memory_occupancy (logged)
  |{s : g(s) ≠ ∞  OR  rhs(s) ≠ ∞}|.  Monotonically non-decreasing.

Multi-goal
----------
All goals initialised with rhs = 0.  When goal g_reached is reached:
  rhs(g_reached) = g(g_reached) = ∞, neighbours updated, km adjusted,
  ComputeShortestPath re-run to route toward the next goal.

GUI (MMS simulator; all calls are no-ops in headless mode)
-----------------------------------------------------------
* Cell text: "g:X r:Y" format (e.g. "g:∞ r:0").
* Goal cells: 'G' until reached, 'g' on arrival.
* Inconsistent nodes (in queue): 'R' (Dark Red).
* Consistent nodes (processed / expanded): 'b' (Blue).
* Trivial nodes (g=rhs=∞): clear color.
* New walls: set_wall after each sensing step.
"""
from __future__ import annotations

import heapq
from collections import defaultdict

from src.algorithms.base_algorithm import BaseAlgorithm
from src.api.base_api import BaseAPI
from src.constants import DIR_TO_DELTA, DIR_TO_STR, Direction
from src.maze_map import MazeMap
from src.metrics.logger import MetricsLogger
from src.robot import Robot

_INF: float = float('inf')


# ---------------------------------------------------------------------------
# Priority queue with lazy deletion
# ---------------------------------------------------------------------------

class _DStarQueue:
    """Min-heap priority queue for D*-Lite inconsistent nodes.

    Keys are (k1, k2) tuples; entries are lazily invalidated via a sequence
    counter so that re-insertions with updated keys are handled correctly.
    """

    def __init__(self) -> None:
        self._heap: list[tuple] = []   # (key, seq, state)
        self._seq: int = 0
        self._valid: dict[tuple[int, int], int] = {}  # state → valid seq

    def insert(self, state: tuple[int, int], key: tuple[float, float]) -> None:
        """Insert or update *state* with *key* (overwrites any previous entry)."""
        seq = self._seq
        self._seq += 1
        self._valid[state] = seq
        heapq.heappush(self._heap, (key, seq, state))

    def remove(self, state: tuple[int, int]) -> None:
        """Mark *state* as removed (lazy)."""
        self._valid.pop(state, None)

    def _is_valid(self, key: tuple, seq: int, state: tuple[int, int]) -> bool:
        return state in self._valid and self._valid[state] == seq

    # ------------------------------------------------------------------
    # Peek (does NOT remove)

    def top(self) -> tuple[int, int] | None:
        """Return the state with the minimum key without removing it."""
        while self._heap and not self._is_valid(*self._heap[0]):
            heapq.heappop(self._heap)
        return self._heap[0][2] if self._heap else None

    def top_key(self) -> tuple[float, float]:
        """Return the minimum key without removing the entry."""
        while self._heap and not self._is_valid(*self._heap[0]):
            heapq.heappop(self._heap)
        return self._heap[0][0] if self._heap else (_INF, _INF)

    # ------------------------------------------------------------------
    # Pop (removes the top valid entry)

    def pop(self) -> tuple[tuple[float, float], tuple[int, int]] | tuple[None, None]:
        """Remove and return ``(key, state)`` of the minimum valid entry."""
        while self._heap:
            key, seq, state = heapq.heappop(self._heap)
            if self._is_valid(key, seq, state):
                del self._valid[state]
                return key, state
        return None, None

    def __contains__(self, state: tuple[int, int]) -> bool:
        return state in self._valid

    def size(self) -> int:
        return len(self._valid)

    @property
    def is_empty(self) -> bool:
        return len(self._valid) == 0


# ---------------------------------------------------------------------------
# D*-Lite explorer
# ---------------------------------------------------------------------------

class DStarLiteExplorer(BaseAlgorithm):
    """D*-Lite: incremental replanning explorer."""

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

        # D*-Lite state (initialised in run())
        self._g: dict[tuple[int, int], float] = defaultdict(lambda: _INF)
        self._rhs: dict[tuple[int, int], float] = defaultdict(lambda: _INF)
        self._U: _DStarQueue = _DStarQueue()
        self._km: float = 0.0
        self._s_start: tuple[int, int] = self._robot.position   # current robot pos
        self._remaining_goal_set: set[tuple[int, int]] = set()

    # ------------------------------------------------------------------
    # D*-Lite internals
    # ------------------------------------------------------------------

    def _h(self, s: tuple[int, int]) -> float:
        """Heuristic: Manhattan distance from *s* to the current robot position."""
        return float(abs(s[0] - self._s_start[0]) + abs(s[1] - self._s_start[1]))

    def _calc_key(self, s: tuple[int, int]) -> tuple[float, float]:
        min_g_rhs = min(self._g[s], self._rhs[s])
        return (min_g_rhs + self._h(s) + self._km, min_g_rhs)

    def _update_rhs(self, u: tuple[int, int]) -> None:
        """Recompute rhs(u) = min over passable neighbours n of (1 + g(n))."""
        if u in self._remaining_goal_set:
            self._rhs[u] = 0.0
            return
        best = _INF
        ux, uy = u
        for direction in Direction:
            if self._maze_map.has_wall(ux, uy, direction):
                continue
            dx, dy = DIR_TO_DELTA[direction]
            nx, ny = ux + dx, uy + dy
            if not self._in_bounds(nx, ny):
                continue
            val = 1.0 + self._g[(nx, ny)]
            if val < best:
                best = val
        self._rhs[u] = best

    def _update_vertex(self, u: tuple[int, int]) -> None:
        """Maintain U: insert/update/remove u depending on consistency."""
        if self._g[u] != self._rhs[u]:
            self._U.insert(u, self._calc_key(u))
        elif u in self._U:
            self._U.remove(u)

    def _gui_cell_text(self, x: int, y: int) -> str:
        g_val = self._g[(x, y)]
        r_val = self._rhs[(x, y)]
        g_str = "∞" if g_val == _INF else str(int(g_val))
        r_str = "∞" if r_val == _INF else str(int(r_val))
        # MMS cell text max 10 chars; keep it short
        return f"g:{g_str} r:{r_str}"[:10]

    def _compute_shortest_path(self) -> int:
        """Run D*-Lite's ComputeShortestPath from the current s_start.

        Returns the number of non-stale state expansions (nodes_expanded metric).
        """
        nodes_expanded = 0

        while True:
            top_key = self._U.top_key()
            k_start = self._calc_key(self._s_start)

            # Termination: queue is exhausted or s_start is consistent and optimal
            if top_key >= k_start and self._rhs[self._s_start] == self._g[self._s_start]:
                break

            u = self._U.top()
            if u is None:
                break  # queue empty: no path exists

            k_old = top_key
            k_new = self._calc_key(u)

            if k_old < k_new:
                # ---- Stale key: re-insert with updated key (not an expansion) ----
                self._U.insert(u, k_new)

            elif self._g[u] > self._rhs[u]:
                # ---- Overconsistent: lower g to rhs ----
                self._U.pop()
                self._g[u] = self._rhs[u]
                nodes_expanded += 1

                self._api.set_color(u[0], u[1], 'b')   # GUI: consistent
                self._api.set_text(u[0], u[1], self._gui_cell_text(u[0], u[1]))

                ux, uy = u
                for direction in Direction:
                    if self._maze_map.has_wall(ux, uy, direction):
                        continue
                    dx, dy = DIR_TO_DELTA[direction]
                    nx, ny = ux + dx, uy + dy
                    if not self._in_bounds(nx, ny):
                        continue
                    v = (nx, ny)
                    if v in self._remaining_goal_set:
                        continue
                    new_val = 1.0 + self._g[u]
                    if new_val < self._rhs[v]:
                        self._rhs[v] = new_val
                        self._update_vertex(v)
                        if v in self._U:
                            self._api.set_color(v[0], v[1], 'R')  # GUI: inconsistent

            else:
                # ---- Underconsistent: raise g to ∞ ----
                g_old = self._g[u]
                self._U.pop()
                self._g[u] = _INF
                nodes_expanded += 1

                self._api.set_text(u[0], u[1], self._gui_cell_text(u[0], u[1]))

                ux, uy = u
                # Update u itself (g → ∞, may become inconsistent)
                if u not in self._remaining_goal_set:
                    # rhs(u) unchanged (depends on neighbours' g, not own g)
                    self._update_vertex(u)
                    if u in self._U:
                        self._api.set_color(ux, uy, 'R')
                    elif self._g[u] == self._rhs[u] == _INF:
                        self._api.clear_color(ux, uy)

                # Update neighbours that used g(u)
                for direction in Direction:
                    if self._maze_map.has_wall(ux, uy, direction):
                        continue
                    dx, dy = DIR_TO_DELTA[direction]
                    nx, ny = ux + dx, uy + dy
                    if not self._in_bounds(nx, ny):
                        continue
                    v = (nx, ny)
                    if v in self._remaining_goal_set:
                        continue
                    if self._rhs[v] == 1.0 + g_old:
                        self._update_rhs(v)
                    self._update_vertex(v)
                    if v in self._U:
                        self._api.set_color(v[0], v[1], 'R')
                    elif self._g[v] == self._rhs[v] == _INF:
                        self._api.clear_color(v[0], v[1])

        return nodes_expanded

    def _memory_occupancy(self) -> int:
        """Count cells where g ≠ ∞ OR rhs ≠ ∞ (monotonically non-decreasing)."""
        nontrivial_g = {s for s, v in self._g.items() if v != _INF}
        nontrivial_rhs = {s for s, v in self._rhs.items() if v != _INF}
        return len(nontrivial_g | nontrivial_rhs)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        """D*-Lite exploration loop."""
        maze_map = self._maze_map
        robot = self._robot
        api = self._api
        logger = self._logger

        logger.start()

        # ---- Initialise D*-Lite ----
        self._g = defaultdict(lambda: _INF)
        self._rhs = defaultdict(lambda: _INF)
        self._U = _DStarQueue()
        self._km = 0.0
        self._s_start = robot.position
        s_last = robot.position
        self._remaining_goal_set = set(self._goals)

        # rhs = 0 for all goal cells; insert into U
        for g in self._remaining_goal_set:
            self._rhs[g] = 0.0
            self._U.insert(g, self._calc_key(g))

        # GUI: initialise cell text and goal colours
        for y in range(self._height):
            for x in range(self._width):
                s = (x, y)
                if s in self._remaining_goal_set:
                    api.set_text(x, y, "g:∞ r:0")
                    api.set_color(x, y, 'G')
                else:
                    api.set_text(x, y, "g:∞ r:∞")

        # Sense start cell before initial plan
        maze_map.mark_visit(robot.x, robot.y)
        initial_walls = self._sense_and_update(maze_map, robot, api)

        # Apply walls discovered at start
        for direction, is_new in initial_walls:
            if is_new:
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = robot.x + dx, robot.y + dy
                pos = robot.position
                if self._in_bounds(nx, ny):
                    v = (nx, ny)
                    if v not in self._remaining_goal_set:
                        self._update_rhs(v)
                        self._update_vertex(v)
                if pos not in self._remaining_goal_set:
                    self._update_rhs(pos)
                    self._update_vertex(pos)
                api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])

        # Initial ComputeShortestPath
        self._compute_shortest_path()

        # ---- Main navigation loop ----
        while self._remaining_goal_set:
            self._s_start = robot.position

            if self._rhs[self._s_start] == _INF:
                break  # No reachable goal

            # Find next cell: argmin over passable neighbours n of (1 + g(n))
            sx, sy = self._s_start
            best_cost = _INF
            best_next: tuple[int, int] | None = None
            for direction in Direction:
                if maze_map.has_wall(sx, sy, direction):
                    continue
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = sx + dx, sy + dy
                if not self._in_bounds(nx, ny):
                    continue
                cost = 1.0 + self._g[(nx, ny)]
                if cost < best_cost:
                    best_cost = cost
                    best_next = (nx, ny)

            if best_next is None or best_cost == _INF:
                break  # Stuck

            # ---- Execute move ----
            self._move_to(best_next, robot, api, logger, maze_map)
            self._s_start = robot.position

            # Update km: h(s_last, s_current) = Manhattan = 1 per adjacent move
            self._km += abs(self._s_start[0] - s_last[0]) + abs(self._s_start[1] - s_last[1])
            s_last = self._s_start

            # ---- Check goal reached ----
            if robot.position in self._remaining_goal_set:
                self._remaining_goal_set.discard(robot.position)
                api.set_color(robot.x, robot.y, 'g')

                if not self._remaining_goal_set:
                    break

                # Remove reached goal from D*-Lite: rhs and g → ∞
                reached = robot.position
                self._rhs[reached] = _INF
                self._g[reached] = _INF
                if reached in self._U:
                    self._U.remove(reached)

                # Reinitialise remaining goals (still have rhs=0)
                # and repair inconsistencies introduced by removing the reached goal
                rx, ry = reached
                for direction in Direction:
                    dx, dy = DIR_TO_DELTA[direction]
                    nx, ny = rx + dx, ry + dy
                    if not self._in_bounds(nx, ny):
                        continue
                    v = (nx, ny)
                    if v in self._remaining_goal_set:
                        continue
                    if self._rhs[v] != _INF:  # v may have used reached as successor
                        self._update_rhs(v)
                        self._update_vertex(v)

                # Replan to next goal (goal reaching is not a "replanning event")
                self._compute_shortest_path()
                continue

            # ---- Sense walls ----
            new_wall_events = self._sense_and_update(maze_map, robot, api)

            # ---- Apply wall changes to D*-Lite ----
            has_new = any(is_new for _, is_new in new_wall_events)
            if has_new:
                for direction, is_new in new_wall_events:
                    if not is_new:
                        continue
                    api.set_wall(robot.x, robot.y, DIR_TO_STR[direction])
                    dx, dy = DIR_TO_DELTA[direction]
                    nx, ny = robot.x + dx, robot.y + dy
                    # Both sides of the newly confirmed wall need rhs updates
                    pos = robot.position
                    if pos not in self._remaining_goal_set:
                        self._update_rhs(pos)
                        self._update_vertex(pos)
                    if self._in_bounds(nx, ny):
                        v = (nx, ny)
                        if v not in self._remaining_goal_set:
                            self._update_rhs(v)
                            self._update_vertex(v)

                # ---- Replanning event (only when plan was actually modified) ----
                # n_exp == 0 means ComputeShortestPath terminated immediately
                # (s_start already consistent): no path change, no event to log.
                logger.start_plan_timer()
                n_exp = self._compute_shortest_path()
                if n_exp > 0:
                    h = self._compute_goal_heuristic(maze_map, list(self._remaining_goal_set))
                    residual = h.get(robot.position, 0)
                    memory = self._memory_occupancy()
                    logger.log_replanning_event(robot.position, n_exp, int(residual), memory)

        logger.stop()
        logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
