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

Default goal
------------
When neither an explicit goal list nor a random-goal count is given, the goal
is the maze's 4-cell centre *area* and the run terminates as soon as the first
of those cells is reached (self._is_default_goal).

GUI (MMS simulator; all calls are no-ops in headless mode)
-----------------------------------------------------------
* Cell text: "g-XXXr-YYY" (10-char budget); collapses to bare "inf" for a
  trivial (untouched) cell where g = rhs = ∞. Every g/rhs mutation site marks
  the cell dirty (self._dirty_text_cells) rather than calling set_text()
  immediately; _flush_gui_text() performs the actual set_text() calls once
  per GUI sync point, so the string formatting never falls inside a
  start_plan_timer()/log_replanning_event(). Independent of the colour
  redraw below.
* _gui_show_search is the single entry point for colour, called once after
  every _compute_shortest_path() cycle completes (including the first),
  recoloring the whole board from live state under a strict, first-match-wins
  priority order: goal ('G'/'g') > inconsistent ('R', Dark Red) > trivial
  (cleared) > expanded this cycle ('b', Blue) > planned path ('c', Cyan) >
  expanded in an earlier cycle ('B', Dark Blue). Because priority is
  re-evaluated against live g/rhs/queue state on every call, neither
  _previously_expanded nor _expanded_this_cycle ever needs active pruning.
* Perimeter outline drawn once at startup; new walls via set_wall as sensed.

stderr diagnostics
------------------
Wall discoveries ('[WALL] (x, y) n e s w' — one consolidated line per sensing
event) and replanning events ('[REPLAN] ...', cost_ratio/time_ms rounded to
2 decimals) are reported to stderr via _report_walls/_report_replan (never
stdout — the MMS protocol channel).
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

    def states(self):
        """All states currently queued (i.e., all inconsistent nodes)."""
        return self._valid.keys()

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

    LEGEND = [
        ("b  (Blue)",       "Expanded in the current planning cycle"),
        ("B  (Dark Blue)",  "Expanded in an earlier planning cycle"),
        ("R  (Dark Red)",   "Inconsistent (in the priority queue)"),
        ("c  (Cyan)",       "Planned path"),
        ("G  (Dark Green)", "Goal cell, not yet reached"),
        ("g  (Green)",      "Goal cell, reached"),
        ("g-XXXr-YYY",      "g-value and rhs-value of a cell"),
        ("inf",             "Trivial cell: g = rhs = infinity"),
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

        # D*-Lite state (initialised in run())
        self._g: dict[tuple[int, int], float] = defaultdict(lambda: _INF)
        self._rhs: dict[tuple[int, int], float] = defaultdict(lambda: _INF)
        self._U: _DStarQueue = _DStarQueue()
        self._km: float = 0.0
        self._s_start: tuple[int, int] = self._robot.position   # current robot pos
        self._remaining_goal_set: set[tuple[int, int]] = set()

        # GUI state consulted by _gui_show_search on every redraw: cells
        # expanded in earlier planning cycles, and cells expanded in the
        # current cycle (self._reached_goals is declared by BaseAlgorithm).
        self._previously_expanded: set[tuple[int, int]] = set()
        self._expanded_this_cycle: set[tuple[int, int]] = set()
        # Cells whose displayed text (g/rhs) is stale. _update_rhs() and
        # _compute_shortest_path() only mark cells dirty here; the actual
        # api.set_text()/_gui_cell_text() string formatting is deferred to
        # _flush_gui_text(), so it never falls inside a timed planning
        # window.
        self._dirty_text_cells: set[tuple[int, int]] = set()

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
        """Recompute rhs(u) = min over passable neighbours n of (1 + g(n)).

        Marks u's displayed text stale instead of calling api.set_text()
        directly — see self._dirty_text_cells / _flush_gui_text().
        """
        if u in self._remaining_goal_set:
            self._rhs[u] = 0.0
            self._dirty_text_cells.add(u)
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
        self._dirty_text_cells.add(u)

    def _update_vertex(self, u: tuple[int, int]) -> None:
        """Maintain U: insert/update/remove u depending on consistency."""
        if self._g[u] != self._rhs[u]:
            self._U.insert(u, self._calc_key(u))
        elif u in self._U:
            self._U.remove(u)

    @staticmethod
    def _fmt3(value: float) -> str:
        """Left-justify into a 3-char slot, or 'inf' — fits 'g-XXXr-YYY' (10 chars).

        3 characters is the correct budget: on a 16x16 maze a pathological
        shortest path can exceed 100 cells, so 2 digits is not always enough.
        """
        text = "inf" if value == _INF else str(int(value))
        return f"{text:<3}"

    def _gui_cell_text(self, x: int, y: int) -> str:
        g_val = self._g[(x, y)]
        r_val = self._rhs[(x, y)]
        # Trivial (untouched) cell: collapse to a bare "inf" instead of
        # "g-infr-inf" so the display isn't cluttered by unexplored cells.
        if g_val == _INF and r_val == _INF:
            return "inf"
        return f"g-{self._fmt3(g_val)}r-{self._fmt3(r_val)}"

    def _flush_gui_text(self, api: BaseAPI) -> None:
        """Write api.set_text() for every cell marked dirty since the last flush.

        _update_rhs()/_compute_shortest_path() only record *which* cells need
        a text refresh (self._dirty_text_cells); the actual string formatting
        (_gui_cell_text()) and API call happen here instead, so this work
        never falls inside a start_plan_timer()/log_replanning_event() window.
        Call once after every _update_rhs()/_compute_shortest_path() sequence
        that isn't itself followed by another such sequence before the next
        GUI sync point — harmless to call with an empty dirty set.
        """
        for (x, y) in self._dirty_text_cells:
            api.set_text(x, y, self._gui_cell_text(x, y))
        self._dirty_text_cells = set()

    def _reconstruct_planned_path(self) -> list[tuple[int, int]]:
        """Greedy argmin-(1+g(n)) walk from s_start toward a goal, for display.

        D*-Lite decides one step at a time from g rather than materialising a
        full path object, so the planned path is reconstructed on demand.
        Bounded by width*height and a visited set to guard against a cycle
        produced by a transiently inconsistent state.
        """
        path: list[tuple[int, int]] = []
        seen: set[tuple[int, int]] = set()
        cur: tuple[int, int] | None = self._s_start
        limit = self._width * self._height
        while cur is not None and cur not in seen and len(path) < limit:
            path.append(cur)
            seen.add(cur)
            if cur in self._remaining_goal_set:
                break
            cx, cy = cur
            best_cost = _INF
            nxt: tuple[int, int] | None = None
            for direction in Direction:
                if self._maze_map.has_wall(cx, cy, direction):
                    continue
                dx, dy = DIR_TO_DELTA[direction]
                nx, ny = cx + dx, cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                cost = 1.0 + self._g[(nx, ny)]
                if cost < best_cost:
                    best_cost = cost
                    nxt = (nx, ny)
            if nxt is None or best_cost == _INF:
                break
            cur = nxt
        return path

    def _gui_show_search(self, api: BaseAPI) -> None:
        """Single entry point: recolor the board from live D*-Lite state.

        Call once after each _compute_shortest_path() cycle completes (and
        before _previously_expanded is merged with _expanded_this_cycle, so
        the two stay disjoint for this cycle's draw). Priority order, first
        match wins: goal > inconsistent > trivial > last-expanded (this
        cycle) > planned path > previously-expanded. Because priority is
        re-evaluated against live g/rhs/queue state every call, a stale
        _previously_expanded/_expanded_this_cycle entry is harmless — rules
        1-3 always preempt rules 4/6 at draw time, so neither set needs
        active pruning.
        """
        api.clear_all_color()
        planned = set(self._reconstruct_planned_path()[1:])  # exclude current cell
        candidates = (
            self._remaining_goal_set
            | set(self._reached_goals)
            | set(self._U.states())
            | self._expanded_this_cycle
            | self._previously_expanded
            | planned
        )
        for (x, y) in candidates:
            if (x, y) in self._remaining_goal_set:
                api.set_color(x, y, 'G')
            elif (x, y) in self._reached_goals:
                api.set_color(x, y, 'g')
            elif self._g[(x, y)] != self._rhs[(x, y)]:
                api.set_color(x, y, 'R')
            elif self._g[(x, y)] == self._rhs[(x, y)] == _INF:
                api.clear_color(x, y)
            elif (x, y) in self._expanded_this_cycle:
                api.set_color(x, y, 'b')
            elif (x, y) in planned:
                api.set_color(x, y, 'c')
            elif (x, y) in self._previously_expanded:
                api.set_color(x, y, 'B')

    def _compute_shortest_path(self) -> int:
        """Run D*-Lite's ComputeShortestPath from the current s_start.

        Returns the number of non-stale state expansions (nodes_expanded metric).
        """
        nodes_expanded = 0
        self._expanded_this_cycle = set()

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
                self._expanded_this_cycle.add(u)

                self._dirty_text_cells.add(u)

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
                        self._dirty_text_cells.add(v)
                        self._update_vertex(v)

            else:
                # ---- Underconsistent: raise g to ∞ ----
                g_old = self._g[u]
                self._U.pop()
                self._g[u] = _INF
                nodes_expanded += 1
                self._expanded_this_cycle.add(u)

                self._dirty_text_cells.add(u)

                ux, uy = u
                # Update u itself (g → ∞, may become inconsistent)
                if u not in self._remaining_goal_set:
                    # rhs(u) unchanged (depends on neighbours' g, not own g)
                    self._update_vertex(u)

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

        # GUI: draw the maze perimeter before anything else (cosmetic).
        self._display_maze_outline(api)

        # ---- Initialise D*-Lite ----
        self._g = defaultdict(lambda: _INF)
        self._rhs = defaultdict(lambda: _INF)
        self._U = _DStarQueue()
        self._km = 0.0
        self._s_start = robot.position
        s_last = robot.position
        self._remaining_goal_set = set(self._goals)
        self._reached_goals = []
        self._previously_expanded = set()
        self._expanded_this_cycle = set()
        self._dirty_text_cells = set()

        # rhs = 0 for all goal cells; insert into U
        for g in self._remaining_goal_set:
            self._rhs[g] = 0.0
            self._U.insert(g, self._calc_key(g))

        # GUI: initialise cell text and goal colours
        for y in range(self._height):
            for x in range(self._width):
                s = (x, y)
                api.set_text(x, y, self._gui_cell_text(x, y))
                if s in self._remaining_goal_set:
                    api.set_color(x, y, 'G')

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
        if any(is_new for _, is_new in initial_walls):
            self._report_walls(maze_map, robot.x, robot.y)

        # Initial ComputeShortestPath.
        self._compute_shortest_path()
        self._flush_gui_text(api)
        self._gui_show_search(api)
        self._previously_expanded |= self._expanded_this_cycle

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

            # Reset handling: robot was sent back to the origin; the search
            # state (g, rhs, U, km) is tied to the old s_start, so it must be
            # rebuilt from scratch rather than repaired incrementally.
            if self._check_reset(robot, api):
                self._g = defaultdict(lambda: _INF)
                self._rhs = defaultdict(lambda: _INF)
                self._U = _DStarQueue()
                self._km = 0.0
                self._s_start = robot.position
                s_last = robot.position
                # The old search's expansion history is discarded with it.
                self._previously_expanded = set()
                for g in self._remaining_goal_set:
                    self._rhs[g] = 0.0
                    self._U.insert(g, self._calc_key(g))
                maze_map.mark_visit(robot.x, robot.y)
                self._sense_and_update(maze_map, robot, api)
                self._compute_shortest_path()
                self._flush_gui_text(api)
                self._gui_show_search(api)
                self._previously_expanded |= self._expanded_this_cycle
                continue

            # Update km: h(s_last, s_current) = Manhattan = 1 per adjacent move
            self._km += abs(self._s_start[0] - s_last[0]) + abs(self._s_start[1] - s_last[1])
            s_last = self._s_start

            # ---- Sense walls on arrival (every cell, before the goal check) ----
            # Mirrors A*: sense the cell we just entered *before* asking whether
            # it is a goal, so the robot never steps off an un-sensed cell (a
            # reached goal included) through a wall it never knew about.
            new_wall_events = self._sense_and_update(maze_map, robot, api)
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
                self._report_walls(maze_map, robot.x, robot.y)

            # ---- Check goal reached ----
            if robot.position in self._remaining_goal_set:
                self._remaining_goal_set.discard(robot.position)
                api.set_color(robot.x, robot.y, 'g')
                self._reached_goals.append(robot.position)

                if self._is_default_goal:
                    # Default goal is a single 4-cell *area*: stop at first reached.
                    self._remaining_goal_set.clear()

                if not self._remaining_goal_set:
                    break

                # Remove reached goal from D*-Lite: rhs and g → ∞
                reached = robot.position
                self._rhs[reached] = _INF
                self._g[reached] = _INF
                api.set_text(reached[0], reached[1], self._gui_cell_text(reached[0], reached[1]))
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
                self._flush_gui_text(api)
                self._gui_show_search(api)
                self._previously_expanded |= self._expanded_this_cycle
                continue

            # ---- Replanning event (only when new walls actually changed the map) ----
            # n_exp = number of expanded nodes
            # n_exp == 0 means ComputeShortestPath terminated immediately
            # (s_start already consistent): no path change, no event to log.
            if has_new:
                logger.start_plan_timer()
                n_exp = self._compute_shortest_path()
                logger.stop_plan_timer()

                self._flush_gui_text(api)
                self._gui_show_search(api)
                self._previously_expanded |= self._expanded_this_cycle
                if n_exp > 0:
                    # residual_distance is always the wall-aware BFS distance
                    # from the current position to the nearest remaining goal
                    # (see MetricsLogger schema doc, §4.1). self._g[s_start]
                    # already *is* that value once _compute_shortest_path()
                    # converges (self._s_start was set to robot.position right
                    # after the move, above, and is not reassigned before this
                    # point) — an O(1) read, versus recomputing it from scratch
                    # via a full-maze BFS. Passed through un-truncated (not
                    # int()-ed here) so MetricsLogger's own inf-safe handling
                    # (the -1 sentinel) is what governs the unreachable case,
                    # not an OverflowError from int(float('inf')) at this call
                    # site.
                    residual = self._g[self._s_start]
                    memory = self._memory_occupancy()
                    logger.log_replanning_event(robot.position, n_exp, residual, memory)
                    self._report_replan(logger.replanning_events[-1])

        self._gui_show_termination(api)
        logger.stop()
        logger.set_matrices(maze_map.export_walls(), maze_map.export_visits())
