"""
Rob-26-MazeSolver: maze-solving algorithms for the MMS micromouse simulator.

Packages:
  src.constants   — Direction enum, wall bitmasks, lookup tables, color codes
  src.maze_map    — MazeMap (2-D wall + visit matrices)
  src.robot       — Robot (position and heading tracker)
  src.api         — BaseAPI, MmsAPI (GUI), SimAPI (headless)
  src.parser      — ASCII maze file parser
  src.metrics     — MetricsLogger for per-run statistics
  src.algorithms  — WallFollower, FloodFill, AStarExplorer (Phase 3)
"""

__version__ = "0.1.0"