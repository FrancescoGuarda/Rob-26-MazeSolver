"""
Maze file parsers.

parse_maze() reads the ASCII .txt map format and returns a wall matrix
using the N=1, E=2, S=4, W=8 bitmask encoding.
"""
from src.parser.maze_parser import parse_maze

__all__ = ["parse_maze"]
