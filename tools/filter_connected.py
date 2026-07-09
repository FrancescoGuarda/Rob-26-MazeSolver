#!/usr/bin/env python3
"""Filter mazes in a directory, keeping only fully-connected ones.

A maze is fully connected when every cell is reachable from cell (0, 0)
by following open passages (i.e. no wall-enclosed islands). Mazes that
fail this check -- or fail to parse -- are deleted.

Usage:
    python3 scripts/filter_connected.py --dry-run
    python3 scripts/filter_connected.py --dir mazes/txt
    python3 scripts/filter_connected.py --dir mazes/txt --strict
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.constants import WALL_E, WALL_N, WALL_S, WALL_W
from src.parser.maze_parser import parse_maze


def check_reciprocity(wall_matrix: list[list[int]], W: int, H: int) -> list[str]:
    """Return a list of mismatch descriptions between adjacent cell walls."""
    mismatches = []
    for y in range(H):
        for x in range(W):
            mask = wall_matrix[y][x]
            if x + 1 < W:
                east_wall = bool(mask & WALL_E)
                west_wall = bool(wall_matrix[y][x + 1] & WALL_W)
                if east_wall != west_wall:
                    mismatches.append(f"({x},{y}) E vs ({x+1},{y}) W")
            if y + 1 < H:
                north_wall = bool(mask & WALL_N)
                south_wall = bool(wall_matrix[y + 1][x] & WALL_S)
                if north_wall != south_wall:
                    mismatches.append(f"({x},{y}) N vs ({x},{y+1}) S")
    return mismatches


def count_reachable(wall_matrix: list[list[int]], W: int, H: int) -> int:
    """Flood-fill from (0, 0) and return the number of reachable cells."""
    visited = [[False] * W for _ in range(H)]
    stack = [(0, 0)]
    visited[0][0] = True
    count = 1
    while stack:
        x, y = stack.pop()
        mask = wall_matrix[y][x]
        if not (mask & WALL_E) and x + 1 < W and not visited[y][x + 1]:
            visited[y][x + 1] = True
            count += 1
            stack.append((x + 1, y))
        if not (mask & WALL_W) and x - 1 >= 0 and not visited[y][x - 1]:
            visited[y][x - 1] = True
            count += 1
            stack.append((x - 1, y))
        if not (mask & WALL_N) and y + 1 < H and not visited[y + 1][x]:
            visited[y + 1][x] = True
            count += 1
            stack.append((x, y + 1))
        if not (mask & WALL_S) and y - 1 >= 0 and not visited[y - 1][x]:
            visited[y - 1][x] = True
            count += 1
            stack.append((x, y - 1))
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dir', default='mazes/txt', help='Directory of .txt maze files to filter')
    parser.add_argument('--dry-run', action='store_true', help='Report only, delete nothing')
    parser.add_argument('--strict', action='store_true', help='Also reject mazes with wall-reciprocity mismatches')
    args = parser.parse_args()

    maze_dir = Path(args.dir)
    files = sorted(maze_dir.glob('*.txt'))

    kept = 0
    rejected = 0

    for path in files:
        try:
            wall_matrix, W, H = parse_maze(str(path))
        except (ValueError, FileNotFoundError) as exc:
            print(f"REJECT {path.name} -- parse error: {exc}")
            rejected += 1
            if not args.dry_run:
                os.remove(path)
            continue

        total = W * H
        reachable = count_reachable(wall_matrix, W, H)

        reasons = []
        if reachable != total:
            reasons.append(f"{reachable}/{total} cells reachable")
        if args.strict:
            mismatches = check_reciprocity(wall_matrix, W, H)
            if mismatches:
                reasons.append(f"{len(mismatches)} wall-reciprocity mismatches")

        if reasons:
            print(f"REJECT {path.name} -- {'; '.join(reasons)}")
            rejected += 1
            if not args.dry_run:
                os.remove(path)
        else:
            print(f"KEEP   {path.name} -- {total}/{total} cells reachable")
            kept += 1

    verb = "would delete" if args.dry_run else "deleted"
    print(f"\nSummary: kept {kept}, {verb} {rejected} (out of {len(files)} total)")


if __name__ == '__main__':
    main()
