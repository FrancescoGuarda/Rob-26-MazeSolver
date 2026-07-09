#!/usr/bin/env python3
"""Generate a *perfect* maze (spanning tree: no loops, no islands) in the
mms ASCII text format used by this project.

A perfect maze has exactly one path between any two cells, so it is
tree-structured with no loops and no wall islands -- i.e. a clean Level-1
maze per the project proposal. Wall-following is guaranteed to reach the
goal on such mazes.

Usage:
    python3 gen_maze.py --rows 16 --cols 16 --seed 1 -o out.txt
    python3 gen_maze.py --rows 8  --cols 8  --seed 3          # prints to stdout
"""
import argparse
import random
import sys

# each cell tracks its 4 walls; start fully walled, carve passages via DFS
DIRS = {'N': (-1, 0), 'S': (1, 0), 'E': (0, 1), 'W': (0, -1)}
OPP = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}


def generate(rows, cols, seed=None):
    rng = random.Random(seed)
    walls = [[{'N': True, 'S': True, 'E': True, 'W': True} for _ in range(cols)]
             for _ in range(rows)]
    visited = [[False] * cols for _ in range(rows)]

    # iterative randomized DFS (recursive backtracker)
    stack = [(rows - 1, 0)]          # start carving from bottom-left
    visited[rows - 1][0] = True
    while stack:
        r, c = stack[-1]
        nbrs = []
        for d, (dr, dc) in DIRS.items():
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                nbrs.append((d, nr, nc))
        if not nbrs:
            stack.pop()
            continue
        d, nr, nc = rng.choice(nbrs)
        walls[r][c][d] = False          # knock down wall both sides
        walls[nr][nc][OPP[d]] = False
        visited[nr][nc] = True
        stack.append((nr, nc))
    return walls


def render(walls, mark_start=True, mark_goal=True):
    rows, cols = len(walls), len(walls[0])
    H, W = 2 * rows + 1, 4 * cols + 1
    grid = [[' '] * W for _ in range(H)]
    # corner posts
    for i in range(rows + 1):
        for j in range(cols + 1):
            grid[2 * i][4 * j] = 'o'
    for r in range(rows):
        for c in range(cols):
            top, left = 2 * r, 4 * c
            if walls[r][c]['N']:
                for k in (1, 2, 3):
                    grid[top][left + k] = '-'
            if walls[r][c]['S']:
                for k in (1, 2, 3):
                    grid[top + 2][left + k] = '-'
            if walls[r][c]['W']:
                grid[top + 1][left] = '|'
            if walls[r][c]['E']:
                grid[top + 1][left + 4] = '|'
    # markers (purely cosmetic; mms derives the goal from the centre)
    if mark_start:
        grid[2 * (rows - 1) + 1][4 * 0 + 2] = 'S'
    if mark_goal:
        for r in (rows // 2 - 1, rows // 2):
            for c in (cols // 2 - 1, cols // 2):
                grid[2 * r + 1][4 * c + 2] = 'G'
    return '\n'.join(''.join(row) for row in grid) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rows', type=int, default=16)
    ap.add_argument('--cols', type=int, default=16)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--no-markers', action='store_true',
                    help='omit S/G markers')
    ap.add_argument('-o', '--out', help='output file (default: stdout)')
    a = ap.parse_args()
    txt = render(generate(a.rows, a.cols, a.seed),
                 mark_start=not a.no_markers, mark_goal=not a.no_markers)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(txt)
        print(f"wrote {a.cols}x{a.rows} perfect maze -> {a.out}", file=sys.stderr)
    else:
        sys.stdout.write(txt)


if __name__ == '__main__':
    main()
