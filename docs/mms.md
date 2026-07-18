# MMS Integration Guide

This document explains how to set up and run the Micromouse algorithms using the **MMS simulator**.

## Prerequisites

Before proceeding, ensure you have completed the **Installation** section in [README.md](README.md#installation):
- ✓ Python 3.8+ installed
- ✓ Virtual environment (`.venv`) created and activated
- ✓ Dependencies installed via `pip install -r requirements.txt`
- ✓ Project structure initialized (Python modules in `src/algorithms/`, maze files in `mazes/`)

## Step 1: Download and Install the Simulator

1. Download the [MMS simulator binary](https://github.com/mackorone/mms/releases) for your operating system.
2. Extract the archive into a folder (e.g., `~/mms/` or `./app/` in the project root).

### macOS-Specific Step

On macOS, you may encounter a quarantine error:
```
"mms.app" is damaged and can't be opened. You should move it to the Trash.
```

To fix this, remove the quarantine attribute:
```bash
cd ~/mms  # or wherever you extracted the simulator
xattr -d com.apple.quarantine mms.app
```

## Step 2: Launch the Simulator

Run the MMS simulator:
```bash
open mms.app              # macOS
```

## Step 3: Configure an Algorithm

1. In the MMS window, click the **`+`** button to add a new algorithm.
   ![MMS gui](res/mms_gui.png)

2. Fill in the **New Mouse Algorithm** dialog:
   - **Name**: Algorithm identifier (e.g., `flood_fill`, `wall_following`, `astar`)
   - **Directory**: Full path to the repository root (e.g., `/path/to/Rob-26-MazeSolver`)
   - **Build command**: Leave blank (Python is interpreted, not compiled)
   - **Run command**: See the section below

### Run Command Format

The run command instructs MMS how to invoke `run.py`, the repository's MMS GUI entry point. Use the template:

```bash
.venv/bin/python run.py --algo [astar|dstar_lite] [--goal X Y ...] [--n-goals N] [--seed S] [--auto-goals MAZE] [-k N] [--heuristic min_path|manhattan] [--output-dir DIR] [--no-log]
```

- `--algo`: the algorithm to run — `astar` or `dstar_lite` (required)
- `--goal X Y`: a goal cell; repeat the flag for multiple goals (e.g. `--goal 3 3 --goal 0 3`)
- `--n-goals N`: generate `N` random goal cells instead of an explicit `--goal` list
- `--seed S`: random seed used together with `--n-goals` (requires `--n-goals`)
- `--auto-goals MAZE`: place goals automatically by detour index in `MAZE` (see below)
- `-k N`, `--n-auto-goals N`: how many goals `--auto-goals` should place, default `4`
- `--goal`, `--n-goals` and `--auto-goals` are mutually exclusive; if none is given, the algorithm defaults to the maze's 4-cell centre area
- `--heuristic min_path|manhattan`: planning heuristic, default `min_path` (wall-aware shortest-known-distance). `manhattan` uses straight-line distance instead, ignoring wall knowledge. Only affects `--algo astar`; `dstar_lite` always uses its own Manhattan-to-current-position heuristic regardless of this flag
- `--output-dir DIR`: directory for the exported JSON log, default `results/logs/`
- `--no-log`: skip writing the JSON metrics log entirely (stderr diagnostics are unaffected); useful to avoid filling `results/logs/` during debugging/testing runs

> **Using conda instead of `.venv`?** The `.venv/bin/python` path above only applies to a `venv`-created environment. If you use conda, use the full path to your conda environment's Python interpreter instead:
>
> ```bash
> /path/to/miniconda/base/envs/[env_name]/bin/python run.py --algo [astar|dstar_lite]
> ```
>
> To find your exact interpreter path, activate your conda environment and run `which python`.

On completion, `run.py` writes a JSON metrics log to `results/logs/` (the same schema produced by headless `SimAPI` runs via `experiments/run_batch.py`), so GUI and batch runs are directly comparable — unless `--no-log` was passed, in which case no file is written and `--output-dir` is ignored.

### Automatic goal placement (`--auto-goals`)

`--auto-goals` removes the manual copy-paste step from the goal workflow. Without it, running a placed-goal scenario in the GUI means running `tools/place_goals.py`, reading the `--goal X Y ...` line it prints, and pasting those coordinates into the "Run command" field — again for every maze and every goal count. With it, you name the maze once and the placement happens inside `run.py`:

```bash
# Equivalent to pasting the output of: python3 tools/place_goals.py 2015japan -k 4
.venv/bin/python run.py --algo astar --auto-goals 2015japan -k 4
```

**The goals are identical to the tool's.** Both paths call the same `src.goal_placement.scenario_goals()`, which is deterministic — no randomness, fixed `(y, x)` tie-break — so for a given maze, start cell and `k` the result cannot differ. `--auto-goals` is a convenience over the same placement, not a second implementation of it, and `tools/place_goals.py` remains the way to *inspect* a placement (it also prints the detour score behind each goal, which `run.py` discards). The `k = 1` special case carries over unchanged: one goal at the maze's centre cell, deliberately not the first goal of a `k ≥ 2` placement — see `tools/README.md` for the rule and its rationale.

**The start cell** is the robot's initial position (`(0, 0)`, the simulator convention), matching the default that `tools/place_goals.py` documents for `--start`.

**Naming the maze is required, and is the one thing you must keep correct.** MMS's stdin/stdout protocol reports only the maze's width and height — it never tells the algorithm which maze file the GUI has loaded. Placement, however, needs the true wall layout (it runs BFS over the complete maze), so `run.py` has to parse the file itself and therefore has to be told which one. `MAZE` is resolved like the tool's argument — a bare name, `.txt` optional, looked up in `mazes/txt/` — and additionally accepts a path (absolute, or relative to the repository root) for mazes kept elsewhere, such as `mazes/maze_test.txt`:

```bash
--auto-goals 2015japan          # -> mazes/txt/2015japan.txt
--auto-goals 2015japan.txt      # -> mazes/txt/2015japan.txt
--auto-goals mazes/maze_test.txt  # -> path relative to the repo root
```

As a guard against a stale name left behind after switching mazes in the GUI, `run.py` compares the parsed file's dimensions with the simulator's and aborts on a mismatch:

```text
error: --auto-goals: '/…/mazes/txt/2015japan.txt' is 16x16, but the simulator
reports 8x8 — the maze loaded in the GUI is not the one named here
```

Note the limit of this check: it only catches mismatches **of different size**. Two distinct 16×16 mazes are indistinguishable over the protocol, so if you load a different maze of the same dimensions the run will proceed with goals placed for the wrong layout. Update the flag when you change maze. Any other failure — missing or malformed file, an unreachable `k = 1` centre cell, a maze with no reachable candidate cells — also aborts with an `error: --auto-goals: …` message on stderr rather than falling back silently.

**Example configuration:**

   ![New Mouse Algorithm dialog](res/new_algorithm_dialog.png)

## Step 4: Select a Maze

1. Click the **Maze** button in the MMS window.
2. Navigate to `mazes` and select a maze file (`.txt` format).
   ![maze selection dialog](res/maze_selection.png)

## Step 5: Run the Simulation

1. Click the **Run** button to start the simulation.
2. Watch the robot explore the maze in real-time. A small **legend window** (Tkinter, with a color swatch next to each entry) also opens in its own process, mapping the on-screen cell colors and text (e.g. `f-XXXh-YYY`, `g-XXXr-YYY`) to their meaning for the selected algorithm. Starting a new run automatically closes a legend window left open from a previous run, so they don't stack up.
3. The **Stats** tab shows exploration metrics (distance, turns, effective distance, score).
4. Wall discoveries and replanning events are logged to **stderr** as they happen — one consolidated `[WALL] (x, y) n e s w` line per sensing event (`_` marks an absent wall), and `[REPLAN] ...` lines with `cost_ratio`/`time_ms` rounded to 2 decimals; stdout is reserved for the MMS protocol.
5. The simulation ends once the algorithm's goal condition is satisfied (or, for the default centre-area goal, as soon as the first of its 4 cells is reached). At that point the GUI clears all non-goal decoration and leaves only goal cells marked: reached goals in green (`g`), labelled with their 1-based reach order for a true multi-goal run; unreached default-centre-area cells stay dark green (`G`).

This runs all algorithms on all mazes and logs metrics to `results/logs/` without requiring MMS to be running.

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Module not found" error | Ensure you are running from the repo root and the virtual environment is activated |
| Algorithm hangs or crashes | Check `stderr` output; run locally with `scripts/batch_run.py` to get detailed error messages |
| Maze file not found | Verify the path is correct and the file has a `.txt` extension |
| On macOS: "mms.app is damaged" | Run `xattr -d com.apple.quarantine mms.app` in the simulator directory |
