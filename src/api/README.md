# Mouse API

This directory contains the API layer that decouples the exploration algorithms from the underlying execution backend. All algorithms are coded against a common interface so that the same algorithm code runs without modification under both the MMS GUI simulator and the headless testing mode.

## Architecture

| Module | Status | Role |
|--------|--------|------|
| [`base_api.py`](#base_apipy--abstract-interface) | ○ planned | Abstract base class declaring the full API contract |
| [`mms_api.py`](#mms_apipy--mms-gui-simulator) | ✓ implemented | Concrete implementation for the MMS GUI simulator via `stdin`/`stdout` |
| [`sim_api.py`](#sim_apipy--headless-simulator) | ○ planned | Concrete implementation for headless batch testing backed by a loaded maze file |

Each concrete implementation inherits from `base_api.py` and overrides every abstract method. Algorithm code imports only `BaseAPI` and receives whichever concrete instance is appropriate at runtime.

---

## `base_api.py` — Abstract Interface

> **Status:** TODO: planned

Will define the abstract base class `BaseAPI` using Python's `abc.ABC`, declaring all methods listed in the [API Reference](#api-reference) as `@abstractmethod`. No logic; no I/O. Its sole purpose is to enforce the interface contract so that any new backend (physical robot, alternative simulator, replay engine) can be added without touching algorithm code.

---

## `mms_api.py` — MMS GUI Simulator

> **Status:** implemented — adapted from [`mackorone/mms-python`](https://github.com/mackorone/mms-python) (`API.py`)

Implements `BaseAPI` as a collection of module-level functions that communicate with the MMS simulator process via `stdin`/`stdout`. Each method serialises its command to `stdout` and, where a response is expected, reads and parses one line from `stdin`. Debug output must be directed to `stderr` to avoid corrupting the protocol stream.

**Exception:** `MouseCrashedError` — raised by `moveForward` and `moveForwardHalf` when the simulator responds with `crash` (movement blocked by a wall or invalid distance).

---

## `sim_api.py` — Headless Simulator

> **Status:** TODO: planned

Will implement `BaseAPI` as a class backed by a maze file loaded via TODO: *`maze_loader.py`*. Sensor queries will be answered by consulting the ground-truth wall matrix; movement will update an internal robot state. `MouseCrashedError` will be raised on the same conditions as `mms_api.py`. This enables fully automated `pytest` test runs and batch evaluations without launching the MMS GUI.

---

## API Reference

Methods are grouped by category. The **Python signature** column reflects the actual function signatures in `mms_api.py`. The `N` parameter in wall-sensing methods checks for a wall `N` half-steps away (default `1`).

### Maze Information

| Method | Python signature | Returns | Notes |
|--------|-----------------|---------|-------|
| `mazeWidth` | `mazeWidth()` | `int` | Width (columns) of the maze |
| `mazeHeight` | `mazeHeight()` | `int` | Height (rows) of the maze |

### Wall Sensing

| Method | Python signature | Returns | Notes |
|--------|-----------------|---------|-------|
| `wallFront` | `wallFront(half_steps_away=None)` | `bool` | Wall directly ahead |
| `wallBack` | `wallBack(half_steps_away=None)` | `bool` | Wall directly behind |
| `wallLeft` | `wallLeft(half_steps_away=None)` | `bool` | Wall to the left |
| `wallRight` | `wallRight(half_steps_away=None)` | `bool` | Wall to the right |
| `wallFrontLeft` | `wallFrontLeft(half_steps_away=None)` | `bool` | Diagonal — front-left |
| `wallFrontRight` | `wallFrontRight(half_steps_away=None)` | `bool` | Diagonal — front-right |
| `wallBackLeft` | `wallBackLeft(half_steps_away=None)` | `bool` | Diagonal — back-left |
| `wallBackRight` | `wallBackRight(half_steps_away=None)` | `bool` | Diagonal — back-right |

### Movement

| Method | Python signature | Returns | Crash condition |
|--------|-----------------|---------|-----------------|
| `moveForward` | `moveForward(distance=None)` | `None` | Raises `MouseCrashedError` if blocked or `distance < 1` |
| `moveForwardHalf` | `moveForwardHalf(num_half_steps=None)` | `None` | Raises `MouseCrashedError` if blocked or `num_half_steps < 1` |
| `turnRight` | `turnRight()` | `None` | — |
| `turnLeft` | `turnLeft()` | `None` | — |
| `turnRight90` | `turnRight90()` | `None` | Alias for `turnRight()` |
| `turnLeft90` | `turnLeft90()` | `None` | Alias for `turnLeft()` |
| `turnRight45` | `turnRight45()` | `None` | — |
| `turnLeft45` | `turnLeft45()` | `None` | — |

### Display (MMS GUI only)

These commands have visual effect only in the MMS GUI; `sim_api.py` will accept them as no-ops.

| Method | Python signature | Effect |
|--------|-----------------|--------|
| `setWall` | `setWall(x, y, direction)` | Display a wall at cell `(x, y)` in direction `n/e/s/w` |
| `clearWall` | `clearWall(x, y, direction)` | Remove displayed wall at cell `(x, y)` |
| `setColor` | `setColor(x, y, color)` | Set cell background color (see [color codes](https://github.com/mackorone/mms#cell-color)) |
| `clearColor` | `clearColor(x, y)` | Clear cell color |
| `clearAllColor` | `clearAllColor()` | Clear all cell colors |
| `setText` | `setText(x, y, text)` | Set cell overlay text (max 10 chars; see [valid chars](https://github.com/mackorone/mms#cell-text)) |
| `clearText` | `clearText(x, y)` | Clear cell text |
| `clearAllText` | `clearAllText()` | Clear all cell text |

### Simulation Control

| Method | Python signature | Returns | Notes |
|--------|-----------------|---------|-------|
| `wasReset` | `wasReset()` | `bool` | `True` if the reset button was pressed |
| `ackReset` | `ackReset()` | `None` | Moves robot back to start; call after handling reset |

### Statistics

> **Note:** `getStat` is part of the official MMS API but is not yet implemented in `mms_api.py`.

| Stat key | Type | Description |
|----------|------|-------------|
| `total-distance` | `int` | Total cells traversed across all runs |
| `total-turns` | `int` | Total turns taken across all runs |
| `best-run-distance` | `int` | Distance of the best start-to-goal run |
| `best-run-turns` | `int` | Turns of the best start-to-goal run |
| `current-run-distance` | `int` | Distance of the current run |
| `current-run-turns` | `int` | Turns of the current run |
| `total-effective-distance` | `float` | Effective distance (penalises non-straight segments) |
| `best-run-effective-distance` | `float` | Effective distance of the best run |
| `current-run-effective-distance` | `float` | Effective distance of the current run |
| `score` | `float` | Final score: `best_turns + best_eff_dist + 0.1 × (total_turns + total_eff_dist)` |

### Summary

#### `mazeWidth`
* **Args:** None
* **Action:** None
* **Response:** The height of the maze

#### `mazeHeight`
* **Args:** None
* **Action:** None
* **Response:** The width of the maze

#### `wallFront [N]`
* **Args:**
  * `N` - (optional) Check for a wall this many half-steps away, default `1`
* **Action:** None
* **Response:** `true` if there is a wall, else `false`

#### `wallRight [N]`
* **Args:** None
  * `N` - (optional) Check for a wall this many half-steps away, default `1`
* **Action:** None
* **Response:** `true` if there is a wall, else `false`

#### `wallLeft [N]`
* **Args:** None
  * `N` - (optional) Check for a wall this many half-steps away, default `1`
* **Action:** None
* **Response:** `true` if there is a wall, else `false`

#### `wallBack [N]`
* **Args:** None
  * `N` - (optional) Check for a wall this many half-steps away, default `1`
* **Action:** None
* **Response:** `true` if there is a wall, else `false`

#### `moveForward [N]`
* **Args:**
  * `N` - (optional) The number of full steps to move forward, default `1`
* **Action:** Move the robot forward the specified number of full-steps
* **Response:**
  * `crash` if `N < 1` or the mouse cannot complete the movement
  * else `ack` once the movement completes

#### `moveForwardHalf [N]`
* **Args:**
  * `N` - (optional) The number of half steps to move forward, default `1`
* **Action:** Move the robot forward the specified number of half-steps
* **Response:**
  * `crash` if `N < 1` or the mouse cannot complete the movement
  * else `ack` once the movement completes

#### `turnRight` or `turnRight90`
* **Args:** None
* **Action:** Turn the robot ninty degrees to the right
* **Response:** `ack` once the movement completes

#### `turnLeft` or `turnLeft90`
* **Args:** None
* **Action:** Turn the robot ninty degrees to the left
* **Response:** `ack` once the movement completes

#### `turnRight45`
* **Args:** None
* **Action:** Turn the robot forty-five degrees to the right
* **Response:** `ack` once the movement completes

#### `turnLeft45`
* **Args:** None
* **Action:** Turn the robot forty-five degrees to the left
* **Response:** `ack` once the movement completes

#### `setWall X Y D`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
  * `D` - The direction of the wall: `n`, `e`, `s`, or `w`
* **Action:** Display a wall at the given position
* **Response:** None

#### `clearWall X Y D`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
  * `D` - The direction of the wall: `n`, `e`, `s`, or `w`
* **Action:** Clear the wall at the given position
* **Response:** None

#### `setColor X Y C`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
  * `C` - The character of the desired [color](https://github.com/mackorone/mms#cell-color)
* **Action:** Set the color of the cell at the given position
* **Response:** None

#### `clearColor X Y`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
* **Action:** Clear the color of the cell at the given position
* **Response:** None

#### `clearAllColor`
* **Args:** None
* **Action:** Clear the color of all cells
* **Response:** None

#### `setText X Y TEXT`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
  * `TEXT` - The desired [text](https://github.com/mackorone/mms#cell-text), max length 10
* **Action:** Set the text of the cell at the given position
* **Response:** None

#### `clearText X Y`
* **Args:**
  * `X` - The X coordinate of the cell
  * `Y` - The Y coordinate of the cell
* **Action:** Clear the text of the cell at the given position
* **Response:** None

#### `clearAllText`
* **Args:** None
* **Action:** Clear the text of all cells
* **Response:** None


#### `wasReset`
* **Args:** None
* **Action:** None
* **Response:** `true` if the reset button was pressed, else `false`

#### `ackReset`
* **Args:** None
* **Action:** Allow the mouse to be moved back to the start of the maze
* **Response:** `ack` once the movement completes

#### `getStat`
* **Args:**
  * `stat`: A string representing the stat to query. Available stats are:
    * `total-distance (int)`
    * `total-turns (int)`
    * `best-run-distance (int)`
    * `best-run-turns (int)`
    * `current-run-distance (int)`
    * `current-run-turns (int)`
    * `total-effective-distance (float)`
    * `best-run-effective-distance (float)`
    * `current-run-effective-distance (float)`
    * `score (float)`
* **Action:** None
* **Response:** The value of the stat, or `-1` if no value exists yet. The value will either be a float or integer, according to the types listed above.

