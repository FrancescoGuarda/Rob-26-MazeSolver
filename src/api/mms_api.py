"""
Mouse API for MacKorone Micromouse Simulator (MMS).
Cloned from MacKorone [mms-python](https://github.com/mackorone/mms-python) repository; file: API.py

This module provides a Python interface to interact with the MMS simulator.
It allows algorithms to communicate with the simulator via stdin/stdout,
issue commands, and handle responses.
"""

import sys

class MouseCrashedError(Exception):
    pass

def command(args, return_type=None):
    line = " ".join([str(x) for x in args]) + "\n"
    sys.stdout.write(line)
    sys.stdout.flush()
    if return_type:
        response = sys.stdin.readline().strip()
        if return_type == bool:
            return response == "true"
        return return_type(response)

def mazeWidth():
    return command(args=["mazeWidth"], return_type=int)

def mazeHeight():
    return command(args=["mazeHeight"], return_type=int)

def checkWall(wallCommand, half_steps_away=None):
    args = [wallCommand]
    if half_steps_away is not None:
        args.append(half_steps_away)
    return command(args, return_type=bool)

def wallFront(half_steps_away=None):
    return checkWall("wallFront", half_steps_away)

def wallBack(half_steps_away=None):
    return checkWall("wallBack", half_steps_away)

def wallLeft(half_steps_away=None):
    return checkWall("wallLeft", half_steps_away)

def wallRight(half_steps_away=None):
    return checkWall("wallRight", half_steps_away)

def wallFrontLeft(half_steps_away=None):
    return checkWall("wallFrontLeft", half_steps_away)

def wallFrontRight(half_steps_away=None):
    return checkWall("wallFrontRight", half_steps_away)

def wallBackLeft(half_steps_away=None):
    return checkWall("wallBackLeft", half_steps_away)

def wallBackRight(half_steps_away=None):
    return checkWall("wallBackRight", half_steps_away)

def moveForward(distance=None):
    args = ["moveForward"]
    # Don't append distance argument unless explicitly specified, for
    # backwards compatibility with older versions of the simulator
    if distance is not None:
        args.append(distance)
    response = command(args=args, return_type=str)
    if response == "crash":
        raise MouseCrashedError()

def moveForwardHalf(num_half_steps=None):
    args = ["moveForwardHalf"]
    if num_half_steps is not None:
        args.append(num_half_steps)
    response = command(args=args, return_type=str)
    if response == "crash":
        raise MouseCrashedError()

def turnRight():
    command(args=["turnRight"], return_type=str)

def turnLeft():
    command(args=["turnLeft"], return_type=str)

def turnRight90():
    turnRight()

def turnLeft90():
    turnLeft()

def turnRight45():
    command(args=["turnRight45"], return_type=str)

def turnLeft45():
    command(args=["turnLeft45"], return_type=str)

def setWall(x, y, direction):
    command(args=["setWall", x, y, direction])

def clearWall(x, y, direction):
    command(args=["clearWall", x, y, direction])

def setColor(x, y, color):
    command(args=["setColor", x, y, color])

def clearColor(x, y):
    command(args=["clearColor", x, y])

def clearAllColor():
    command(args=["clearAllColor"])

def setText(x, y, text):
    command(args=["setText", x, y, text])

def clearText(x, y):
    command(args=["clearText", x, y])

def clearAllText():
    command(args=["clearAllText"])

def wasReset():
    return command(args=["wasReset"], return_type=bool)

def ackReset():
    command(args=["ackReset"], return_type=str)


# ─── Class-based interface (implements BaseAPI) ───────────────────────────────

from src.api.base_api import BaseAPI  # noqa: E402


class MmsAPI(BaseAPI):
    """BaseAPI implementation for the MMS GUI simulator.

    Delegates every method to the corresponding module-level function in this
    module. All I/O is via stdin/stdout as required by the MMS protocol.
    """

    def maze_width(self) -> int:            return mazeWidth()  # type: ignore[return-value]
    def maze_height(self) -> int:           return mazeHeight()  # type: ignore[return-value]
    def wall_front(self) -> bool:           return wallFront()  # type: ignore[return-value]
    def wall_back(self) -> bool:            return wallBack()  # type: ignore[return-value]
    def wall_left(self) -> bool:            return wallLeft()  # type: ignore[return-value]
    def wall_right(self) -> bool:           return wallRight()  # type: ignore[return-value]
    def move_forward(self) -> None:         moveForward()   # raises MouseCrashedError on crash
    def turn_right(self) -> None:           turnRight()
    def turn_left(self) -> None:            turnLeft()
    def set_wall(self, x, y, direction):    setWall(x, y, direction)
    def clear_wall(self, x, y, direction):  clearWall(x, y, direction)
    def set_color(self, x, y, color):       setColor(x, y, color)
    def clear_color(self, x, y):            clearColor(x, y)
    def clear_all_color(self):              clearAllColor()
    def set_text(self, x, y, text):         setText(x, y, text)
    def clear_text(self, x, y):             clearText(x, y)
    def clear_all_text(self):               clearAllText()
    def was_reset(self) -> bool:            return wasReset()  # type: ignore[return-value]
    def ack_reset(self) -> None:            ackReset()
    def get_stat(self, stat: str):
        raw = command(["getStat", stat], return_type=str)
        try:
            return int(raw)  # type: ignore[arg-type]
        except (ValueError, TypeError):
            try:
                return float(raw)  # type: ignore[arg-type]
            except (ValueError, TypeError):
                return -1
