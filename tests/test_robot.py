"""Unit tests for Robot."""
import pytest
from src.constants import Direction
from src.robot import Robot


# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------

def test_initial_state_defaults():
    r = Robot()
    assert r.x == 0
    assert r.y == 0
    assert r.heading == Direction.N
    assert r.position == (0, 0)


def test_initial_state_custom():
    r = Robot(3, 5, Direction.W)
    assert r.x == 3
    assert r.y == 5
    assert r.heading == Direction.W


# ---------------------------------------------------------------------------
# Turns
# ---------------------------------------------------------------------------

def test_turn_right_single():
    r = Robot()
    r.turn_right()
    assert r.heading == Direction.E


def test_turn_right_cycle():
    r = Robot()
    for _ in range(4):
        r.turn_right()
    assert r.heading == Direction.N


def test_turn_left_single():
    r = Robot()
    r.turn_left()
    assert r.heading == Direction.W


def test_turn_left_cycle():
    r = Robot()
    for _ in range(4):
        r.turn_left()
    assert r.heading == Direction.N


def test_turn_right_full_sequence():
    r = Robot()
    expected = [Direction.E, Direction.S, Direction.W, Direction.N]
    for exp in expected:
        r.turn_right()
        assert r.heading == exp


def test_turn_left_full_sequence():
    r = Robot()
    expected = [Direction.W, Direction.S, Direction.E, Direction.N]
    for exp in expected:
        r.turn_left()
        assert r.heading == exp


# ---------------------------------------------------------------------------
# move_forward
# ---------------------------------------------------------------------------

def test_move_forward_north():
    r = Robot(0, 0, Direction.N)
    r.move_forward()
    assert r.x == 0 and r.y == 1


def test_move_forward_east():
    r = Robot(0, 0, Direction.E)
    r.move_forward()
    assert r.x == 1 and r.y == 0


def test_move_forward_south():
    r = Robot(0, 1, Direction.S)
    r.move_forward()
    assert r.x == 0 and r.y == 0


def test_move_forward_west():
    r = Robot(1, 0, Direction.W)
    r.move_forward()
    assert r.x == 0 and r.y == 0


def test_move_forward_does_not_change_heading():
    r = Robot(0, 0, Direction.E)
    r.move_forward()
    assert r.heading == Direction.E


# ---------------------------------------------------------------------------
# Absolute wall direction helpers
# ---------------------------------------------------------------------------

def test_wall_dirs_heading_north():
    r = Robot(0, 0, Direction.N)
    assert r.wall_front_dir() == Direction.N
    assert r.wall_right_dir() == Direction.E
    assert r.wall_left_dir()  == Direction.W
    assert r.wall_back_dir()  == Direction.S


def test_wall_dirs_heading_east():
    r = Robot(0, 0, Direction.E)
    assert r.wall_front_dir() == Direction.E
    assert r.wall_right_dir() == Direction.S
    assert r.wall_left_dir()  == Direction.N
    assert r.wall_back_dir()  == Direction.W


def test_wall_dirs_heading_south():
    r = Robot(0, 0, Direction.S)
    assert r.wall_front_dir() == Direction.S
    assert r.wall_right_dir() == Direction.W
    assert r.wall_left_dir()  == Direction.E
    assert r.wall_back_dir()  == Direction.N


def test_wall_dirs_heading_west():
    r = Robot(0, 0, Direction.W)
    assert r.wall_front_dir() == Direction.W
    assert r.wall_right_dir() == Direction.N
    assert r.wall_left_dir()  == Direction.S
    assert r.wall_back_dir()  == Direction.E


# ---------------------------------------------------------------------------
# Reset
# ---------------------------------------------------------------------------

def test_reset_defaults():
    r = Robot(3, 5, Direction.S)
    r.reset()
    assert r.x == 0 and r.y == 0 and r.heading == Direction.N


def test_reset_custom():
    r = Robot(0, 0, Direction.N)
    r.move_forward()
    r.turn_right()
    r.reset(2, 4, Direction.W)
    assert r.x == 2 and r.y == 4 and r.heading == Direction.W
