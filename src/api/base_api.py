"""
BaseAPI: abstract base class defining the full API contract for all backend
implementations of the MMS micromouse interface.

All algorithm code depends only on BaseAPI. Concrete implementations:
  - MmsAPI  (src/api/mms_api.py)  — MMS GUI simulator via stdin/stdout
  - SimAPI  (src/api/sim_api.py)  — headless simulator backed by a maze file
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class BaseAPI(ABC):
    """Abstract interface for the MMS micromouse API.

    All methods mirror the MMS protocol categories. Subclasses implement
    each method for their specific backend (GUI or headless).
    """

    # ------------------------------------------------------------------
    # Maze information
    # ------------------------------------------------------------------

    @abstractmethod
    def maze_width(self) -> int:
        """Return the number of columns in the maze."""

    @abstractmethod
    def maze_height(self) -> int:
        """Return the number of rows in the maze."""

    # ------------------------------------------------------------------
    # Wall sensing
    # ------------------------------------------------------------------

    @abstractmethod
    def wall_front(self) -> bool:
        """Return True if there is a wall directly in front of the robot."""

    @abstractmethod
    def wall_back(self) -> bool:
        """Return True if there is a wall directly behind the robot."""

    @abstractmethod
    def wall_left(self) -> bool:
        """Return True if there is a wall to the robot's left."""

    @abstractmethod
    def wall_right(self) -> bool:
        """Return True if there is a wall to the robot's right."""

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    @abstractmethod
    def move_forward(self) -> None:
        """Move the robot one cell forward.

        Raises:
            MouseCrashedError: if a wall blocks the move.
        """

    @abstractmethod
    def turn_right(self) -> None:
        """Turn the robot 90 degrees clockwise."""

    @abstractmethod
    def turn_left(self) -> None:
        """Turn the robot 90 degrees counter-clockwise."""

    # ------------------------------------------------------------------
    # Display — no-op in headless mode, visual in MMS GUI
    # ------------------------------------------------------------------

    @abstractmethod
    def set_wall(self, x: int, y: int, direction: str) -> None:
        """Display a wall at cell (x, y) in direction 'n'/'e'/'s'/'w'."""

    @abstractmethod
    def clear_wall(self, x: int, y: int, direction: str) -> None:
        """Remove the displayed wall at cell (x, y) in direction."""

    @abstractmethod
    def set_color(self, x: int, y: int, color: str) -> None:
        """Set the background color of cell (x, y)."""

    @abstractmethod
    def clear_color(self, x: int, y: int) -> None:
        """Clear the background color of cell (x, y)."""

    @abstractmethod
    def clear_all_color(self) -> None:
        """Clear the background color of all cells."""

    @abstractmethod
    def set_text(self, x: int, y: int, text: str) -> None:
        """Set the overlay text of cell (x, y). Maximum 10 characters."""

    @abstractmethod
    def clear_text(self, x: int, y: int) -> None:
        """Clear the overlay text of cell (x, y)."""

    @abstractmethod
    def clear_all_text(self) -> None:
        """Clear the overlay text of all cells."""

    # ------------------------------------------------------------------
    # Simulation control
    # ------------------------------------------------------------------

    @abstractmethod
    def was_reset(self) -> bool:
        """Return True if the reset button was pressed."""

    @abstractmethod
    def ack_reset(self) -> None:
        """Acknowledge the reset; move the robot back to the start."""

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @abstractmethod
    def get_stat(self, stat: str) -> int | float:
        """Return the value of the named statistic, or -1 if unavailable.

        Available stats: total-distance, total-turns, best-run-distance,
        best-run-turns, current-run-distance, current-run-turns,
        total-effective-distance, best-run-effective-distance,
        current-run-effective-distance, score.
        """
