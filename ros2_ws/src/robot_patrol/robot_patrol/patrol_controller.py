"""Patrol movement state and sequencing."""

from typing import List


class PatrolController:
    def __init__(self, waypoints: List[str]) -> None:
        self._waypoints = waypoints
        self._index = 0

    def current_target(self) -> str:
        return self._waypoints[self._index]

    def advance(self) -> int:
        self._index = (self._index + 1) % len(self._waypoints)
        return self._index
