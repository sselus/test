"""Patrol movement state and sequencing."""

from dataclasses import dataclass
from typing import List


@dataclass
class PatrolMetrics:
    transitions: int = 0


class PatrolController:
    def __init__(self, waypoints: List[str]) -> None:
        self._waypoints = waypoints
        self._index = 0
        self.metrics = PatrolMetrics()

    def current_target(self) -> str:
        return self._waypoints[self._index]

    def advance(self) -> int:
        self._index = (self._index + 1) % len(self._waypoints)
        self.metrics.transitions += 1
        return self._index
