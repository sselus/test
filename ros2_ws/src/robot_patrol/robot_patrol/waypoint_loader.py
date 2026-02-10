"""Waypoint loading helpers."""

from pathlib import Path
from typing import List

import yaml


def load_waypoints(path: str) -> List[str]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return [str(item) for item in data.get("waypoints", [])]
