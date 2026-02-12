from pathlib import Path

from robot_patrol.waypoint_loader import load_waypoints


def test_load_waypoints_reads_list(tmp_path: Path) -> None:
    config = tmp_path / "waypoints.yaml"
    config.write_text("waypoints:\n  - a\n  - b\n", encoding="utf-8")

    assert load_waypoints(str(config)) == ["a", "b"]


def test_load_waypoints_handles_missing_key(tmp_path: Path) -> None:
    config = tmp_path / "waypoints.yaml"
    config.write_text("foo: bar\n", encoding="utf-8")

    assert load_waypoints(str(config)) == []
