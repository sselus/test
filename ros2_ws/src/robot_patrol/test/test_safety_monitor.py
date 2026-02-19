from robot_patrol.safety_monitor import DebouncedStop


def test_stop_requires_threshold_votes() -> None:
    stop = DebouncedStop(threshold=2)

    assert stop.update(True) is False
    assert stop.update(True) is True


def test_stop_clears_on_false_signal() -> None:
    stop = DebouncedStop(threshold=2)
    stop.update(True)
    stop.update(True)

    assert stop.active is True
    assert stop.update(False) is False
    assert stop.active is False
