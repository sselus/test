"""Safety monitor for emergency stop state."""


class DebouncedStop:
    def __init__(self) -> None:
        self._active = False

    def update(self, stop_signal: bool) -> bool:
        self._active = stop_signal
        return self._active

    @property
    def active(self) -> bool:
        return self._active
