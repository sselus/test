"""Safety monitor for emergency stop state."""


class DebouncedStop:
    def __init__(self, threshold: int = 2) -> None:
        self._active = False
        self._threshold = threshold
        self._votes = 0

    def update(self, stop_signal: bool) -> bool:
        if stop_signal:
            self._votes += 1
            if self._votes >= self._threshold:
                self._active = True
        else:
            self._votes = 0
            self._active = False
        return self._active

    @property
    def active(self) -> bool:
        return self._active
