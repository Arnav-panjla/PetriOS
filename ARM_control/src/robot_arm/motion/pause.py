"""A monotonic clock that can be frozen.

The arm's trajectory runner samples its pose from this clock. While the clock is
paused the elapsed time stops growing, so the arm keeps re-sending the same pose
(it holds still) and carries on from exactly that point when the clock resumes.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable


class PausableClock:
    def __init__(self, base: Callable[[], float] = time.monotonic) -> None:
        self._base = base
        self._lock = threading.Lock()
        self._offset = 0.0  # total time spent paused
        self._paused_at: float | None = None

    def __call__(self) -> float:
        with self._lock:
            now = self._base() if self._paused_at is None else self._paused_at
            return now - self._offset

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused_at is not None

    def pause(self) -> None:
        with self._lock:
            if self._paused_at is None:
                self._paused_at = self._base()

    def resume(self) -> None:
        with self._lock:
            if self._paused_at is not None:
                self._offset += self._base() - self._paused_at
                self._paused_at = None
