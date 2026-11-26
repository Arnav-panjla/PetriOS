"""Record arm poses to a file and read them back for playback.

One session = one CSV in ARM_control/recordings/. Each Record press adds one row:
the commanded servo angle of every joint at that moment. A "hold" is stored as the
last column (hold_s) of the pose it follows: playback waits that many seconds after
reaching that pose. Playback itself is done by the caller (the GUI service).

Files without the hold_s column (8 columns) are still read; the hold is then 0.
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

JOINTS = 6
HEADER = "n,time," + ",".join(f"j{i}" for i in range(JOINTS)) + ",hold_s"
MAX_HOLD_S = 600.0


class SessionRecorder:
    def __init__(self, directory: Path) -> None:
        self.directory = Path(directory)
        self._lock = threading.Lock()
        self.file: Path | None = None  # set while a session is active
        self.points: list[dict] = []  # {"n", "time", "angles", "hold_s"} of the active session
        self._preamble = ""

    @property
    def active(self) -> bool:
        return self.file is not None

    @staticmethod
    def _row(point: dict) -> str:
        angles = ",".join(f"{a:.3f}" for a in point["angles"])
        return f"{point['n']},{point['time']},{angles},{point['hold_s']:g}\n"

    def start(self) -> Path:
        with self._lock:
            self.directory.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = self.directory / f"session_{stamp}.csv"
            n = 1
            while path.exists():  # two sessions in the same second
                n += 1
                path = self.directory / f"session_{stamp}_{n}.csv"
            self._preamble = (
                f"# PetriOS arm session, started {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                "# Commanded servo degrees per joint (no position feedback), one row per Record press.\n"
                "# hold_s = seconds to wait after reaching that pose during playback.\n"
                f"{HEADER}\n"
            )
            path.write_text(self._preamble, encoding="utf-8")
            self.file = path
            self.points = []
            return path

    def record(self, angles: list[float]) -> dict:
        if len(angles) != JOINTS:
            raise ValueError(f"expected {JOINTS} joint angles, got {len(angles)}")
        with self._lock:
            if self.file is None:
                raise ValueError("start a session first")
            point = {
                "n": len(self.points) + 1,
                "time": time.strftime("%H:%M:%S"),
                "angles": [round(float(a), 3) for a in angles],
                "hold_s": 0.0,
            }
            with self.file.open("a", encoding="utf-8") as fh:
                fh.write(self._row(point))
            self.points.append(point)
            return point

    def set_hold(self, seconds: float) -> dict:
        """Make playback wait `seconds` after the most recently recorded pose."""
        if not 0 < seconds <= MAX_HOLD_S:
            raise ValueError(f"hold time must be more than 0 and at most {MAX_HOLD_S:g} seconds")
        with self._lock:
            if self.file is None:
                raise ValueError("start a session first")
            if not self.points:
                raise ValueError("record at least one pose before adding a hold")
            self.points[-1]["hold_s"] = float(seconds)
            self.file.write_text(
                self._preamble + "".join(self._row(p) for p in self.points), encoding="utf-8"
            )
            return self.points[-1]

    def end(self) -> None:
        with self._lock:
            self.file = None

    def files(self) -> list[str]:
        """Saved session file names, newest first."""
        if not self.directory.is_dir():
            return []
        return sorted((p.name for p in self.directory.glob("session_*.csv")), reverse=True)

    def load(self, name: str) -> list[dict]:
        """Read a saved session, including optional hold_s and Gazebo dish event."""
        if name not in self.files():
            raise ValueError(f"unknown session file: {name}")
        points = []
        for line in (self.directory / name).read_text(encoding="utf-8").splitlines():
            if not line.strip() or line.startswith("#") or line.startswith("n,"):
                continue
            parts = [c.strip() for c in line.split(",")]
            if len(parts) not in (2 + JOINTS, 3 + JOINTS, 4 + JOINTS):
                raise ValueError(f"{name}: bad row {line!r}")
            hold_s = float(parts[2 + JOINTS]) if len(parts) >= 3 + JOINTS else 0.0
            if not math.isfinite(hold_s) or not 0 <= hold_s <= MAX_HOLD_S:
                raise ValueError(f"{name}: hold_s must be between 0 and {MAX_HOLD_S:g} seconds")
            event = parts[3 + JOINTS].strip() if len(parts) == 4 + JOINTS else ""
            if event not in ("", "attach", "detach"):
                raise ValueError(f"{name}: event must be attach, detach, or empty")
            points.append({"n": int(parts[0]), "time": parts[1],
                           "angles": [float(v) for v in parts[2:2 + JOINTS]],
                           "hold_s": hold_s, "event": event or None})
        return points
