"""Send a Gazebo detachable-joint event and verify the resulting state."""

from __future__ import annotations

import re
import subprocess
import time


def send_dish_event(kind: str, *, attempts: int = 3) -> None:
    if kind not in ("attach", "detach"):
        raise ValueError(f"unknown Petri dish event {kind!r}")
    expected = "attached" if kind == "attach" else "detached"
    state_topic = "/petri_dish_1/state"
    command_topic = f"/petri_dish_1/{kind}"
    last_error = "no state message received"
    for _ in range(attempts):
        listener = subprocess.Popen(
            ["ign", "topic", "-t", state_topic, "-e", "-n", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            # The CLI subscriber must be registered before the plugin emits
            # its one-time state message.
            time.sleep(0.5)
            published = subprocess.run(
                ["ign", "topic", "-t", command_topic,
                 "-m", "ignition.msgs.Empty", "-p", "unused: true"],
                capture_output=True, text=True, timeout=3, check=False,
            )
            if published.returncode:
                last_error = published.stderr.strip() or "publish failed"
                continue
            output, error = listener.communicate(timeout=3)
            if re.search(rf'data:\s*"{expected}"', output):
                return
            last_error = (output.strip() or error.strip() or
                          f"Gazebo did not report {expected}")
        except subprocess.TimeoutExpired:
            last_error = f"timed out waiting for Gazebo to report {expected}"
        finally:
            if listener.poll() is None:
                listener.kill()
                listener.communicate()
    raise RuntimeError(f"Gazebo {kind} was not confirmed: {last_error}")
