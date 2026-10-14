"""Load the three YAML files. This module never writes them."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from robot_arm.hardware.servo import TRAVEL_DEG, ServoCalibration
from robot_arm.utils.errors import ConfigError

JOINT_COUNT = 6


@dataclass(frozen=True)
class HardwareSettings:
    i2c_bus: int
    address: int
    pwm_frequency_hz: float
    require_confirmation: bool


@dataclass(frozen=True)
class RobotConfig:
    hardware: HardwareSettings
    joints: tuple[ServoCalibration, ...]
    control_hz: float

    @property
    def require_confirmation(self) -> bool:
        return self.hardware.require_confirmation

    @staticmethod
    def default_dir() -> Path:
        # src/robot_arm/config.py -> robotic-arm/configs
        return Path(__file__).resolve().parents[2] / "configs"

    @classmethod
    def load(cls, config_dir: Path | None = None) -> RobotConfig:
        root = Path(config_dir) if config_dir is not None else cls.default_dir()
        hardware_doc = _read_yaml(root / "hardware.yaml")
        joints_doc = _read_yaml(root / "joints.yaml")
        motion_doc = _read_yaml(root / "motion.yaml")
        hardware = _hardware(hardware_doc)
        travel = _number(joints_doc, "travel_deg", root / "joints.yaml")
        if float(travel) != TRAVEL_DEG:
            raise ConfigError(f"travel_deg must be {TRAVEL_DEG:.0f}, got {travel}")
        control_hz = float(_number(motion_doc, "control_hz", root / "motion.yaml"))
        if control_hz <= 0:
            raise ConfigError("control_hz must be positive")
        joints = _joints(joints_doc, motion_doc, root)
        return cls(hardware=hardware, joints=joints, control_hz=control_hz)


def _read_yaml(path: Path) -> dict:
    if not path.is_file():
        raise ConfigError(f"missing config file: {path}")
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle)
    if not isinstance(loaded, dict):
        raise ConfigError(f"{path} must be a mapping")
    return loaded


def _number(doc: dict, key: str, path: Path) -> float | int:
    if key not in doc:
        raise ConfigError(f"{path} is missing {key}")
    value = doc[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{path} field {key} must be a number")
    return value


def _address(value: object, path: Path) -> int:
    if isinstance(value, str):
        try:
            parsed = int(value, 0)
        except ValueError as exc:
            raise ConfigError(f"{path} address {value!r} is not an integer") from exc
    elif isinstance(value, int) and not isinstance(value, bool):
        parsed = value
    else:
        raise ConfigError(f"{path} address must be an integer or hex string")
    if not 0 <= parsed <= 0x7F:
        raise ConfigError(f"{path} address {parsed} is outside 0..127")
    return parsed


def _hardware(doc: dict) -> HardwareSettings:
    path = Path("hardware.yaml")
    startup = doc.get("startup", {})
    if not isinstance(startup, dict):
        raise ConfigError("hardware.yaml startup must be a mapping")
    if "require_confirmation" not in startup:
        raise ConfigError("hardware.yaml startup.require_confirmation is required")
    require = startup["require_confirmation"]
    if not isinstance(require, bool):
        raise ConfigError("startup.require_confirmation must be true or false")
    frequency = float(_number(doc, "pwm_frequency_hz", path))
    bus = int(_number(doc, "i2c_bus", path))
    if frequency <= 0:
        raise ConfigError("pwm_frequency_hz must be positive")
    return HardwareSettings(
        i2c_bus=bus,
        address=_address(doc.get("address"), path),
        pwm_frequency_hz=frequency,
        require_confirmation=require,
    )


def _joint_rows(doc: dict, path: Path) -> list[dict]:
    rows = doc.get("joints")
    if not isinstance(rows, list) or len(rows) != JOINT_COUNT:
        raise ConfigError(f"{path} must list {JOINT_COUNT} joints")
    parsed: list[dict] = []
    for row in rows:
        if not isinstance(row, dict):
            raise ConfigError(f"{path} joint entries must be mappings")
        parsed.append(row)
    return parsed


def _joints(
    joints_doc: dict,
    motion_doc: dict,
    root: Path,
) -> tuple[ServoCalibration, ...]:
    joint_rows = _joint_rows(joints_doc, root / "joints.yaml")
    motion_rows = _joint_rows(motion_doc, root / "motion.yaml")
    motion_by_index: dict[int, dict] = {}
    for row in motion_rows:
        index = int(_number(row, "index", root / "motion.yaml"))
        if index in motion_by_index:
            raise ConfigError(f"duplicate motion limit for joint {index}")
        motion_by_index[index] = row

    calibrations: list[ServoCalibration] = []
    seen_index: set[int] = set()
    seen_channel: set[int] = set()
    for row in joint_rows:
        path = root / "joints.yaml"
        index = int(_number(row, "index", path))
        channel = int(_number(row, "channel", path))
        min_us = int(_number(row, "min_us", path))
        max_us = int(_number(row, "max_us", path))
        home_us = int(_number(row, "home_us", path))
        if index in seen_index:
            raise ConfigError(f"duplicate joint index {index}")
        if channel in seen_channel:
            raise ConfigError(f"duplicate PCA9685 channel {channel}")
        if not 0 <= index < JOINT_COUNT:
            raise ConfigError(f"joint index {index} is outside 0..{JOINT_COUNT - 1}")
        if not 0 <= channel <= 15:
            raise ConfigError(f"channel {channel} is outside 0..15")
        if max_us <= min_us:
            raise ConfigError(f"joint {index} max_us must be above min_us")
        if not min_us <= home_us <= max_us:
            raise ConfigError(
                f"joint {index} home_us {home_us} is outside {min_us}..{max_us}"
            )
        if index not in motion_by_index:
            raise ConfigError(f"motion.yaml has no limits for joint {index}")
        limits = motion_by_index[index]
        velocity = float(_number(limits, "max_velocity_deg_s", root / "motion.yaml"))
        acceleration = float(
            _number(limits, "max_acceleration_deg_s2", root / "motion.yaml")
        )
        if velocity <= 0 or acceleration <= 0:
            raise ConfigError(f"joint {index} speed limits must be positive")
        seen_index.add(index)
        seen_channel.add(channel)
        calibrations.append(
            ServoCalibration(
                index=index,
                channel=channel,
                min_us=min_us,
                max_us=max_us,
                home_us=home_us,
                max_velocity_deg_s=velocity,
                max_acceleration_deg_s2=acceleration,
            )
        )
    calibrations.sort(key=lambda item: item.index)
    if seen_index != set(range(JOINT_COUNT)):
        raise ConfigError("joint indexes must be exactly 0..5")
    return tuple(calibrations)
