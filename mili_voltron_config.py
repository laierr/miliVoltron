"""Small TOML configuration loader for miliVoltron.

Command-line values override these settings.  The module deliberately avoids a
configuration framework; Python 3.11's stdlib ``tomllib`` is enough.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - only relevant to old Python
    tomllib = None  # type: ignore[assignment]


DEFAULT_CONFIG: dict[str, Any] = {
    "serial": {
        "baud": 115200,
        "auto_detect": True,
    },
    "dashboard": {
        "refresh_hz": 5.0,
        "color": True,
        "recent_changes_limit": 8,
        "stale_after_s": 5.0,
    },
    "recent_changes": {
        "vehicle_state": True,
        "battery_mode": True,
        "error_code": True,
        "battery_latch": True,
        "bms_link": True,
        "wheel_moving": False,
        "left_indicator": False,
        "right_indicator": False,
        "flags": {
            # Known routine activity is disabled.  Unknown bits remain useful.
            "bit_0": False,
            "bit_1": True,
            "bit_2": True,
            "bit_3": False,
            "bit_4": False,
            "bit_5": True,
            "bit_6": True,
            "bit_7": True,
        },
    },
    "inquisitor": {
        "poll_interval_s": 1.0,
        "response_timeout_s": 0.35,
        "inter_request_gap_s": 0.01,
        "retries": 1,
        "startup_identity": True,
    },
    "battery": {
        "rest_current_max_a": 1.5,
        "rest_motor_power_max_w": 20.0,
        "baseline_samples": 2,
        "baseline_window_samples": 3,
        "resistance_min_current_a": 2.0,
    },
    "logging": {
        "timestamp": True,
        "timestamp_format": "%Y%m%d-%H%M%S",
        "all_logs_directory": "comm-logs",
        "all_logs_prefix": "inq",
        "battery_log_directory": "battery-log",
        "battery_log_prefix": "battery",
        "raw_log": "",
        "decoded_log": "",
        "combined_log": "",
        "jsonl": "",
        "battery_log": "",
    },
}


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: Path | None) -> tuple[dict[str, Any], Path | None]:
    """Load configuration, returning merged values and the file actually used."""

    if path is None:
        candidate = Path(__file__).with_name("mili-voltron.toml")
        if not candidate.exists():
            return deepcopy(DEFAULT_CONFIG), None
        path = candidate

    if not path.exists():
        raise FileNotFoundError(f"configuration file not found: {path}")
    if tomllib is None:
        raise RuntimeError("TOML configuration requires Python 3.11 or newer")

    with path.open("rb") as handle:
        loaded = tomllib.load(handle)
    if not isinstance(loaded, dict):
        raise ValueError("top-level TOML value must be a table")
    return _merge(DEFAULT_CONFIG, loaded), path
