"""Stateful battery-health analytics and CSV logging for Voltron.

The analyzer consumes already-decoded UART events. It deliberately ignores most
vehicle telemetry and focuses on battery behaviour under load and while charging.
"""

from __future__ import annotations

import csv
import math
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from statistics import fmean
from typing import TextIO

from mili_voltron_defs import (
    BATTERY_BASELINE_MIN_SAMPLES,
    BATTERY_BASELINE_WINDOW_SAMPLES,
    BATTERY_RESISTANCE_MIN_CURRENT_A,
    BATTERY_REST_CURRENT_THRESHOLD_A,
    BATTERY_REST_MOTOR_POWER_THRESHOLD_W,
)


BATTERY_LOG_FIELDS = [
    "timestamp_utc",
    "elapsed_s",
    "bms_serial",
    "bms_firmware",
    "mode",
    "bms_voltage_v",
    "bms_current_a",
    "bms_power_w",
    "ecu_motor_power_latest_w",
    "ecu_motor_power_average_w",
    "ecu_motor_power_peak_w",
    "battery_soc_percent",
    "health_percent",
    *[f"cell_{index}_mv" for index in range(1, 11)],
    "cell_min_mv",
    "cell_min_index",
    "cell_max_mv",
    "cell_max_index",
    "cell_delta_mv",
    *[f"temp_{index}_c" for index in range(1, 5)],
    "temp_min_c",
    "temp_max_c",
    "reference_voltage_v",
    "pack_sag_v",
    "charge_rise_v",
    "resistance_estimate_mohm",
    "worst_cell_sag_mv",
    "worst_cell_sag_index",
    "highest_cell_rise_mv",
    "highest_cell_rise_index",
]


@dataclass(slots=True)
class CompleteBatterySample:
    elapsed_s: float
    voltage_v: float
    current_a: float
    power_w: float
    mode: str
    cells_mv: list[int]
    temperatures_c: list[int]
    motor_power_latest_w: float | None
    motor_power_average_w: float | None
    motor_power_peak_w: float | None
    motor_power_abs_peak_w: float | None
    soc_percent: int | None
    health_percent: int | None
    bms_serial: str | None
    bms_firmware: str | None


class BatteryAnalyzer:
    """Collect coherent BMS cycles, calculate sag, and optionally write CSV.

    A complete sample is emitted after STATUS, CELL_VOLTAGES and the BMS
    temperature block have all arrived for the same cycle. Fast ECU motor-power
    heartbeats are aggregated between complete BMS samples.
    """

    def __init__(
        self,
        csv_path: Path | None = None,
        *,
        rest_current_max_a: float = BATTERY_REST_CURRENT_THRESHOLD_A,
        rest_motor_power_max_w: float = BATTERY_REST_MOTOR_POWER_THRESHOLD_W,
        baseline_min_samples: int = BATTERY_BASELINE_MIN_SAMPLES,
        baseline_window_samples: int = BATTERY_BASELINE_WINDOW_SAMPLES,
        resistance_min_current_a: float = BATTERY_RESISTANCE_MIN_CURRENT_A,
    ) -> None:
        if baseline_min_samples < 1:
            raise ValueError("baseline_min_samples must be at least 1")
        if baseline_window_samples < baseline_min_samples:
            raise ValueError("baseline_window_samples must be >= baseline_min_samples")
        self.rest_current_max_a = float(rest_current_max_a)
        self.rest_motor_power_max_w = float(rest_motor_power_max_w)
        self.baseline_min_samples = int(baseline_min_samples)
        self.baseline_window_samples = int(baseline_window_samples)
        self.resistance_min_current_a = float(resistance_min_current_a)
        self.csv_path = csv_path
        self._csv_handle: TextIO | None = None
        self._csv_writer: csv.DictWriter[str] | None = None
        if csv_path is not None:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_handle = csv_path.open("x", encoding="utf-8", newline="")
            self._csv_writer = csv.DictWriter(self._csv_handle, fieldnames=BATTERY_LOG_FIELDS)
            self._csv_writer.writeheader()
            self._csv_handle.flush()

        self.vehicle_state_raw: int | None = None
        self.soc_percent: int | None = None
        self.health_percent: int | None = None
        self.bms_serial: str | None = None
        self.bms_firmware: str | None = None

        self._status_generation = 0
        self._emitted_generation = 0
        self._pending_status: dict[str, float] | None = None
        self._pending_cells: list[int] | None = None
        self._pending_temperatures: list[int] | None = None
        self._pending_elapsed_s: float | None = None

        self._motor_values: list[float] = []
        self._motor_latest: float | None = None

        self._rest_samples: deque[CompleteBatterySample] = deque(
            maxlen=self.baseline_window_samples
        )
        self.reference_voltage_v: float | None = None
        self.reference_cells_mv: list[float] | None = None

        self.latest_sample: CompleteBatterySample | None = None
        self.latest_analytics: dict[str, object] = {}
        self._last_mode: str | None = None
        self._active_direction: str | None = None
        self.last_status_monotonic: float | None = None
        self.last_complete_monotonic: float | None = None

        self.initial_temperature_c: float | None = None
        self.peaks: dict[str, float | int | None] = {
            "discharge_current_a": 0.0,
            "charge_current_a": 0.0,
            "discharge_power_w": 0.0,
            "charge_power_w": 0.0,
            "motor_power_w": 0.0,
            "pack_sag_v": 0.0,
            "charge_rise_v": 0.0,
            "discharge_resistance_mohm": 0.0,
            "cell_delta_mv": 0,
            "cell_sag_mv": 0.0,
            "cell_rise_mv": 0.0,
            "temperature_c": None,
            "temperature_rise_c": 0.0,
        }

    def close(self) -> None:
        if self._csv_handle is not None:
            self._csv_handle.flush()
            self._csv_handle.close()
            self._csv_handle = None

    def on_bms_link_lost(self) -> None:
        """Discard identity and references that may belong to a removed battery."""

        self.bms_serial = None
        self.bms_firmware = None
        self.health_percent = None

        self._pending_status = None
        self._pending_cells = None
        self._pending_temperatures = None
        self._pending_elapsed_s = None
        self._emitted_generation = self._status_generation

        self._rest_samples.clear()
        self.reference_voltage_v = None
        self.reference_cells_mv = None
        self._active_direction = None
        self.latest_sample = None
        self.latest_analytics = {}
        self._motor_values.clear()

    def _derive_mode(self, current_a: float, motor_abs_peak_w: float | None) -> str:
        # Charging is authoritative either from signed BMS current or ECU state 20.
        if current_a < -self.rest_current_max_a or self.vehicle_state_raw == 20:
            return "CHARGING"

        motor_active = (
            motor_abs_peak_w is not None
            and motor_abs_peak_w > self.rest_motor_power_max_w
        )
        if current_a > self.rest_current_max_a or motor_active:
            return "DISCHARGING"
        return "REST"

    def update(self, decoded: dict[str, object], elapsed_s: float, source_name: str) -> None:
        kind = str(decoded.get("kind", "generic"))

        if kind == "heartbeat":
            state = decoded.get("vehicle_state_raw")
            if isinstance(state, int):
                self.vehicle_state_raw = state
            soc = decoded.get("battery_soc_percent")
            if isinstance(soc, int):
                self.soc_percent = soc
            motor_power = decoded.get("motor_power_w")
            if isinstance(motor_power, (int, float)):
                value = float(motor_power)
                self._motor_latest = value
                self._motor_values.append(value)
                self.peaks["motor_power_w"] = max(float(self.peaks["motor_power_w"] or 0.0), value)
            return

        if kind == "serial" and source_name in {"BMS", "BMS2"}:
            serial = decoded.get("serial")
            if isinstance(serial, str) and serial:
                self.bms_serial = serial
            firmware = decoded.get("firmware_version")
            if isinstance(firmware, str) and firmware:
                self.bms_firmware = firmware
            else:
                raw_firmware = decoded.get("firmware_raw_u16")
                if isinstance(raw_firmware, int):
                    self.bms_firmware = f"0x{raw_firmware:04X}"
            return

        if kind == "firmware" and source_name in {"BMS", "BMS2"}:
            version = decoded.get("version")
            raw_firmware = decoded.get("raw_u16")
            if isinstance(version, str) and version:
                self.bms_firmware = version
            elif isinstance(raw_firmware, int):
                self.bms_firmware = f"0x{raw_firmware:04X}"
            return

        if kind == "capacity_health":
            health = decoded.get("percent")
            if isinstance(health, int):
                self.health_percent = health
            return

        if kind == "bms_status":
            voltage = decoded.get("voltage_v")
            current = decoded.get("current_a")
            if not isinstance(voltage, (int, float)) or not isinstance(current, (int, float)):
                return
            voltage_f = float(voltage)
            current_f = float(current)
            self._status_generation += 1
            self._pending_status = {
                "voltage_v": voltage_f,
                "current_a": current_f,
                "power_w": voltage_f * current_f,
            }
            self._pending_cells = None
            self._pending_temperatures = None
            self._pending_elapsed_s = elapsed_s
            self.last_status_monotonic = time.monotonic()
            return

        if kind == "cell_voltages" and self._pending_status is not None:
            cells = decoded.get("cells_mv")
            if isinstance(cells, list) and all(isinstance(value, int) for value in cells):
                self._pending_cells = list(cells)
                self._try_complete()
            return

        if kind == "pcb_temperature_block" and self._pending_status is not None:
            temperatures = decoded.get("temperatures_c")
            if isinstance(temperatures, list) and all(isinstance(value, int) for value in temperatures):
                self._pending_temperatures = list(temperatures)
                self._try_complete()

    def _try_complete(self) -> None:
        if (
            self._pending_status is None
            or self._pending_cells is None
            or self._pending_temperatures is None
            or self._pending_elapsed_s is None
            or self._status_generation == self._emitted_generation
        ):
            return

        motor_average = fmean(self._motor_values) if self._motor_values else self._motor_latest
        motor_peak = max(self._motor_values) if self._motor_values else self._motor_latest
        motor_abs_peak = (
            max(abs(value) for value in self._motor_values)
            if self._motor_values
            else abs(self._motor_latest) if self._motor_latest is not None else None
        )
        current_a = self._pending_status["current_a"]
        sample = CompleteBatterySample(
            elapsed_s=self._pending_elapsed_s,
            voltage_v=self._pending_status["voltage_v"],
            current_a=current_a,
            power_w=self._pending_status["power_w"],
            mode=self._derive_mode(current_a, motor_abs_peak),
            cells_mv=self._pending_cells,
            temperatures_c=self._pending_temperatures,
            motor_power_latest_w=self._motor_latest,
            motor_power_average_w=motor_average,
            motor_power_peak_w=motor_peak,
            motor_power_abs_peak_w=motor_abs_peak,
            soc_percent=self.soc_percent,
            health_percent=self.health_percent,
            bms_serial=self.bms_serial,
            bms_firmware=self.bms_firmware,
        )

        self._emitted_generation = self._status_generation
        self._motor_values.clear()
        self.latest_sample = sample
        self.last_complete_monotonic = time.monotonic()
        self.latest_analytics = self._analyse_sample(sample)
        self._update_peaks(sample, self.latest_analytics)
        self._write_csv(sample, self.latest_analytics)

    def _analyse_sample(self, sample: CompleteBatterySample) -> dict[str, object]:
        cells = sample.cells_mv
        cell_min_mv = min(cells) if cells else None
        cell_max_mv = max(cells) if cells else None
        cell_min_index = cells.index(cell_min_mv) + 1 if cell_min_mv is not None else None
        cell_max_index = cells.index(cell_max_mv) + 1 if cell_max_mv is not None else None
        cell_delta_mv = (cell_max_mv - cell_min_mv) if cell_min_mv is not None and cell_max_mv is not None else None

        # Build a baseline from coherent, mechanically unloaded samples. The
        # scooter's electronics may consume close to 1 A at idle, so REST is
        # determined from both BMS current and the fast ECU motor-power stream.
        if sample.mode == "REST":
            self._rest_samples.append(sample)
            if len(self._rest_samples) >= self.baseline_min_samples:
                self.reference_voltage_v = fmean(item.voltage_v for item in self._rest_samples)
                shortest = min(len(item.cells_mv) for item in self._rest_samples)
                self.reference_cells_mv = [
                    fmean(item.cells_mv[index] for item in self._rest_samples)
                    for index in range(shortest)
                ]
                # A fresh resting reference separates load directions.
                self._active_direction = None
        else:
            # Switching directly from discharge to charge (or vice versa)
            # without an intervening resting baseline invalidates the old one.
            if self._active_direction is not None and self._active_direction != sample.mode:
                self.reference_voltage_v = None
                self.reference_cells_mv = None
            self._active_direction = sample.mode
            self._rest_samples.clear()

        pack_sag_v: float | None = None
        charge_rise_v: float | None = None
        resistance_mohm: float | None = None
        worst_cell_sag_mv: float | None = None
        worst_cell_sag_index: int | None = None
        highest_cell_rise_mv: float | None = None
        highest_cell_rise_index: int | None = None

        if self.reference_voltage_v is not None:
            if sample.mode == "DISCHARGING":
                pack_sag_v = max(0.0, self.reference_voltage_v - sample.voltage_v)
                if sample.current_a >= self.resistance_min_current_a and pack_sag_v > 0:
                    resistance_mohm = pack_sag_v / sample.current_a * 1000.0
            elif sample.mode == "CHARGING":
                charge_rise_v = max(0.0, sample.voltage_v - self.reference_voltage_v)
                if abs(sample.current_a) >= self.resistance_min_current_a and charge_rise_v > 0:
                    resistance_mohm = charge_rise_v / abs(sample.current_a) * 1000.0

        if self.reference_cells_mv is not None and cells:
            count = min(len(self.reference_cells_mv), len(cells))
            if sample.mode == "DISCHARGING":
                deviations = [max(0.0, self.reference_cells_mv[i] - cells[i]) for i in range(count)]
                if deviations:
                    worst_cell_sag_mv = max(deviations)
                    worst_cell_sag_index = deviations.index(worst_cell_sag_mv) + 1
            elif sample.mode == "CHARGING":
                deviations = [max(0.0, cells[i] - self.reference_cells_mv[i]) for i in range(count)]
                if deviations:
                    highest_cell_rise_mv = max(deviations)
                    highest_cell_rise_index = deviations.index(highest_cell_rise_mv) + 1

        temperatures = sample.temperatures_c
        temp_min_c = min(temperatures) if temperatures else None
        temp_max_c = max(temperatures) if temperatures else None
        if temp_max_c is not None and self.initial_temperature_c is None:
            self.initial_temperature_c = float(temp_max_c)
        temperature_rise_c = (
            float(temp_max_c) - self.initial_temperature_c
            if temp_max_c is not None and self.initial_temperature_c is not None
            else None
        )

        self._last_mode = sample.mode
        if self.reference_voltage_v is not None:
            baseline_state = "ready"
        elif sample.mode == "REST":
            baseline_state = "establishing"
        else:
            baseline_state = "waiting_for_rest"

        return {
            "cell_min_mv": cell_min_mv,
            "cell_min_index": cell_min_index,
            "cell_max_mv": cell_max_mv,
            "cell_max_index": cell_max_index,
            "cell_delta_mv": cell_delta_mv,
            "temp_min_c": temp_min_c,
            "temp_max_c": temp_max_c,
            "temperature_rise_c": temperature_rise_c,
            "reference_voltage_v": self.reference_voltage_v,
            "baseline_ready": self.reference_voltage_v is not None,
            "baseline_state": baseline_state,
            "baseline_samples": len(self._rest_samples),
            "baseline_required_samples": self.baseline_min_samples,
            "pack_sag_v": pack_sag_v,
            "charge_rise_v": charge_rise_v,
            "resistance_estimate_mohm": resistance_mohm,
            "worst_cell_sag_mv": worst_cell_sag_mv,
            "worst_cell_sag_index": worst_cell_sag_index,
            "highest_cell_rise_mv": highest_cell_rise_mv,
            "highest_cell_rise_index": highest_cell_rise_index,
        }

    def _update_peaks(self, sample: CompleteBatterySample, analytics: dict[str, object]) -> None:
        if sample.current_a > 0:
            self.peaks["discharge_current_a"] = max(
                float(self.peaks["discharge_current_a"] or 0.0), sample.current_a
            )
        elif sample.current_a < 0:
            self.peaks["charge_current_a"] = max(
                float(self.peaks["charge_current_a"] or 0.0), abs(sample.current_a)
            )

        if sample.power_w > 0:
            self.peaks["discharge_power_w"] = max(
                float(self.peaks["discharge_power_w"] or 0.0), sample.power_w
            )
        elif sample.power_w < 0:
            self.peaks["charge_power_w"] = max(
                float(self.peaks["charge_power_w"] or 0.0), abs(sample.power_w)
            )

        for key, peak_key in (
            ("pack_sag_v", "pack_sag_v"),
            ("charge_rise_v", "charge_rise_v"),
            ("cell_delta_mv", "cell_delta_mv"),
            ("worst_cell_sag_mv", "cell_sag_mv"),
            ("highest_cell_rise_mv", "cell_rise_mv"),
        ):
            value = analytics.get(key)
            if isinstance(value, (int, float)):
                self.peaks[peak_key] = max(float(self.peaks[peak_key] or 0.0), float(value))

        resistance = analytics.get("resistance_estimate_mohm")
        if sample.mode == "DISCHARGING" and isinstance(resistance, (int, float)):
            self.peaks["discharge_resistance_mohm"] = max(
                float(self.peaks["discharge_resistance_mohm"] or 0.0), float(resistance)
            )

        temp_max = analytics.get("temp_max_c")
        if isinstance(temp_max, (int, float)):
            prior = self.peaks["temperature_c"]
            self.peaks["temperature_c"] = float(temp_max) if prior is None else max(float(prior), float(temp_max))
        temp_rise = analytics.get("temperature_rise_c")
        if isinstance(temp_rise, (int, float)):
            self.peaks["temperature_rise_c"] = max(
                float(self.peaks["temperature_rise_c"] or 0.0), float(temp_rise)
            )

    @staticmethod
    def _round_or_blank(value: object, digits: int = 3) -> object:
        if isinstance(value, float):
            if not math.isfinite(value):
                return ""
            return round(value, digits)
        return "" if value is None else value

    def _write_csv(self, sample: CompleteBatterySample, analytics: dict[str, object]) -> None:
        if self._csv_writer is None or self._csv_handle is None:
            return

        row: dict[str, object] = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            "elapsed_s": round(sample.elapsed_s, 3),
            "bms_serial": sample.bms_serial or "",
            "bms_firmware": sample.bms_firmware or "",
            "mode": sample.mode,
            "bms_voltage_v": round(sample.voltage_v, 3),
            "bms_current_a": round(sample.current_a, 3),
            "bms_power_w": round(sample.power_w, 2),
            "ecu_motor_power_latest_w": self._round_or_blank(sample.motor_power_latest_w, 2),
            "ecu_motor_power_average_w": self._round_or_blank(sample.motor_power_average_w, 2),
            "ecu_motor_power_peak_w": self._round_or_blank(sample.motor_power_peak_w, 2),
            "battery_soc_percent": self._round_or_blank(sample.soc_percent),
            "health_percent": self._round_or_blank(sample.health_percent),
            "cell_min_mv": self._round_or_blank(analytics.get("cell_min_mv")),
            "cell_min_index": self._round_or_blank(analytics.get("cell_min_index")),
            "cell_max_mv": self._round_or_blank(analytics.get("cell_max_mv")),
            "cell_max_index": self._round_or_blank(analytics.get("cell_max_index")),
            "cell_delta_mv": self._round_or_blank(analytics.get("cell_delta_mv")),
            "temp_min_c": self._round_or_blank(analytics.get("temp_min_c")),
            "temp_max_c": self._round_or_blank(analytics.get("temp_max_c")),
            "reference_voltage_v": self._round_or_blank(analytics.get("reference_voltage_v"), 3),
            "pack_sag_v": self._round_or_blank(analytics.get("pack_sag_v"), 3),
            "charge_rise_v": self._round_or_blank(analytics.get("charge_rise_v"), 3),
            "resistance_estimate_mohm": self._round_or_blank(
                analytics.get("resistance_estimate_mohm"), 2
            ),
            "worst_cell_sag_mv": self._round_or_blank(analytics.get("worst_cell_sag_mv"), 1),
            "worst_cell_sag_index": self._round_or_blank(analytics.get("worst_cell_sag_index")),
            "highest_cell_rise_mv": self._round_or_blank(analytics.get("highest_cell_rise_mv"), 1),
            "highest_cell_rise_index": self._round_or_blank(analytics.get("highest_cell_rise_index")),
        }

        for index in range(10):
            row[f"cell_{index + 1}_mv"] = sample.cells_mv[index] if index < len(sample.cells_mv) else ""
        for index in range(4):
            row[f"temp_{index + 1}_c"] = (
                sample.temperatures_c[index] if index < len(sample.temperatures_c) else ""
            )

        self._csv_writer.writerow(row)
        self._csv_handle.flush()

    def snapshot(self) -> dict[str, object]:
        sample = self.latest_sample
        status_age = (
            None if self.last_status_monotonic is None else time.monotonic() - self.last_status_monotonic
        )
        complete_age = (
            None if self.last_complete_monotonic is None else time.monotonic() - self.last_complete_monotonic
        )
        return {
            "sample": sample,
            "analytics": dict(self.latest_analytics),
            "peaks": dict(self.peaks),
            "status_age_s": status_age,
            "complete_age_s": complete_age,
            "reference_voltage_v": self.reference_voltage_v,
            "baseline_samples": len(self._rest_samples),
            "baseline_required_samples": self.baseline_min_samples,
            "csv_path": str(self.csv_path) if self.csv_path is not None else None,
        }
