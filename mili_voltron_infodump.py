"""One-shot, structured BMS information dump and JSON exporter."""

from __future__ import annotations

import json
import re
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

from mili_voltron_polling import PollRequest


BMS_BOOL_BITS = {
    0: "PASSWORD",
    1: "ACT",
    2: "DMOS",
    3: "CMOS",
    4: "WRITE_CMD",
    5: "DISCHARGE",
    6: "CHARGE",
    7: "CHARGERIN",
    8: "DISOVER",
    9: "CHGOVER",
    10: "VOERTEMP",
    11: "TEST_MODE",
    12: "MINI_POWERON",
}

WARNING_LABELS = {
    0x02: "SPI",
    0x03: "OVR_CHG",
    0x04: "LOAD",
    0x05: "JUMP_APP",
    0x06: "PASSWORD",
    0x07: "PUPIN",
}


@dataclass(frozen=True, slots=True)
class RegisterSpec:
    index: int
    var_name: str
    interpreted_name: str
    width_registers: int = 1
    decoder: str = "raw"


def _register_specs() -> tuple[RegisterSpec, ...]:
    identity = (
        RegisterSpec(0x10, "BMS_INF_SN", "BMS serial number", 7, "serial"),
        RegisterSpec(0x17, "BMS_INF_FW_VERSION", "BMS firmware version", decoder="firmware"),
        RegisterSpec(0x18, "BMS_INF_DSG_FULL_CAP", "Rated capacity", decoder="mah"),
        RegisterSpec(0x19, "BMS_INF_CRT_FULL_CAP", "Current full capacity", decoder="mah"),
        RegisterSpec(0x1A, "BMS_INF_DSG_VOL", "Discharge voltage (unconfirmed)"),
        RegisterSpec(0x1B, "BMS_INF_DIS_LOOP", "Discharge loop (unconfirmed)"),
        RegisterSpec(0x1C, "BMS_INF_CHG_CNT", "Charge count", decoder="count"),
        RegisterSpec(0x1D, "BMS_INF_TIME_L", "Operating time, low word (unconfirmed)"),
        RegisterSpec(0x1E, "BMS_INF_TIME_H", "Operating time, high word (unconfirmed)"),
        RegisterSpec(0x1F, "BMS_INF_OCRT_ODIS_CNT", "Overcurrent/overdischarge count", decoder="count"),
        RegisterSpec(0x20, "BMS_INF_PRD_DATE", "Production date (unconfirmed)"),
        RegisterSpec(0x21, "BMS_INF_FUSE_DATE", "Fuse date (unconfirmed)"),
        RegisterSpec(0x22, "BMS_INF_F_CODE", "F code (unconfirmed)"),
        RegisterSpec(0x23, "BMS_INF_B_CODE", "B code (unconfirmed)"),
    )
    live = (
        RegisterSpec(0x30, "BMS_INF_BOOL", "BMS state flags", decoder="flags"),
        RegisterSpec(0x31, "BMS_INF_CRT_CAP", "Remaining capacity", decoder="mah"),
        RegisterSpec(0x32, "BMS_UNKNOWN_32", "Unknown register 0x32"),
        RegisterSpec(0x33, "BMS_INF_CRT", "Pack current", decoder="current"),
        RegisterSpec(0x34, "BMS_INF_VOL", "Pack voltage", decoder="voltage"),
        RegisterSpec(0x35, "BMS_INF_TEMP", "Temperature sensors 1-2", decoder="temp12"),
        RegisterSpec(0x36, "BMS_INF_BLA_STATE", "Balancing state (unconfirmed)"),
        RegisterSpec(0x37, "BMS_INF_ODIS_STATE", "Overdischarge state (unconfirmed)"),
        RegisterSpec(0x38, "BMS_INF_OCHG_STATE", "Overcharge state (unconfirmed)"),
        RegisterSpec(0x39, "BMS_INF_CAP_COULO", "Coulomb-derived capacity", decoder="mah"),
        RegisterSpec(0x3A, "BMS_INF_CAP_VOL", "Voltage-derived capacity", decoder="mah"),
        RegisterSpec(0x3B, "BMS_INF_CAP_HEALTH", "Capacity health (unconfirmed)", decoder="health"),
        RegisterSpec(0x3C, "BMS_ST_CAP_VOL", "Capacity/voltage state (unconfirmed)"),
        RegisterSpec(0x3D, "BMS_INF_CTO_INDEX", "CTO index (unconfirmed)"),
        RegisterSpec(0x3E, "BMS_INF_CAP_PCT", "State of charge", decoder="soc"),
        RegisterSpec(0x3F, "BMS_UNKNOWN_3F", "Unknown register 0x3F"),
    )
    cells = tuple(
        RegisterSpec(
            0x40 + cell,
            f"BMS_INF_CELL_VOL_{cell}",
            f"Cell {cell + 1} voltage",
            decoder="cell",
        )
        for cell in range(10)
    )
    extended = (
        RegisterSpec(0x50, "BMS_INF_BEEN_ACT", "Activation history (unconfirmed)"),
        RegisterSpec(0x51, "BMS_INF_PCB_VER", "PCB version", decoder="pcb"),
        RegisterSpec(0x52, "BMS_ST_TEMP34", "Temperature sensors 3-4", decoder="temp34"),
        RegisterSpec(0x53, "BMS_ST_TEMP56", "Temperature sensors 5-6", decoder="temp56"),
    )
    deep = tuple(
        RegisterSpec(
            0x70 + part,
            f"BMS_CPUID_{chr(ord('A') + part)}",
            f"CPU ID segment {chr(ord('A') + part)}",
            decoder="cpu_id",
        )
        for part in range(6)
    ) + tuple(
        RegisterSpec(index, f"BMS_UNKNOWN_{index:02X}", f"Unknown register 0x{index:02X}")
        for index in range(0x76, 0x7B)
    ) + (
        RegisterSpec(0x7B, "BMS_INF_WARNING", "BMS warning code", decoder="warning"),
    )
    return identity + live + cells + extended + deep


REGISTER_SPECS = _register_specs()


@dataclass(slots=True)
class _Pending:
    request: PollRequest
    sent_at: float
    attempts: int


class InfoDumpSession:
    """Four-window, single-flight reader used by ``--infodump``.

    The Rust Voltron implementation reads most named registers individually.
    These windows cover the same map plus nearby unknown registers with four
    requests, while keeping every request below the confirmed 64-byte limit.
    Failed windows are recorded and the remaining windows are still attempted.
    """

    REQUESTS = (
        PollRequest(0x22, 0x10, 40, "INFODUMP_IDENTITY"),  # 0x10..0x23
        PollRequest(0x22, 0x30, 52, "INFODUMP_TELEMETRY"),  # 0x30..0x49
        PollRequest(0x22, 0x50, 8, "INFODUMP_EXTENDED"),  # 0x50..0x53
        PollRequest(0x22, 0x70, 24, "INFODUMP_DEEP"),  # 0x70..0x7B
    )

    def __init__(
        self,
        *,
        response_timeout_s: float = 0.35,
        inter_request_gap_s: float = 0.01,
        retries: int = 1,
    ) -> None:
        self.response_timeout_s = response_timeout_s
        self.inter_request_gap_s = inter_request_gap_s
        self.retries = max(0, retries)
        self.queue: deque[PollRequest] = deque(self.REQUESTS)
        self.pending: _Pending | None = None
        self.next_send_at = 0.0
        self.blocks: dict[int, bytes] = {}
        self.errors: dict[int, str] = {}
        self.sent = 0
        self.retries_sent = 0
        self.unexpected_replies = 0

    @property
    def complete(self) -> bool:
        return not self.queue and self.pending is None

    def tick(self, now: float, send: Callable[[bytes], None]) -> None:
        if self.pending is not None:
            if now - self.pending.sent_at < self.response_timeout_s:
                return
            if self.pending.attempts <= self.retries:
                send(self.pending.request.frame())
                self.pending.sent_at = now
                self.pending.attempts += 1
                self.sent += 1
                self.retries_sent += 1
                return
            request = self.pending.request
            self.errors[request.index] = (
                f"no matching {request.length}-byte reply after {self.pending.attempts} attempts"
            )
            self.pending = None
            self.next_send_at = now + self.inter_request_gap_s

        if self.pending is None and self.queue and now >= self.next_send_at:
            request = self.queue.popleft()
            send(request.frame())
            self.pending = _Pending(request=request, sent_at=now, attempts=1)
            self.sent += 1

    def observe_packet(self, packet: object, now: float) -> bool:
        pending = self.pending
        if pending is None:
            return False
        request = pending.request
        if not (
            getattr(packet, "checksum_ok", False)
            and getattr(packet, "cmd", None) == 0x04
            and getattr(packet, "src", None) == request.dst
            and getattr(packet, "dst", None) == 0x3D
            and getattr(packet, "index", None) == request.index
            and getattr(packet, "length", None) == request.length
        ):
            if getattr(packet, "cmd", None) == 0x04 and getattr(packet, "dst", None) == 0x3D:
                self.unexpected_replies += 1
            return False
        self.blocks[request.index] = bytes(getattr(packet, "payload"))
        self.pending = None
        self.next_send_at = now + self.inter_request_gap_s
        return True


def _firmware(value: int) -> str:
    return f"{value >> 8}.{(value >> 4) & 0x0F}.{value & 0x0F}"


def _temperature(raw: int) -> int | None:
    return None if raw in (0, 0xFF) else raw - 20


def _find_bytes(blocks: dict[int, bytes], index: int, width_registers: int) -> bytes | None:
    size = width_registers * 2
    for start, payload in blocks.items():
        offset = (index - start) * 2
        if offset >= 0 and offset + size <= len(payload):
            return payload[offset:offset + size]
    return None


def _decode_record(spec: RegisterSpec, raw: bytes | None) -> dict[str, object]:
    record: dict[str, object] = {
        "offset": (
            f"0x{spec.index:02X}-0x{spec.index + spec.width_registers - 1:02X}"
            if spec.width_registers > 1
            else f"0x{spec.index:02X}"
        ),
        "var_name": spec.var_name,
        "interpreted_name": spec.interpreted_name,
        "decoded": None,
        "hex": None,
        "bin": None,
        "units": None,
    }
    if raw is None:
        return record

    if spec.decoder == "serial":
        serial = raw.split(b"\x00", 1)[0].split(b"\xFF", 1)[0].decode("ascii", errors="replace").strip()
        record.update(decoded=serial, hex=raw.hex(" ").upper())
        return record

    value = int.from_bytes(raw[:2], "little")
    signed = int.from_bytes(raw[:2], "little", signed=True)
    record["hex"] = f"0x{value:04X}"
    record["bin"] = f"0b{value:016b}"

    if spec.decoder == "firmware":
        record["decoded"] = _firmware(value)
    elif spec.decoder == "mah":
        # All observed BMS capacity fields use signed words; negative values
        # are meaningful failure/sentinel states and must not wrap to u16.
        record.update(decoded=signed, units="mAh")
    elif spec.decoder == "count":
        record.update(decoded=value, units="count")
    elif spec.decoder == "current":
        record.update(decoded=round(signed * 0.01, 2), units="A")
    elif spec.decoder == "voltage":
        record.update(decoded=round(value * 0.01, 2), units="V")
    elif spec.decoder.startswith("temp"):
        labels = {
            "temp12": ("T1", "T2"),
            "temp34": ("T3", "T4"),
            "temp56": ("T5", "T6"),
        }[spec.decoder]
        temperatures = {
            labels[0]: _temperature((value >> 8) & 0xFF),
            labels[1]: _temperature(value & 0xFF),
        }
        record.update(decoded=temperatures, units="°C")
    elif spec.decoder == "flags":
        active = [name for bit, name in BMS_BOOL_BITS.items() if value & (1 << bit)]
        known_mask = sum(1 << bit for bit in BMS_BOOL_BITS)
        unknown_active_bits = [bit for bit in range(16) if value & (1 << bit) and not known_mask & (1 << bit)]
        record["decoded"] = active + [f"BIT_{bit}_UNKNOWN" for bit in unknown_active_bits]
        record["bitflags"] = [
            {
                "bit": bit,
                "mask": f"0x{1 << bit:04X}",
                "name": name,
                "active": bool(value & (1 << bit)),
                "known": True,
            }
            for bit, name in BMS_BOOL_BITS.items()
        ] + [
            {
                "bit": bit,
                "mask": f"0x{1 << bit:04X}",
                "name": None,
                "active": True,
                "known": False,
            }
            for bit in unknown_active_bits
        ]
    elif spec.decoder == "soc":
        record.update(decoded=signed if 0 <= signed <= 100 else None, units="%")
    elif spec.decoder == "cell":
        record.update(decoded=value, units="mV")
    elif spec.decoder == "health":
        record.update(decoded=value if 1 <= value <= 100 else None, units="%")
    elif spec.decoder == "warning":
        label = WARNING_LABELS.get(value & 0xFF)
        record["decoded"] = label if label is not None else value
    elif spec.decoder == "pcb":
        record["decoded"] = value
    elif spec.decoder == "cpu_id":
        record["decoded"] = raw.hex().upper()
    else:
        record["decoded"] = value
    return record


def build_infodump_snapshot(
    blocks: dict[int, bytes],
    errors: dict[int, str],
    *,
    port: str,
    baud: int,
    sent: int,
    retries: int,
    unexpected_replies: int,
) -> dict[str, object]:
    records = [_decode_record(spec, _find_bytes(blocks, spec.index, spec.width_registers)) for spec in REGISTER_SPECS]
    by_index = {spec.index: record for spec, record in zip(REGISTER_SPECS, records)}

    cell_values = [
        int(by_index[index]["decoded"])
        for index in range(0x40, 0x4A)
        if by_index[index]["decoded"] is not None
    ]
    cpu_id = "".join(
        str(by_index[index]["decoded"] or "") for index in range(0x70, 0x76)
    ) or None
    battery_id = by_index[0x10]["decoded"] or None
    summary = {
        "serial": battery_id,
        "firmware": by_index[0x17]["decoded"],
        "design_full_capacity_mah": by_index[0x18]["decoded"],
        "current_full_capacity_mah": by_index[0x19]["decoded"],
        "soc_percent": by_index[0x3E]["decoded"],
        "cell_min_mv": min(cell_values) if cell_values else None,
        "cell_max_mv": max(cell_values) if cell_values else None,
        "cell_delta_mv": max(cell_values) - min(cell_values) if cell_values else None,
        "cpu_id_hex": cpu_id,
    }
    return {
        "schema": "mili-voltron-infodump/v2",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "battery_id": battery_id,
        "source": {"port": port, "baud": baud, "bms_address": "0x22"},
        "capture": {
            "complete": not errors and len(blocks) == len(InfoDumpSession.REQUESTS),
            "requests_sent": sent,
            "retries_sent": retries,
            "unexpected_replies": unexpected_replies,
            "windows": [
                {
                    "start_index": f"0x{request.index:02X}",
                    "length_bytes": request.length,
                    "received": request.index in blocks,
                    "raw_hex": blocks[request.index].hex(" ").upper() if request.index in blocks else None,
                    "error": errors.get(request.index),
                }
                for request in InfoDumpSession.REQUESTS
            ],
        },
        "summary": summary,
        "registers": records,
    }


def write_infodump(snapshot: dict[str, object], path: str | Path) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as handle:
        json.dump(snapshot, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def infodump_path(directory: str | Path, timestamp: str, battery_id: object) -> Path:
    raw_id = str(battery_id or "unknown-battery")
    safe_id = re.sub(r"[^A-Za-z0-9._-]+", "_", raw_id).strip("._-")
    return Path(directory) / f"{timestamp}-{safe_id or 'unknown-battery'}.json"
