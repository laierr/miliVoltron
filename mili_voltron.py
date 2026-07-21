#!/usr/bin/env python3
"""miliVoltron: Ninebot UART decoder, dashboard, battery logger and poller."""

from __future__ import annotations

import argparse
import json
import os
import re
import select
import signal
import subprocess
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Iterable, Iterator, TextIO

from mili_voltron_battery import BatteryAnalyzer
from mili_voltron_config import load_config
from mili_voltron_defs import (
    BMS_CURRENT_SCALE_A,
    BMS_INDEX,
    BMS_VOLTAGE_SCALE_V,
    COMMANDS,
    DEVICES,
    ECU_ERRORS,
    ECU_FLAG_BITS,
    ECU_FLAGS_INTERPRETED_MASK,
    ECU_INDEX,
    SOF,
    VEHICLE_STATES,
)
from mili_voltron_infodump import (
    InfoDumpSession,
    build_infodump_snapshot,
    infodump_path,
    write_infodump,
)
from mili_voltron_polling import InquisitorPoller


class Ansi:
    RESET = "\x1b[0m"
    BOLD = "\x1b[1m"
    DIM = "\x1b[2m"
    RED = "\x1b[31m"
    GREEN = "\x1b[32m"
    YELLOW = "\x1b[33m"
    BLUE = "\x1b[34m"
    MAGENTA = "\x1b[35m"
    CYAN = "\x1b[36m"
    WHITE = "\x1b[37m"

    SOF = "\x1b[1;35m"
    LENGTH = "\x1b[1;36m"
    ADDRESS = "\x1b[1;33m"
    COMMAND = "\x1b[1;32m"
    INDEX = "\x1b[1;34m"
    PAYLOAD = "\x1b[0;37m"
    CHECKSUM = "\x1b[1;31m"


def paint(text: object, colour: str, enabled: bool, *, bold: bool = False) -> str:
    rendered = str(text)
    if not enabled:
        return rendered
    prefix = (Ansi.BOLD if bold else "") + colour
    return f"{prefix}{rendered}{Ansi.RESET}"


@dataclass(slots=True)
class Packet:
    timestamp_ns: int
    raw: bytes

    @property
    def length(self) -> int:
        return self.raw[2]

    @property
    def src(self) -> int:
        return self.raw[3]

    @property
    def dst(self) -> int:
        return self.raw[4]

    @property
    def cmd(self) -> int:
        return self.raw[5]

    @property
    def index(self) -> int:
        return self.raw[6]

    @property
    def payload(self) -> bytes:
        return self.raw[7:-2]

    @property
    def checksum_received(self) -> int:
        return int.from_bytes(self.raw[-2:], "little")

    @property
    def checksum_expected(self) -> int:
        return 0xFFFF - (sum(self.raw[2:-2]) & 0xFFFF)

    @property
    def checksum_ok(self) -> bool:
        return self.checksum_received == self.checksum_expected


def u16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=False)


def i16(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def u32(data: bytes, offset: int = 0) -> int:
    return int.from_bytes(data[offset:offset + 4], "little", signed=False)


def device_name(value: int) -> str:
    return DEVICES.get(value, f"0x{value:02X}")


def command_name(value: int) -> str:
    return COMMANDS.get(value, f"CMD_0x{value:02X}")


def index_name(packet: Packet) -> str:
    if packet.src in (0x22, 0x23) or packet.dst in (0x22, 0x23):
        return BMS_INDEX.get(packet.index, f"INDEX_0x{packet.index:02X}")
    if 0xDA <= packet.index <= 0xDF:
        return "CPU_ID"
    return ECU_INDEX.get(packet.index, f"INDEX_0x{packet.index:02X}")


def decode_ascii(payload: bytes) -> str:
    clean = payload.split(b"\x00", 1)[0].split(b"\xff", 1)[0]
    return clean.decode("ascii", errors="replace")


def decode_bms_firmware_word(value: int) -> str:
    """Render the observed packed firmware word, e.g. 0x0652 -> 6.5.2."""

    major = value >> 8
    minor = (value >> 4) & 0x0F
    patch = value & 0x0F
    return f"{major}.{minor}.{patch}"


def decode_temperature_word(value: int) -> list[int | None]:
    """Decode one packed pair in canonical high-byte, low-byte sensor order."""

    raw_values = ((value >> 8) & 0xFF, value & 0xFF)
    return [None if raw == 0 else raw - 20 for raw in raw_values]


def optional_celsius(raw: int) -> int | None:
    """Treat raw 0 as an absent / unpopulated ECU temperature sensor."""

    return None if raw == 0 else raw


def decode_packet(packet: Packet) -> dict[str, object]:
    p = packet.payload

    if packet.cmd == 0x55 and packet.src == 0x20:
        if len(p) != 25:
            return {"kind": "heartbeat", "payload_len": len(p)}
        flags = p[6]
        speed = p[9]
        state = p[5]
        error = p[7]
        ecu_temps_raw = list(p[12:17])
        return {
            "kind": "heartbeat",
            "motor_power_w": i16(p, 0),
            "ecu_pack_voltage_v": u16(p, 2),
            "ecu_soc_reported_percent": p[4],
            "vehicle_state_raw": state,
            "vehicle_state": VEHICLE_STATES.get(state, f"STATE_{state}"),
            "ecu_flags": flags,
            "battery_compartment_closed": bool(flags & 0x01),
            "unknown_bit_2": bool(flags & 0x04),
            "left_indicator_on": bool(flags & 0x08),
            "right_indicator_on": bool(flags & 0x10),
            "unknown_flags_mask": flags & ~ECU_FLAGS_INTERPRETED_MASK,
            "error_code": error,
            "error_name": ECU_ERRORS.get(error, f"ERROR_{error}"),
            "alarm_or_status_raw": p[8],
            "speed_kmh": speed,
            # Live Boolean derived from this heartbeat's speed field.  It is
            # intentionally displayed as state, not recorded as an event.
            "wheel_moving": speed > 0,
            "wheel_counter_raw": u16(p, 10),
            # Heartbeat-only temperatures (°C). Keep these separate from
            # Inquisitor/BMS-polled T1–T6 even when offsets 15–16 echo BMS T2/T1.
            # Raw 0 means absent on this hardware (often the unknown slot).
            "ecu_temperatures_raw_c": ecu_temps_raw,
            "mosfet_radiator_temperature_reported_c": optional_celsius(ecu_temps_raw[0]),
            "motor_temperature_reported_c": optional_celsius(ecu_temps_raw[1]),
            "ecu_temperature_unknown_reported_c": optional_celsius(ecu_temps_raw[2]),
            "ecu_bms_t2_temperature_reported_c": optional_celsius(ecu_temps_raw[3]),
            "ecu_bms_t1_temperature_reported_c": optional_celsius(ecu_temps_raw[4]),
            "ride_time_seconds": u32(p, 17),
            "odometer_metres": u32(p, 21),
        }

    if packet.cmd == 0x04 and packet.src in (0x22, 0x23) and packet.index == 0x30:
        if len(p) >= 10:
            current_raw = i16(p, 6)
            voltage_raw = u16(p, 8)
            current_a = current_raw * BMS_CURRENT_SCALE_A
            voltage_v = voltage_raw * BMS_VOLTAGE_SCALE_V
            result: dict[str, object] = {
                "kind": "bms_telemetry" if len(p) >= 52 else "bms_status",
                "bool_flags_raw": u16(p, 0),
                "remaining_capacity_reported_mah": i16(p, 2),
                "status_prefix_hex": p[:6].hex(),
                "current_raw": current_raw,
                "current_a": round(current_a, 3),
                "voltage_raw": voltage_raw,
                "voltage_v": round(voltage_v, 3),
                "power_w": round(voltage_v * current_a, 2),
                "tail_hex": p[10:].hex(),
            }
            if len(p) >= 12:
                result["temperatures_12_c"] = decode_temperature_word(u16(p, 10))
            if len(p) >= 24:
                result.update({
                    "coulomb_capacity_reported_mah": i16(p, 18),
                    "voltage_capacity_reported_mah": i16(p, 20),
                    "register_3b_raw": u16(p, 22),
                })
            if len(p) >= 30:
                # Capacities and SOC are signed. Accept only a usable 0..100%
                # range; values outside that (including the 0xFFFF / -1 sentinel)
                # are treated as invalid / unavailable.
                soc_raw = i16(p, 28)
                soc_invalid = soc_raw < 0 or soc_raw > 100
                result.update({
                    "bms_soc_reported_raw": soc_raw,
                    "bms_soc_reported_percent": None if soc_invalid else soc_raw,
                    "bms_soc_reported_valid": not soc_invalid,
                })
            if len(p) >= 52:
                cells = [u16(p, offset) for offset in range(32, 52, 2)]
                result.update({
                    "cells_mv": cells,
                    "pack_v": round(sum(cells) / 1000, 3),
                    "delta_mv": max(cells) - min(cells),
                    "min_cell": cells.index(min(cells)) + 1,
                    "max_cell": cells.index(max(cells)) + 1,
                })
            return result
        return {"kind": "bms_status", "payload_hex": p.hex()}

    if packet.cmd == 0x04 and packet.index == 0x10:
        # BMS register 0x10 has a confirmed 16-byte variant:
        # bytes 0–13 = serial, bytes 14–15 = firmware word.
        serial_payload = p[:14] if len(p) >= 14 else p
        result: dict[str, object] = {
            "kind": "serial",
            "serial": decode_ascii(serial_payload),
        }
        if packet.src in (0x22, 0x23) and len(p) >= 16:
            firmware_raw = u16(p, 14)
            result.update({
                "firmware_raw_u16": firmware_raw,
                "firmware_hex": f"0x{firmware_raw:04X}",
                "firmware_version": decode_bms_firmware_word(firmware_raw),
            })
        if packet.src in (0x22, 0x23) and len(p) >= 20:
            result.update({
                "design_full_capacity_reported_mah": i16(p, 16),
                "current_full_capacity_reported_mah": i16(p, 18),
            })
        if packet.src in (0x22, 0x23) and len(p) >= 24:
            result.update({
                "register_1a_raw": u16(p, 20),
                "register_1b_raw": u16(p, 22),
            })
        return result

    if packet.cmd == 0x04 and packet.index == 0x17 and packet.src in (0x22, 0x23) and len(p) >= 2:
        raw = u16(p)
        return {
            "kind": "firmware",
            "raw_u16": raw,
            "bytes": p[:2].hex(),
            "version": decode_bms_firmware_word(raw),
        }

    if packet.cmd == 0x04 and packet.index == 0x1A and packet.src == 0x20 and len(p) >= 2:
        return {"kind": "firmware", "raw_u16": u16(p), "bytes": p[:2].hex()}

    if packet.cmd == 0x04 and packet.src in (0x22, 0x23) and packet.index == 0x40:
        cells = [u16(p, i) for i in range(0, len(p) - 1, 2)]
        result: dict[str, object] = {"kind": "cell_voltages", "cells_mv": cells}
        if cells:
            result.update({
                "pack_v": round(sum(cells) / 1000, 3),
                "delta_mv": max(cells) - min(cells),
                "min_cell": cells.index(min(cells)) + 1,
                "max_cell": cells.index(max(cells)) + 1,
            })
        return result

    if packet.cmd == 0x04 and packet.src in (0x22, 0x23) and packet.index == 0x3B and len(p) >= 2:
        return {"kind": "register_3b", "raw": u16(p)}

    if packet.cmd == 0x04 and packet.src in (0x22, 0x23) and packet.index == 0x51 and len(p) >= 6:
        return {
            "kind": "pcb_temperature_block",
            "pcb_version": u16(p),
            "temperatures_34_56_c": (
                decode_temperature_word(u16(p, 2))
                + decode_temperature_word(u16(p, 4))
            ),
        }

    if packet.cmd == 0x04 and packet.src == 0x20 and packet.index == 0x34 and len(p) >= 4:
        raw = u32(p)
        return {"kind": "ride_time", "raw": raw, "hours": round(raw / 3600, 2)}

    if packet.cmd == 0x04 and packet.src == 0x20 and packet.index == 0x48 and len(p) >= 2:
        raw = u16(p)
        return {"kind": "battery_voltage", "raw": raw, "voltage_v": round(raw / 100, 3)}

    if packet.cmd == 0x04 and packet.src == 0x20 and 0xDA <= packet.index <= 0xDF:
        return {"kind": "cpu_id", "hex": p.hex()}

    if packet.cmd == 0x01 and p:
        return {"kind": "read_request", "requested_bytes": p[0]}

    if packet.cmd == 0x05 and p:
        return {"kind": "write_ack_or_exrd", "value": p[0]}

    return {"kind": "generic", "payload_hex": p.hex()}


def decode_summary(decoded: dict[str, object]) -> str:
    parts = [str(decoded.get("kind", "generic"))]
    parts.extend(f"{key}={value}" for key, value in decoded.items() if key != "kind")
    return " ".join(parts)


def colourised_hex(packet: Packet, enabled: bool) -> str:
    if not enabled:
        return " ".join(f"{value:02X}" for value in packet.raw)
    chunks: list[str] = []
    for index, value in enumerate(packet.raw):
        if index <= 1:
            colour = Ansi.SOF
        elif index == 2:
            colour = Ansi.LENGTH
        elif index in (3, 4):
            colour = Ansi.ADDRESS
        elif index == 5:
            colour = Ansi.COMMAND
        elif index == 6:
            colour = Ansi.INDEX
        elif index >= len(packet.raw) - 2:
            colour = Ansi.CHECKSUM
        else:
            colour = Ansi.PAYLOAD
        chunks.append(f"{colour}{value:02X}{Ansi.RESET}")
    return " ".join(chunks)


class Framer:
    def __init__(self, max_payload: int = 255) -> None:
        self.max_payload = max_payload
        self.state = 0
        self.packet = bytearray()
        self.expected = 0
        self.sof_timestamp = 0

    def feed(self, chunk: bytes, timestamp_ns: int) -> list[Packet]:
        output: list[Packet] = []
        for value in chunk:
            if self.state == 0:
                if value == 0x5A:
                    self.packet = bytearray((value,))
                    self.sof_timestamp = timestamp_ns
                    self.state = 1
                continue
            if self.state == 1:
                if value == 0xA5:
                    self.packet.append(value)
                    self.state = 2
                elif value == 0x5A:
                    self.packet = bytearray((value,))
                    self.sof_timestamp = timestamp_ns
                else:
                    self.state = 0
                    self.packet.clear()
                continue
            if self.state == 2:
                if value > self.max_payload:
                    self.state = 0
                    self.packet.clear()
                    continue
                self.packet.append(value)
                self.expected = 9 + value
                self.state = 3
                continue
            self.packet.append(value)
            if len(self.packet) == self.expected:
                output.append(Packet(timestamp_ns=self.sof_timestamp, raw=bytes(self.packet)))
                self.state = 0
                self.packet.clear()
            elif len(self.packet) > self.expected:
                self.state = 0
                self.packet.clear()
        return output


def frame_stream(source: Iterable[tuple[int, int]], max_payload: int = 255) -> Iterator[Packet]:
    framer = Framer(max_payload=max_payload)
    for timestamp_ns, value in source:
        yield from framer.feed(bytes((value,)), timestamp_ns)


def configure_serial(port: Path, baud: int) -> None:
    subprocess.run([
        "stty", "-F", str(port), str(baud), "cs8", "-cstopb", "-parenb",
        "-ixon", "-ixoff", "-crtscts", "raw", "-echo",
    ], check=True)


class SerialTransport:
    def __init__(self, port: Path, *, writable: bool) -> None:
        flags = os.O_NOCTTY | os.O_NONBLOCK | (os.O_RDWR if writable else os.O_RDONLY)
        self.fd = os.open(port, flags)
        self.writable = writable

    def read(self, timeout: float) -> bytes:
        ready, _, _ = select.select([self.fd], [], [], max(0.0, timeout))
        if not ready:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def write(self, data: bytes) -> None:
        if not self.writable:
            raise RuntimeError("serial transport is read-only")
        view = memoryview(data)
        while view:
            try:
                written = os.write(self.fd, view)
            except BlockingIOError:
                _, ready, _ = select.select([], [self.fd], [], 0.25)
                if not ready:
                    raise TimeoutError("serial TX timed out")
                continue
            view = view[written:]

    def close(self) -> None:
        os.close(self.fd)


def binary_bytes(path: Path) -> Iterator[tuple[int, int]]:
    with path.open("rb") as handle:
        while chunk := handle.read(4096):
            timestamp = time.monotonic_ns()
            for value in chunk:
                yield timestamp, value


def hex_bytes(path: Path) -> Iterator[tuple[int, int]]:
    text = path.read_text(errors="replace")
    text = re.sub(r"(?m)#.*$", "", text)
    pairs = re.findall(r"(?i)(?<![0-9a-f])[0-9a-f]{2}(?![0-9a-f])", text)
    if not pairs:
        compact = re.sub(r"[^0-9a-fA-F]", "", text)
        if len(compact) % 2:
            raise ValueError("hex input contains an odd number of digits")
        pairs = [compact[index:index + 2] for index in range(0, len(compact), 2)]
    timestamp = time.monotonic_ns()
    for pair in pairs:
        yield timestamp, int(pair, 16)


def discover_ports() -> list[Path]:
    candidates: list[Path] = []
    by_id = Path("/dev/serial/by-id")
    if by_id.is_dir():
        candidates.extend(sorted(path for path in by_id.iterdir() if path.exists()))
    candidates.extend(sorted(Path("/dev").glob("ttyUSB*")))
    candidates.extend(sorted(Path("/dev").glob("ttyACM*")))

    seen_targets: set[str] = set()
    result: list[Path] = []
    for candidate in candidates:
        try:
            target = str(candidate.resolve())
        except OSError:
            target = str(candidate)
        if target in seen_targets:
            continue
        seen_targets.add(target)
        result.append(candidate)
    return result


def choose_port_noninteractive() -> Path:
    ports = discover_ports()
    if not ports:
        raise RuntimeError("no /dev/serial/by-id, /dev/ttyUSB*, or /dev/ttyACM* device found")
    if len(ports) > 1:
        print(
            "Multiple serial devices found; using the first. Pass --port to choose explicitly:\n  "
            + "\n  ".join(str(port) for port in ports),
            file=sys.stderr,
        )
    return ports[0]


class LogWriters:
    def __init__(
        self,
        *,
        raw_log: str | None,
        decoded_log: str | None,
        combined_log: str | None,
        jsonl_log: str | None,
    ) -> None:
        self.paths = {
            "raw": raw_log,
            "decoded": decoded_log,
            "combined": combined_log,
            "jsonl": jsonl_log,
        }
        self.raw = self._open(raw_log)
        self.decoded = self._open(decoded_log)
        self.combined = self._open(combined_log)
        self.jsonl = self._open(jsonl_log)

    @staticmethod
    def _open(path: str | None) -> TextIO | None:
        if not path:
            return None
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        return output.open("x", encoding="utf-8", buffering=1)

    def active_names(self, battery_log: Path | None) -> list[str]:
        names = [name for name, path in self.paths.items() if path]
        if battery_log is not None:
            names.append("battery")
        return names

    def write(
        self,
        *,
        packet: Packet,
        relative: float,
        delta_ms: float,
        label: str,
        packet_no: int,
        decoded: dict[str, object],
    ) -> None:
        src_name = device_name(packet.src)
        dst_name = device_name(packet.dst)
        cmd_name = command_name(packet.cmd)
        idx_name = index_name(packet)
        summary = decode_summary(decoded)
        status = "OK" if packet.checksum_ok else "BAD"
        raw_hex = packet.raw.hex()
        spaced = " ".join(f"{value:02X}" for value in packet.raw)

        if self.raw:
            self.raw.write(
                f"{relative:.6f} {label} {src_name}->{dst_name} checksum={status} raw={raw_hex}\n"
            )
        if self.decoded:
            self.decoded.write(
                f"{relative:.6f} {label} {src_name}->{dst_name} {cmd_name} {idx_name} "
                f"checksum={status} {summary}\n"
            )
        if self.combined:
            self.combined.write(
                f"{relative:.6f} {label} {src_name}->{dst_name} {cmd_name} {idx_name} [{status}]\n"
                f"  raw: {spaced}\n  decoded: {summary}\n\n"
            )
        if self.jsonl:
            json.dump({
                "t": round(relative, 6), "delta_ms": round(delta_ms, 3),
                "line": label, "packet_no": packet_no,
                "src": packet.src, "src_name": src_name,
                "dst": packet.dst, "dst_name": dst_name,
                "cmd": packet.cmd, "cmd_name": cmd_name,
                "index": packet.index, "index_name": idx_name,
                "payload_len": packet.length, "payload_hex": packet.payload.hex(),
                "checksum_ok": packet.checksum_ok,
                "checksum_expected": packet.checksum_expected,
                "checksum_received": packet.checksum_received,
                "raw_hex": raw_hex, "decoded": decoded,
            }, self.jsonl, separators=(",", ":"))
            self.jsonl.write("\n")

    def close(self) -> None:
        for handle in (self.raw, self.decoded, self.combined, self.jsonl):
            if handle is not None:
                handle.close()


@dataclass(slots=True)
class Event:
    elapsed_s: float
    text: str


class DashboardModel:
    def __init__(self, config: dict[str, object]) -> None:
        self.config = config
        dashboard = config["dashboard"]  # type: ignore[index]
        self.events: deque[Event] = deque(maxlen=int(dashboard["recent_changes_limit"]))  # type: ignore[index]
        self.values: dict[str, object] = {}
        self.heartbeat: dict[str, object] = {}
        self.bms_status: dict[str, object] = {}
        self.cells: dict[str, object] = {}
        self.bms_temperatures: dict[str, object] = {}
        self.serials: dict[str, str] = {}
        self.firmware: dict[str, str] = {}
        self.source_seen: dict[str, float] = {}
        self.packet_times: deque[float] = deque()
        self.tx_times: deque[float] = deque()
        self.packet_count = 0
        self.tx_count = 0
        self.checksum_errors = 0
        self.bms_link_state: bool | None = None

    def _tracking(self, name: str) -> bool:
        table = self.config["recent_changes"]  # type: ignore[index]
        return bool(table.get(name, False))  # type: ignore[union-attr]

    def _flag_tracking(self, bit: int) -> bool:
        table = self.config["recent_changes"]  # type: ignore[index]
        flags = table.get("flags", {})  # type: ignore[union-attr]
        return bool(flags.get(f"bit_{bit}", False))

    def _event_change(self, key: str, label: str, value: object, elapsed_s: float, enabled: bool) -> None:
        old = self.values.get(key, _MISSING)
        self.values[key] = value
        if old is not _MISSING and old != value and enabled:
            self.events.appendleft(Event(elapsed_s, f"{label}: {old} → {value}"))

    def observe_packet(self, packet: Packet, now: float, elapsed_s: float, *, tx: bool) -> None:
        if tx:
            self.tx_count += 1
            self.tx_times.append(now)
            return
        self.packet_count += 1
        self.packet_times.append(now)
        self.source_seen[device_name(packet.src)] = now
        if not packet.checksum_ok:
            self.checksum_errors += 1
        if packet.src in (0x22, 0x23):
            if self.bms_link_state is False and self._tracking("bms_link"):
                self.events.appendleft(Event(elapsed_s, "BMS link: OFFLINE → ONLINE"))
            self.bms_link_state = True

    def update(self, decoded: dict[str, object], elapsed_s: float, source_name: str, analyzer: BatteryAnalyzer) -> None:
        kind = str(decoded.get("kind", "generic"))
        if kind == "heartbeat":
            flags = int(decoded.get("ecu_flags", 0))
            prior_flags = self.values.get("ecu_flags")
            if isinstance(prior_flags, int) and prior_flags != flags:
                changed = prior_flags ^ flags
                for bit in range(8):
                    if changed & (1 << bit) and self._flag_tracking(bit):
                        before = "ON" if prior_flags & (1 << bit) else "OFF"
                        after = "ON" if flags & (1 << bit) else "OFF"
                        self.events.appendleft(Event(elapsed_s, f"ECU flag bit {bit}: {before} → {after}"))
            self.values["ecu_flags"] = flags

            self._event_change(
                "vehicle_state", "Vehicle state", decoded.get("vehicle_state"), elapsed_s,
                self._tracking("vehicle_state"),
            )
            error = f"{decoded.get('error_code')} {decoded.get('error_name')}"
            self._event_change("error", "ECU error", error, elapsed_s, self._tracking("error_code"))
            latch = "CLOSED" if decoded.get("battery_compartment_closed") else "OPEN"
            self._event_change("battery_latch", "Battery compartment", latch, elapsed_s, self._tracking("battery_latch"))
            self._event_change(
                "wheel_moving", "Wheel moving", bool(decoded.get("wheel_moving")), elapsed_s,
                self._tracking("wheel_moving"),
            )
            self._event_change(
                "left_indicator", "Left indicator", bool(decoded.get("left_indicator_on")), elapsed_s,
                self._tracking("left_indicator"),
            )
            self._event_change(
                "right_indicator", "Right indicator", bool(decoded.get("right_indicator_on")), elapsed_s,
                self._tracking("right_indicator"),
            )
            self.heartbeat = dict(decoded)

        elif kind in {"bms_status", "bms_telemetry"}:
            self.bms_status = dict(decoded)
            if kind == "bms_telemetry" and isinstance(decoded.get("cells_mv"), list):
                self.cells = dict(decoded)
        elif kind == "cell_voltages":
            self.cells = dict(decoded)
        elif kind == "pcb_temperature_block":
            self.bms_temperatures = dict(decoded)
        elif kind == "serial":
            serial = decoded.get("serial")
            if isinstance(serial, str):
                self.serials[source_name] = serial
            firmware = decoded.get("firmware_version")
            if isinstance(firmware, str):
                self.firmware[source_name] = firmware
            elif isinstance(decoded.get("firmware_raw_u16"), int):
                self.firmware[source_name] = f"0x{int(decoded['firmware_raw_u16']):04X}"
        elif kind == "firmware":
            self.firmware[source_name] = str(
                decoded.get("version") or decoded.get("raw_u16")
            )

        sample = analyzer.latest_sample
        if sample is not None:
            self._event_change(
                "battery_mode", "Battery mode", sample.mode, elapsed_s,
                self._tracking("battery_mode"),
            )

    def tick(self, now: float, elapsed_s: float) -> bool:
        """Update freshness state; return True on a new BMS link-loss event."""
        while self.packet_times and now - self.packet_times[0] > 5.0:
            self.packet_times.popleft()
        while self.tx_times and now - self.tx_times[0] > 5.0:
            self.tx_times.popleft()

        stale_after = float(self.config["dashboard"]["stale_after_s"])  # type: ignore[index]
        bms_seen = max((value for key, value in self.source_seen.items() if key in {"BMS", "BMS2"}), default=None)
        bms_lost_now = False
        if bms_seen is not None and now - bms_seen > stale_after and self.bms_link_state is True:
            if self._tracking("bms_link"):
                self.events.appendleft(Event(elapsed_s, "BMS link: ONLINE → OFFLINE"))
            self.bms_link_state = False
            bms_lost_now = True
            for source_name in ("BMS", "BMS2"):
                self.serials.pop(source_name, None)
                self.firmware.pop(source_name, None)
        return bms_lost_now

    def rate(self, *, tx: bool) -> float:
        queue = self.tx_times if tx else self.packet_times
        return len(queue) / 5.0


_MISSING = object()


class TerminalDashboard:
    def __init__(
        self,
        *,
        model: DashboardModel,
        analyzer: BatteryAnalyzer,
        mode: str,
        port_label: str,
        baud: int,
        logs: list[str],
        colour: bool,
        poller: InquisitorPoller | None,
    ) -> None:
        self.model = model
        self.analyzer = analyzer
        self.mode = mode
        self.port_label = port_label
        self.baud = baud
        self.logs = logs
        self.colour = colour
        self.poller = poller

    def enter(self) -> None:
        sys.stdout.write("\x1b[?1049h\x1b[?25l")
        sys.stdout.flush()

    def exit(self) -> None:
        sys.stdout.write("\x1b[?25h\x1b[?7h\x1b[?1049l")
        sys.stdout.flush()

    def _display_port(self) -> str:
        """Return a compact port label without the full by-id path."""
        raw = self.port_label
        try:
            resolved = Path(raw).resolve()
        except (OSError, RuntimeError):
            return raw
        if "/dev/serial/by-id/" in raw:
            if resolved.name.startswith(("ttyUSB", "ttyACM")):
                return f"/dev/{resolved.name} [by-id]"
            match = re.search(r"(CP210\w*|CH34\w*|FT23\w*|PL2303)", Path(raw).name, re.IGNORECASE)
            adapter = match.group(1) if match else "serial"
            return f"{adapter} [by-id]"
        return raw

    @staticmethod
    def _value(value: object, suffix: str = "", digits: int = 2) -> str:
        if value is None:
            return "—"
        if isinstance(value, float):
            return f"{value:.{digits}f}{suffix}"
        return f"{value}{suffix}"

    @staticmethod
    def _temp_slot(label: str, value: object) -> str:
        text = "—" if value is None else str(value)
        return f"{label}:{text}"

    def _age(self, source: str, now: float) -> str:
        seen = self.model.source_seen.get(source)
        if seen is None:
            return "—"
        return f"{now - seen:.1f}s"

    def _age_any(self, sources: tuple[str, ...], now: float) -> str:
        seen = [self.model.source_seen[source] for source in sources if source in self.model.source_seen]
        if not seen:
            return "—"
        return f"{now - max(seen):.1f}s"

    def render(self, now: float, elapsed_s: float) -> None:
        m = self.model
        h = m.heartbeat
        b = m.bms_status
        cells = m.cells.get("cells_mv", [])
        battery = self.analyzer.snapshot()
        sample = battery.get("sample")
        analytics = battery.get("analytics", {})
        peaks = battery.get("peaks", {})
        temps = getattr(sample, "temperatures_c", []) if sample is not None else []
        temps_text = (
            "  ".join(
                f"T{index}:{'—' if value is None else value}"
                for index, value in enumerate(temps, 1)
            )
            if temps
            else "—"
        )

        active_logs = ",".join(self.logs) if self.logs else "off"
        if self.mode == "inquisitor":
            title = "Voltron the Inquisitor"
            safety = paint("ACTIVE READ-ONLY", Ansi.YELLOW, self.colour, bold=True)
        else:
            title = "miliVoltron"
            safety = paint("PASSIVE RX", Ansi.CYAN, self.colour, bold=True)

        lines = [
            f"{paint(title, Ansi.MAGENTA, self.colour, bold=True)} | {safety} | "
            f"{self._display_port()} @{self.baud} | "
            f"LOG:{paint(active_logs, Ansi.GREEN if self.logs else Ansi.DIM, self.colour)}",
            f"RX {m.rate(tx=False):5.1f} pkt/s   TX {m.rate(tx=True):5.1f} pkt/s   "
            f"packets {m.packet_count}   checksum errors {m.checksum_errors}   "
            f"ECU age {self._age('ECU', now)}   BMS age {self._age_any(('BMS', 'BMS2'), now)}",
            "─" * 112,
            "",
            paint("ECU LIVE", Ansi.BLUE, self.colour, bold=True),
        ]

        state = h.get("vehicle_state", "—")
        error_code = h.get("error_code")
        error_name = h.get("error_name", "—")
        error_colour = Ansi.GREEN if error_code in (None, 0) else (
            Ansi.YELLOW if error_code == 55 else Ansi.RED
        )
        has_heartbeat = bool(h)
        latch = (
            "CLOSED" if h.get("battery_compartment_closed")
            else "OPEN" if has_heartbeat
            else "—"
        )
        wheel = (
            "MOVING" if h.get("wheel_moving")
            else "STOPPED" if has_heartbeat
            else "—"
        )
        left = "ON" if h.get("left_indicator_on") else "off" if has_heartbeat else "—"
        right = "ON" if h.get("right_indicator_on") else "off" if has_heartbeat else "—"
        bit2 = "ON" if h.get("unknown_bit_2") else "off" if has_heartbeat else "—"

        error_text = (
            f"{error_code} {error_name}" if error_code is not None
            else "—"
        )
        flags_text = f"0x{int(h.get('ecu_flags', 0)):02X}" if has_heartbeat else "—"
        lines.append(
            f"State {paint(state, Ansi.CYAN, self.colour)}   "
            f"Error {paint(error_text, error_colour, self.colour)}   "
            f"Flags {flags_text}"
        )
        lines.append(
            f"Compartment {paint(latch, Ansi.GREEN if latch == 'CLOSED' else Ansi.YELLOW, self.colour)}   "
            f"Wheel {paint(wheel, Ansi.CYAN if wheel == 'MOVING' else Ansi.WHITE, self.colour)}   "
            f"Indicators L:{left} R:{right}   unknown bit2:{bit2}"
        )
        lines.append(
            f"Motor {paint(self._value(h.get('motor_power_w'), ' W', 0), Ansi.CYAN, self.colour)}   "
            f"Speed {paint(self._value(h.get('speed_kmh'), ' km/h', 0), Ansi.CYAN, self.colour)}   "
            f"ECU pack {self._value(h.get('ecu_pack_voltage_v'), ' V', 0)}   "
            f"ECU SoC {self._value(h.get('ecu_soc_reported_percent'), '%', 0)}"
        )
        if has_heartbeat:
            ecu_temp_text = "  ".join(
                (
                    self._temp_slot("MOSFET", h.get("mosfet_radiator_temperature_reported_c")),
                    self._temp_slot("motor", h.get("motor_temperature_reported_c")),
                    self._temp_slot("unk", h.get("ecu_temperature_unknown_reported_c")),
                    self._temp_slot("BMS-T2", h.get("ecu_bms_t2_temperature_reported_c")),
                    self._temp_slot("BMS-T1", h.get("ecu_bms_t1_temperature_reported_c")),
                )
            )
        else:
            ecu_temp_text = "—"
        lines.append(f"ECU temps {ecu_temp_text} °C")
        lines.append(
            f"Wheel counter {h.get('wheel_counter_raw', '—')}   "
            f"Ride {self._value((h.get('ride_time_seconds') or 0) / 3600 if h.get('ride_time_seconds') is not None else None, ' h')}   "
            f"Odo {self._value((h.get('odometer_metres') or 0) / 1000 if h.get('odometer_metres') is not None else None, ' km', 3)}"
        )

        lines.extend(["", paint("BMS LIVE", Ansi.BLUE, self.colour, bold=True)])
        if sample is None and not b and not cells and not temps:
            lines.append(paint("Waiting for first complete BMS sample…", Ansi.DIM, self.colour))
        else:
            mode = getattr(sample, "mode", "—")
            mode_colour = (
                Ansi.BLUE if mode == "CHARGING"
                else Ansi.CYAN if mode == "DISCHARGING"
                else Ansi.GREEN
            )
            lines.append(
                f"Mode {paint(mode, mode_colour, self.colour, bold=True)}   "
                f"Voltage {paint(self._value(b.get('voltage_v'), ' V'), Ansi.CYAN, self.colour)}   "
                f"Current {paint(self._value(b.get('current_a'), ' A'), Ansi.CYAN, self.colour)}   "
                f"Power {paint(self._value(b.get('power_w'), ' W', 0), Ansi.CYAN, self.colour)}"
            )
            if isinstance(cells, list) and cells:
                first = "  ".join(f"{index + 1}:{value}" for index, value in enumerate(cells[:5]))
                second = "  ".join(f"{index + 6}:{value}" for index, value in enumerate(cells[5:10]))
                lines.append(f"Cells 1–5 mV    {first}")
                if second:
                    lines.append(f"Cells 6–10 mV   {second}")
                lines.append(
                    f"Min #{m.cells.get('min_cell', '—')}   "
                    f"Max #{m.cells.get('max_cell', '—')}   "
                    f"Delta {m.cells.get('delta_mv', '—')} mV   "
                    f"BMS temps {temps_text} °C"
                )
            else:
                lines.append(paint("Cell voltages pending…", Ansi.DIM, self.colour))

        lines.extend(["", paint("BATTERY HEALTH", Ansi.MAGENTA, self.colour, bold=True)])
        if sample is None:
            lines.append(paint("Waiting for coherent status + cells + temperature sample…", Ansi.DIM, self.colour))
        else:
            reference = analytics.get("reference_voltage_v") if isinstance(analytics, dict) else None
            sag = analytics.get("pack_sag_v") if isinstance(analytics, dict) else None
            rise = analytics.get("charge_rise_v") if isinstance(analytics, dict) else None
            resistance = analytics.get("resistance_estimate_mohm") if isinstance(analytics, dict) else None
            baseline_state = analytics.get("baseline_state") if isinstance(analytics, dict) else None
            baseline_samples = analytics.get("baseline_samples", 0) if isinstance(analytics, dict) else 0
            baseline_required = analytics.get("baseline_required_samples", 0) if isinstance(analytics, dict) else 0
            if reference is not None:
                reference_text = self._value(reference, " V", 3)
            elif baseline_state == "establishing":
                reference_text = f"establishing {baseline_samples}/{baseline_required}"
            else:
                reference_text = "waiting for idle"
            lines.append(
                f"Reference {reference_text}   "
                f"Sag {self._value(sag, ' V', 3)}   "
                f"Charge rise {self._value(rise, ' V', 3)}   "
                f"Z estimate {self._value(resistance, ' mΩ', 1)}"
            )
            bms_soc_raw = getattr(sample, "bms_soc_reported_raw", None)
            bms_soc = getattr(sample, "bms_soc_reported_percent", None)
            if bms_soc is None and isinstance(bms_soc_raw, int):
                bms_soc_text = f"INVALID ({bms_soc_raw})"
            else:
                bms_soc_text = self._value(bms_soc, "%", 1)
            coulomb_soc = (
                analytics.get("coulomb_soc_derived_percent")
                if isinstance(analytics, dict)
                else None
            )
            voltage_soc = (
                analytics.get("voltage_soc_derived_percent")
                if isinstance(analytics, dict)
                else None
            )
            soc_delta = (
                analytics.get("soc_estimate_delta_derived_percentage_points")
                if isinstance(analytics, dict)
                else None
            )
            lines.append(
                f"SOC BMS reported {bms_soc_text}   "
                f"voltage-derived {self._value(voltage_soc, '%', 1)}   "
                f"coulomb-derived {self._value(coulomb_soc, '%', 1)}   "
                f"Δ(V−C) {self._value(soc_delta, ' pp', 1)}"
            )
            worst_index = analytics.get("worst_cell_sag_index") if isinstance(analytics, dict) else None
            rise_index = analytics.get("highest_cell_rise_index") if isinstance(analytics, dict) else None
            lines.append(
                f"Worst sag cell #{worst_index if worst_index is not None else '—'} "
                f"{self._value(analytics.get('worst_cell_sag_mv') if isinstance(analytics, dict) else None, ' mV', 1)}   "
                f"Highest rise cell #{rise_index if rise_index is not None else '—'} "
                f"{self._value(analytics.get('highest_cell_rise_mv') if isinstance(analytics, dict) else None, ' mV', 1)}"
            )
            lines.append(
                f"Peaks: discharge {self._value(peaks.get('discharge_current_a') if isinstance(peaks, dict) else None, ' A')} / "
                f"{self._value(peaks.get('discharge_power_w') if isinstance(peaks, dict) else None, ' W', 0)}   "
                f"charge {self._value(peaks.get('charge_current_a') if isinstance(peaks, dict) else None, ' A')}   "
                f"motor {self._value(peaks.get('motor_power_w') if isinstance(peaks, dict) else None, ' W', 0)} / "
                f"{self._value(peaks.get('motor_temperature_c') if isinstance(peaks, dict) else None, ' °C', 0)}   "
                f"MOSFET {self._value(peaks.get('mosfet_radiator_temperature_c') if isinstance(peaks, dict) else None, ' °C', 0)}   "
                f"Δcell {self._value(peaks.get('cell_delta_mv') if isinstance(peaks, dict) else None, ' mV', 0)}"
            )

        if self.poller is not None:
            poll = self.poller.snapshot(now)
            lines.extend(["", paint("INQUISITOR", Ansi.YELLOW, self.colour, bold=True)])
            lines.append(
                f"Poll {poll['poll_interval_s']} s   sent {poll['sent']}   replies {poll['replies']}   "
                f"timeouts {poll['timeouts']}   retries {poll['retries']}   "
                f"pending {poll['pending'] or '—'}   "
                f"identity {'OK' if poll['identity_valid'] else 'needed'}"
            )

        if m.serials or m.firmware:
            lines.extend(["", paint("IDENTITY", Ansi.BLUE, self.colour, bold=True)])
            serial_text = "  ".join(f"{key}:{value}" for key, value in sorted(m.serials.items())) or "—"
            firmware_text = "  ".join(f"{key}:{value}" for key, value in sorted(m.firmware.items())) or "—"
            lines.append(f"Serials {serial_text}")
            lines.append(f"Firmware {firmware_text}")

        lines.extend(["", paint("RECENT EVENTS", Ansi.BLUE, self.colour, bold=True)])
        if m.events:
            for event in m.events:
                event_time = event.elapsed_s if event.elapsed_s > 0 else elapsed_s
                lines.append(f"{event_time:9.3f}s  {event.text}")
        else:
            lines.append(paint("—", Ansi.DIM, self.colour))

        # Clear each line before rewriting it. This prevents stale suffixes
        # whenever a value or section becomes shorter than the prior frame.
        frame = "\x1b[H" + "".join(f"\x1b[2K\r{line}\n" for line in lines) + "\x1b[J"
        sys.stdout.write(frame)
        sys.stdout.flush()


def print_packet(packet: Packet, relative: float, delta_ms: float, label: str, decoded: dict[str, object], colour: bool) -> None:
    mark = "✓" if packet.checksum_ok else "✗"
    mark = paint(mark, Ansi.GREEN if packet.checksum_ok else Ansi.RED, colour)
    print(
        f"+{relative:10.6f}  Δ{delta_ms:8.3f}ms  {mark}  {label}  "
        f"{device_name(packet.src)} -> {device_name(packet.dst)}  "
        f"{command_name(packet.cmd)}  {index_name(packet)}"
    )
    print("  " + colourised_hex(packet, colour))
    print("  " + decode_summary(decoded))
    if not packet.checksum_ok:
        print(
            f"  checksum expected=0x{packet.checksum_expected:04X} "
            f"received=0x{packet.checksum_received:04X}"
        )


def config_string(value: object) -> str | None:
    if isinstance(value, str) and value:
        return value
    return None


def _append_suffix(path: Path, suffix: str) -> Path:
    return path.with_name(path.name + suffix)


def _numbered_base(base: Path, suffixes: tuple[str, ...]) -> Path:
    """Return a base whose complete output set does not already exist."""
    candidate = base
    counter = 2
    while any(_append_suffix(candidate, suffix).exists() for suffix in suffixes):
        candidate = base.with_name(f"{base.name}-{counter:02d}")
        counter += 1
    return candidate


def _configured_prefix(
    override: str,
    *,
    directory: object,
    prefix: object,
    timestamp_enabled: bool,
    timestamp_text: str,
) -> Path:
    if override:
        base = Path(override)
    else:
        directory_text = str(directory) if directory else "."
        prefix_text = str(prefix) if prefix else "voltron"
        base = Path(directory_text) / prefix_text
    if timestamp_enabled:
        base = base.with_name(f"{base.name}-{timestamp_text}")
    base.parent.mkdir(parents=True, exist_ok=True)
    return base


def _unique_exact_path(path: str | None) -> str | None:
    if not path:
        return None
    original = Path(path)
    original.parent.mkdir(parents=True, exist_ok=True)
    if not original.exists():
        return str(original)
    suffix = "".join(original.suffixes)
    stem = original.name[:-len(suffix)] if suffix else original.name
    counter = 2
    while True:
        candidate = original.with_name(f"{stem}-{counter:02d}{suffix}")
        if not candidate.exists():
            return str(candidate)
        counter += 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Ninebot UART decoder and Voltron test-rig instrument")
    parser.add_argument("--config", type=Path, help="TOML configuration file (default: ./mili-voltron.toml beside script)")
    parser.add_argument("--mode", choices=("passive", "inquisitor"), default=None)
    parser.add_argument("--inquisitor", action="store_true", help="shortcut for --mode inquisitor")
    parser.add_argument("-p", "--port", type=Path)
    parser.add_argument("--binary", type=Path)
    parser.add_argument("--hex", dest="hex_file", type=Path)
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("-b", "--baud", type=int)
    parser.add_argument("--label", default="RX")
    parser.add_argument("--dashboard", action="store_true")
    parser.add_argument(
        "--infodump", action="store_true",
        help="read the known BMS register map once and save JSON under the battery-log directory",
    )
    parser.add_argument("--refresh-rate", type=float)
    parser.add_argument("--poll-interval", type=float)
    parser.add_argument("--response-timeout", type=float)
    parser.add_argument(
        "--all-logs", nargs="?", const="", metavar="PREFIX",
        help="enable raw/decoded/combined/JSONL logs; optional prefix overrides config",
    )
    parser.add_argument("--raw-log")
    parser.add_argument("--decoded-log")
    parser.add_argument("--combined-log")
    parser.add_argument("--jsonl")
    parser.add_argument(
        "--battery-log", nargs="?", const="", metavar="PREFIX",
        help="enable battery CSV; optional prefix overrides config",
    )
    parser.add_argument("--no-color", action="store_true")
    parser.add_argument("-q", "--quiet", action="store_true")
    parser.add_argument("--max-payload", type=int, default=255)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config, _config_path = load_config(args.config)

    infodump_requested = args.infodump
    if infodump_requested and args.dashboard:
        raise SystemExit("--infodump is a one-shot command and cannot be combined with --dashboard")
    if infodump_requested and args.mode == "passive":
        raise SystemExit("--infodump requires active read-only access; remove --mode passive")

    if args.list_ports:
        ports = discover_ports()
        if ports:
            print("\n".join(str(port) for port in ports))
            return 0
        print("No serial devices found.", file=sys.stderr)
        return 1

    selected_sources = sum(value is not None for value in (args.port, args.binary, args.hex_file))
    if selected_sources > 1:
        raise SystemExit("choose only one of --port, --binary, or --hex")

    mode = "inquisitor" if args.inquisitor or infodump_requested else (args.mode or "passive")
    if mode == "inquisitor" and (args.binary is not None or args.hex_file is not None):
        raise SystemExit("active read-only modes require a live serial port")

    baud = args.baud or int(config["serial"]["baud"])
    refresh_hz = args.refresh_rate or float(config["dashboard"]["refresh_hz"])
    if refresh_hz <= 0:
        raise SystemExit("dashboard refresh rate must be positive")

    logging_cfg = config["logging"]
    timestamp_enabled = bool(logging_cfg.get("timestamp", True))
    timestamp_format = str(logging_cfg.get("timestamp_format", "%Y%m%d-%H%M%S"))
    run_timestamp = datetime.now().strftime(timestamp_format)

    raw_log = args.raw_log or config_string(logging_cfg.get("raw_log"))
    decoded_log = args.decoded_log or config_string(logging_cfg.get("decoded_log"))
    combined_log = args.combined_log or config_string(logging_cfg.get("combined_log"))
    jsonl_log = args.jsonl or config_string(logging_cfg.get("jsonl"))
    battery_log: Path | None = None

    if args.all_logs is not None:
        base = _configured_prefix(
            args.all_logs,
            directory=logging_cfg.get("all_logs_directory", "comm-logs"),
            prefix=logging_cfg.get("all_logs_prefix", "inq"),
            timestamp_enabled=timestamp_enabled,
            timestamp_text=run_timestamp,
        )
        base = _numbered_base(base, (".raw.log", ".decoded.log", ".combined.log", ".jsonl"))
        raw_log = raw_log or str(_append_suffix(base, ".raw.log"))
        decoded_log = decoded_log or str(_append_suffix(base, ".decoded.log"))
        combined_log = combined_log or str(_append_suffix(base, ".combined.log"))
        jsonl_log = jsonl_log or str(_append_suffix(base, ".jsonl"))

    if args.battery_log is not None:
        base = _configured_prefix(
            args.battery_log,
            directory=logging_cfg.get("battery_log_directory", "battery-log"),
            prefix=logging_cfg.get("battery_log_prefix", "battery"),
            timestamp_enabled=timestamp_enabled,
            timestamp_text=run_timestamp,
        )
        base = _numbered_base(base, (".csv",))
        battery_log = _append_suffix(base, ".csv")
    else:
        configured_battery = config_string(logging_cfg.get("battery_log"))
        battery_log = Path(_unique_exact_path(configured_battery)) if configured_battery else None

    raw_log = _unique_exact_path(raw_log)
    decoded_log = _unique_exact_path(decoded_log)
    combined_log = _unique_exact_path(combined_log)
    jsonl_log = _unique_exact_path(jsonl_log)

    live_port = args.port
    if args.binary is None and args.hex_file is None and live_port is None:
        if not bool(config["serial"].get("auto_detect", True)):
            raise SystemExit("no serial port supplied and serial.auto_detect is disabled")
        live_port = choose_port_noninteractive()
        print(f"Auto-selected serial port: {live_port}", file=sys.stderr)

    colour = bool(config["dashboard"]["color"]) and not args.no_color and sys.stdout.isatty()
    logs = LogWriters(
        raw_log=raw_log, decoded_log=decoded_log, combined_log=combined_log, jsonl_log=jsonl_log,
    )
    battery_cfg = config["battery"]
    analyzer = BatteryAnalyzer(
        csv_path=battery_log,
        rest_current_max_a=float(battery_cfg["rest_current_max_a"]),
        rest_motor_power_max_w=float(battery_cfg["rest_motor_power_max_w"]),
        baseline_min_samples=int(battery_cfg["baseline_samples"]),
        baseline_window_samples=int(battery_cfg["baseline_window_samples"]),
        resistance_min_current_a=float(battery_cfg["resistance_min_current_a"]),
    )
    model = DashboardModel(config)

    poller: InquisitorPoller | None = None
    infodump_session: InfoDumpSession | None = None
    if mode == "inquisitor":
        inquisitor_cfg = config["inquisitor"]
    if infodump_requested:
        infodump_session = InfoDumpSession(
            response_timeout_s=args.response_timeout or float(inquisitor_cfg["response_timeout_s"]),
            inter_request_gap_s=float(inquisitor_cfg["inter_request_gap_s"]),
            retries=int(inquisitor_cfg["retries"]),
        )
    elif mode == "inquisitor":
        poller = InquisitorPoller(
            poll_interval_s=args.poll_interval or float(inquisitor_cfg["poll_interval_s"]),
            response_timeout_s=args.response_timeout or float(inquisitor_cfg["response_timeout_s"]),
            inter_request_gap_s=float(inquisitor_cfg["inter_request_gap_s"]),
            retries=int(inquisitor_cfg["retries"]),
            startup_identity=bool(inquisitor_cfg["startup_identity"]),
        )

    stop = [False]

    def stop_handler(_signum: int, _frame: object) -> None:
        stop[0] = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    port_label = str(live_port) if live_port is not None else str(args.binary or args.hex_file)
    dashboard = TerminalDashboard(
        model=model, analyzer=analyzer, mode=mode, port_label=port_label,
        baud=baud, logs=logs.active_names(battery_log), colour=colour,
        poller=poller,
    ) if args.dashboard else None

    start_ns: int | None = None
    previous_ns: int | None = None
    packet_no = 0

    def process(packet: Packet, label: str, *, tx: bool = False) -> None:
        nonlocal start_ns, previous_ns, packet_no
        if start_ns is None:
            start_ns = packet.timestamp_ns
        relative = (packet.timestamp_ns - start_ns) / 1_000_000_000
        delta_ms = 0.0 if previous_ns is None else (packet.timestamp_ns - previous_ns) / 1_000_000
        previous_ns = packet.timestamp_ns
        packet_no += 1
        decoded = decode_packet(packet)
        now = time.monotonic()
        model.observe_packet(packet, now, relative, tx=tx)
        if not tx:
            analyzer.update(decoded, relative, device_name(packet.src))
            model.update(decoded, relative, device_name(packet.src), analyzer)
            if infodump_session is not None:
                infodump_session.observe_packet(packet, now)
            if poller is not None:
                poller.observe(
                    src=packet.src,
                    dst=packet.dst,
                    cmd=packet.cmd,
                    index=packet.index,
                    payload_length=packet.length,
                    checksum_ok=packet.checksum_ok,
                    now=now,
                )
        logs.write(
            packet=packet, relative=relative, delta_ms=delta_ms, label=label,
            packet_no=packet_no, decoded=decoded,
        )
        if dashboard is None and infodump_session is None and not args.quiet:
            print_packet(packet, relative, delta_ms, label, decoded, colour)

    try:
        if dashboard is not None:
            dashboard.enter()

        if live_port is not None:
            configure_serial(live_port, baud)
            transport = SerialTransport(live_port, writable=mode == "inquisitor")
            framer = Framer(max_payload=args.max_payload)
            start_wall = time.monotonic()
            next_render = start_wall
            if poller is not None:
                poller.start(start_wall)

            def send_frame(frame: bytes) -> None:
                transport.write(frame)
                process(Packet(timestamp_ns=time.monotonic_ns(), raw=frame), "TX", tx=True)

            try:
                while not stop[0]:
                    now = time.monotonic()
                    timeout = min(0.03, max(0.0, next_render - now)) if dashboard is not None else 0.03
                    chunk = transport.read(timeout)
                    if chunk:
                        timestamp_ns = time.monotonic_ns()
                        for packet in framer.feed(chunk, timestamp_ns):
                            process(packet, args.label)
                    now = time.monotonic()
                    if infodump_session is not None:
                        infodump_session.tick(now, send_frame)
                    if poller is not None:
                        poller.tick(now, send_frame)
                    elapsed_s = now - start_wall
                    if model.tick(now, elapsed_s):
                        analyzer.on_bms_link_lost()
                        if poller is not None:
                            poller.mark_bms_link_lost()
                    if dashboard is not None and now >= next_render:
                        dashboard.render(now, elapsed_s)
                        next_render = now + 1.0 / refresh_hz
                    if infodump_session is not None and infodump_session.complete:
                        break
            finally:
                transport.close()
        else:
            source = binary_bytes(args.binary) if args.binary is not None else hex_bytes(args.hex_file)
            for packet in frame_stream(source, max_payload=args.max_payload):
                if stop[0]:
                    break
                process(packet, args.label)
            if dashboard is not None:
                now = time.monotonic()
                model.tick(now, 0.0)
                dashboard.render(now, 0.0)
    finally:
        if dashboard is not None:
            dashboard.exit()
        analyzer.close()
        logs.close()

    exit_code = 0
    if infodump_session is not None:
        if not infodump_session.complete:
            for request in InfoDumpSession.REQUESTS:
                if request.index not in infodump_session.blocks:
                    infodump_session.errors.setdefault(
                        request.index, "capture interrupted before a matching reply was received"
                    )
        snapshot = build_infodump_snapshot(
            infodump_session.blocks,
            infodump_session.errors,
            port=port_label,
            baud=baud,
            sent=infodump_session.sent,
            retries=infodump_session.retries_sent,
            unexpected_replies=infodump_session.unexpected_replies,
        )
        infodump_directory = logging_cfg.get("battery_log_directory") or "battery-log"
        requested_path = infodump_path(
            str(infodump_directory), run_timestamp, snapshot.get("battery_id")
        )
        output_path = _unique_exact_path(str(requested_path))
        assert output_path is not None
        write_infodump(snapshot, output_path)
        print(f"Infodump written to {output_path}", file=sys.stderr)
        if infodump_session.errors:
            exit_code = 2

    print(f"Captured {packet_no} complete packets.", file=sys.stderr)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
