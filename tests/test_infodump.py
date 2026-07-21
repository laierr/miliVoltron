"""Hardware-independent tests for the one-shot BMS information dump."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from mili_voltron import Packet
from mili_voltron_infodump import (
    InfoDumpSession,
    build_infodump_snapshot,
    infodump_path,
    write_infodump,
)
from mili_voltron_polling import build_packet


def sample_blocks() -> dict[int, bytes]:
    identity = bytearray(40)
    identity[:14] = b"BPECV22AAB1001"
    identity[14:24] = bytes.fromhex("52 06 FC 6C D8 72 42 0E 01 00")

    telemetry = bytearray(52)
    telemetry[:32] = bytes.fromhex(
        "C1 00 0B 5D 55 00 4C FE FB 0F 28 29 00 00 00 00 "
        "00 00 7C 5D 4F 5B 62 00 00 00 00 00 61 00 00 00"
    )
    cells = [4077, 4079, 4078, 4080, 4076, 4081, 4082, 4080, 4081, 4079]
    for offset, value in enumerate(cells, start=16):
        telemetry[offset * 2:offset * 2 + 2] = value.to_bytes(2, "little")

    extended = bytes.fromhex("01 00 07 00 31 31 32 00")
    deep = bytes.fromhex(
        "01 02 03 04 05 06 07 08 09 0A 0B 0C "
        "00 00 00 00 00 00 00 00 00 00 FF FF"
    )
    return {0x10: bytes(identity), 0x30: bytes(telemetry), 0x50: extended, 0x70: deep}


class InfoDumpSessionTests(unittest.TestCase):
    def test_session_reads_four_broad_windows(self) -> None:
        session = InfoDumpSession(response_timeout_s=0.2, inter_request_gap_s=0.0)
        sent: list[bytes] = []
        now = 0.0

        for request in InfoDumpSession.REQUESTS:
            session.tick(now, sent.append)
            outgoing = Packet(timestamp_ns=0, raw=sent[-1])
            self.assertEqual((outgoing.index, outgoing.payload[0]), (request.index, request.length))
            reply = Packet(
                timestamp_ns=0,
                raw=build_packet(0x22, 0x3D, 0x04, request.index, bytes(request.length)),
            )
            self.assertTrue(session.observe_packet(reply, now + 0.01))
            now += 0.02

        self.assertTrue(session.complete)
        self.assertEqual(len(session.blocks), 4)
        self.assertEqual(session.errors, {})

    def test_timeout_is_recorded_and_later_windows_continue(self) -> None:
        session = InfoDumpSession(
            response_timeout_s=0.1,
            inter_request_gap_s=0.0,
            retries=0,
        )
        sent: list[bytes] = []
        session.tick(0.0, sent.append)
        session.tick(0.1, sent.append)

        self.assertIn(0x10, session.errors)
        self.assertEqual(Packet(timestamp_ns=0, raw=sent[-1]).index, 0x30)


class InfoDumpExportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.snapshot = build_infodump_snapshot(
            sample_blocks(),
            {},
            port="/dev/ttyUSB0",
            baud=115200,
            sent=4,
            retries=0,
            unexpected_replies=0,
        )

    def test_snapshot_preserves_raw_and_decoded_values(self) -> None:
        registers = {row["offset"]: row for row in self.snapshot["registers"]}

        self.assertEqual(self.snapshot["battery_id"], "BPECV22AAB1001")
        self.assertEqual(self.snapshot["summary"]["serial"], "BPECV22AAB1001")
        self.assertEqual(self.snapshot["summary"]["firmware"], "6.5.2")
        self.assertEqual(registers["0x33"]["hex"], "0xFE4C")
        self.assertEqual(registers["0x33"]["bin"], "0b1111111001001100")
        self.assertEqual(registers["0x33"]["decoded"], -4.36)
        self.assertEqual(registers["0x33"]["units"], "A")
        self.assertEqual(registers["0x35"]["decoded"], {"T1": 21, "T2": 20})
        self.assertEqual(registers["0x3E"]["decoded"], 97)
        self.assertEqual(self.snapshot["summary"]["cell_delta_mv"], 6)

    def test_register_schema_and_complete_bitflag_definitions(self) -> None:
        registers = {row["offset"]: row for row in self.snapshot["registers"]}
        normal_fields = {
            "offset", "var_name", "interpreted_name", "decoded", "hex", "bin", "units"
        }
        self.assertEqual(set(registers["0x33"]), normal_fields)
        self.assertEqual(set(registers["0x10-0x16"]), normal_fields)

        flags = registers["0x30"]
        self.assertEqual(set(flags), normal_fields | {"bitflags"})
        self.assertEqual(flags["decoded"], ["PASSWORD", "CHARGE", "CHARGERIN"])
        definitions = {row["name"]: row for row in flags["bitflags"]}
        self.assertTrue(definitions["PASSWORD"]["active"])
        self.assertFalse(definitions["DISCHARGE"]["active"])
        self.assertTrue(definitions["CHARGE"]["active"])
        self.assertEqual(definitions["CHARGE"]["mask"], "0x0040")

    def test_unknown_active_bit_is_preserved(self) -> None:
        blocks = sample_blocks()
        telemetry = bytearray(blocks[0x30])
        telemetry[0:2] = (0x20C1).to_bytes(2, "little")
        blocks[0x30] = bytes(telemetry)
        snapshot = build_infodump_snapshot(
            blocks,
            {},
            port="/dev/ttyUSB0",
            baud=115200,
            sent=4,
            retries=0,
            unexpected_replies=0,
        )
        flags = next(row for row in snapshot["registers"] if row["offset"] == "0x30")
        self.assertIn("BIT_13_UNKNOWN", flags["decoded"])
        unknown = next(row for row in flags["bitflags"] if row["bit"] == 13)
        self.assertEqual(
            unknown,
            {"bit": 13, "mask": "0x2000", "name": None, "active": True, "known": False},
        )

    def test_json_writer_uses_v2_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            json_path = Path(directory) / "dump.json"
            write_infodump(self.snapshot, json_path)

            parsed_json = json.loads(json_path.read_text(encoding="utf-8"))
            self.assertEqual(parsed_json["schema"], "mili-voltron-infodump/v2")
            self.assertEqual(parsed_json["battery_id"], "BPECV22AAB1001")

    def test_default_filename_uses_timestamp_and_battery_id(self) -> None:
        self.assertEqual(
            infodump_path("battery-log", "20260721-171530", "BPECV22AAB1001"),
            Path("battery-log/20260721-171530-BPECV22AAB1001.json"),
        )
        self.assertEqual(
            infodump_path("battery-log", "20260721-171530", None),
            Path("battery-log/20260721-171530-unknown-battery.json"),
        )

    def test_capacity_failure_sentinel_remains_signed(self) -> None:
        blocks = sample_blocks()
        telemetry = bytearray(blocks[0x30])
        telemetry[18:20] = bytes.fromhex("F4 FF")
        blocks[0x30] = bytes(telemetry)
        snapshot = build_infodump_snapshot(
            blocks,
            {},
            port="/dev/ttyUSB0",
            baud=115200,
            sent=4,
            retries=0,
            unexpected_replies=0,
        )
        register = next(row for row in snapshot["registers"] if row["offset"] == "0x39")
        self.assertEqual(register["decoded"], -12)
        self.assertEqual(register["hex"], "0xFFF4")


if __name__ == "__main__":
    unittest.main()
