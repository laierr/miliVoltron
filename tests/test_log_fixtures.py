"""Replay sanitized captures placed in tests/fixtures/logs."""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path
from statistics import median

from mili_voltron import Packet, binary_bytes, frame_stream, hex_bytes


FIXTURE_DIR = Path(__file__).parent / "fixtures" / "logs"
RAW_FIELD = re.compile(r"(?:^|\s)raw=([0-9a-fA-F]+)(?:\s|$)")


def packets_from_fixture(path: Path) -> list[Packet]:
    if path.name.endswith(".raw.log"):
        packets: list[Packet] = []
        with path.open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                fields = line.split(maxsplit=1)
                if len(fields) != 2:
                    raise AssertionError(f"{path}:{line_number} has no timestamp and packet data")
                try:
                    timestamp_ns = int(float(fields[0]) * 1_000_000_000)
                except ValueError as error:
                    raise AssertionError(f"{path}:{line_number} has an invalid timestamp") from error
                match = RAW_FIELD.search(fields[1])
                if match is None:
                    raise AssertionError(f"{path}:{line_number} has no raw hex field")
                packets.append(Packet(timestamp_ns=timestamp_ns, raw=bytes.fromhex(match.group(1))))
        return packets
    if path.suffix == ".bin":
        return list(frame_stream(binary_bytes(path)))
    if path.suffix == ".hex":
        return list(frame_stream(hex_bytes(path)))
    if path.suffix == ".jsonl":
        packets: list[Packet] = []
        with path.open(encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, 1):
                if not line.strip():
                    continue
                record = json.loads(line)
                raw_hex = record.get("raw_hex")
                if not isinstance(raw_hex, str):
                    raise AssertionError(f"{path}:{line_number} has no string raw_hex field")
                packets.append(Packet(timestamp_ns=0, raw=bytes.fromhex(raw_hex)))
        return packets
    raise AssertionError(f"unsupported fixture format: {path}")


class LogFixtureTests(unittest.TestCase):
    def test_available_captures_contain_packets(self) -> None:
        fixtures = sorted(
            path
            for path in FIXTURE_DIR.iterdir()
            if path.is_file()
            and (path.name.endswith(".raw.log") or path.suffix in {".bin", ".hex", ".jsonl"})
        )
        if not fixtures:
            self.skipTest(
                "add sanitized .raw.log, .bin, .hex, or .jsonl captures to tests/fixtures/logs"
            )

        for fixture in fixtures:
            with self.subTest(fixture=fixture.name):
                packets = packets_from_fixture(fixture)
                self.assertGreater(len(packets), 0, f"{fixture} contains no complete packets")
                for packet in packets:
                    self.assertEqual(len(packet.raw), 9 + packet.length)

    def test_observed_raw_logs_have_valid_checksums(self) -> None:
        captures = sorted(
            path
            for path in FIXTURE_DIR.glob("*.raw.log")
            if not path.name.startswith("synthetic-")
        )
        if not captures:
            self.skipTest("add a sanitized observed .raw.log capture")

        for capture in captures:
            with self.subTest(capture=capture.name):
                packets = packets_from_fixture(capture)
                bad_packets = [index for index, packet in enumerate(packets, 1) if not packet.checksum_ok]
                self.assertEqual(bad_packets, [], f"{capture} contains checksum failures")

    def test_raw_log_timestamps_are_monotonic(self) -> None:
        captures = sorted(FIXTURE_DIR.glob("*.raw.log"))
        for capture in captures:
            with self.subTest(capture=capture.name):
                packets = packets_from_fixture(capture)
                timestamps = [packet.timestamp_ns for packet in packets]
                self.assertEqual(timestamps, sorted(timestamps))

    def test_boot_capture_cadence(self) -> None:
        fixture = FIXTURE_DIR / "boot.raw.log"
        if not fixture.exists():
            self.skipTest("add sanitized boot.raw.log capture")

        packets = packets_from_fixture(fixture)
        expected_cadence = (
            ("ECU heartbeat", (0x20, 0x3D, 0x55, 0x00), 0.08, 0.12),
            ("BMS status", (0x22, 0x3D, 0x04, 0x30), 2.5, 3.5),
            ("BMS cells", (0x22, 0x3D, 0x04, 0x40), 2.5, 3.5),
            ("BMS temperatures", (0x22, 0x3D, 0x04, 0x51), 2.5, 3.5),
        )

        for name, signature, minimum_s, maximum_s in expected_cadence:
            timestamps = [
                packet.timestamp_ns / 1_000_000_000
                for packet in packets
                if (packet.src, packet.dst, packet.cmd, packet.index) == signature
            ]
            with self.subTest(stream=name):
                self.assertGreaterEqual(len(timestamps), 5)
                deltas = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])]
                observed_median = median(deltas)
                self.assertGreaterEqual(observed_median, minimum_s)
                self.assertLessEqual(observed_median, maximum_s)

    def test_synthetic_checksum_fixture_contains_good_and_bad_frames(self) -> None:
        fixture = FIXTURE_DIR / "synthetic-checksum-errors.raw.log"
        packets = packets_from_fixture(fixture)

        self.assertTrue(any(packet.checksum_ok for packet in packets))
        self.assertTrue(any(not packet.checksum_ok for packet in packets))


if __name__ == "__main__":
    unittest.main()
