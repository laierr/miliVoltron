"""Fast, hardware-independent tests for core protocol behavior."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from mili_voltron_battery import BATTERY_LOG_FIELDS, BatteryAnalyzer
from mili_voltron import Framer, Packet, decode_packet
from mili_voltron_polling import InquisitorPoller, build_packet, build_read_request


class PacketTests(unittest.TestCase):
    def test_build_packet_has_valid_checksum(self) -> None:
        raw = build_packet(src=0x3D, dst=0x22, cmd=0x01, index=0x30, payload=b"\x0c")
        packet = Packet(timestamp_ns=0, raw=raw)

        self.assertTrue(packet.checksum_ok)
        self.assertEqual(packet.payload, b"\x0c")

    def test_read_request_uses_read_only_command(self) -> None:
        packet = Packet(timestamp_ns=0, raw=build_read_request(0x22, 0x40, 20))

        self.assertEqual(packet.src, 0x3D)
        self.assertEqual(packet.dst, 0x22)
        self.assertEqual(packet.cmd, 0x01)
        self.assertEqual(packet.index, 0x40)
        self.assertEqual(decode_packet(packet), {"kind": "read_request", "requested_bytes": 20})

    def test_read_request_rejects_zero_and_more_than_64_bytes(self) -> None:
        with self.assertRaises(ValueError):
            build_read_request(0x22, 0x30, 0)
        with self.assertRaises(ValueError):
            build_read_request(0x22, 0x30, 65)

    def test_broad_identity_reply_decodes_capacity_values(self) -> None:
        payload = (
            b"BPECV22AAB1001"
            + bytes.fromhex("52 06 FC 6C D8 72 42 0E 01 00")
        )
        packet = Packet(
            timestamp_ns=0,
            raw=build_packet(0x22, 0x3D, 0x04, 0x10, payload),
        )

        decoded = decode_packet(packet)

        self.assertEqual(decoded["serial"], "BPECV22AAB1001")
        self.assertEqual(decoded["firmware_version"], "6.5.2")
        self.assertEqual(decoded["design_full_capacity_reported_mah"], 27900)
        self.assertEqual(decoded["current_full_capacity_reported_mah"], 29400)
        self.assertEqual(decoded["register_1a_raw"], 0x0E42)

    def test_broad_telemetry_reply_decodes_soc_capacities_and_cells(self) -> None:
        payload = bytes.fromhex(
            "01 00 0B 5D 55 00 00 00 91 0F 27 27 00 00 00 00 "
            "00 00 7C 5D 4F 5B 00 00 00 00 00 00 FF FF 00 00 "
            "85 0F 8D 0F 8C 0F 8F 0F 8C 0F 97 0F 9A 0F 98 0F "
            "9A 0F 99 0F FF FF FF FF FF FF FF FF FF FF FF FF"
        )
        packet = Packet(
            timestamp_ns=0,
            raw=build_packet(0x22, 0x3D, 0x04, 0x30, payload),
        )

        decoded = decode_packet(packet)

        self.assertEqual(decoded["kind"], "bms_telemetry")
        self.assertEqual(decoded["coulomb_capacity_reported_mah"], 23932)
        self.assertEqual(decoded["voltage_capacity_reported_mah"], 23375)
        self.assertEqual(decoded["bms_soc_reported_raw"], 0xFFFF)
        self.assertIsNone(decoded["bms_soc_reported_percent"])
        self.assertEqual(decoded["temperatures_12_c"], [19, 19])
        self.assertEqual(len(decoded["cells_mv"]), 10)

    def test_temperature_reply_preserves_sensor_order_and_missing_values(self) -> None:
        packet = Packet(
            timestamp_ns=0,
            raw=build_packet(
                0x22,
                0x3D,
                0x04,
                0x51,
                bytes.fromhex("07 00 31 31 32 00"),
            ),
        )

        decoded = decode_packet(packet)

        self.assertEqual(decoded["temperatures_34_56_c"], [29, 29, None, 30])


class FramerTests(unittest.TestCase):
    def test_fragmented_frame_is_reassembled(self) -> None:
        raw = build_read_request(0x22, 0x30, 12)
        framer = Framer()

        self.assertEqual(framer.feed(raw[:3], 100), [])
        packets = framer.feed(raw[3:], 200)

        self.assertEqual(len(packets), 1)
        self.assertEqual(packets[0].raw, raw)
        self.assertEqual(packets[0].timestamp_ns, 100)

    def test_noise_before_start_marker_is_ignored(self) -> None:
        raw = build_read_request(0x22, 0x51, 6)
        packets = Framer().feed(b"\x00\xff\x5a\x00" + raw, 100)

        self.assertEqual([packet.raw for packet in packets], [raw])


class PollerTests(unittest.TestCase):
    def test_poll_windows_use_confirmed_broad_reads(self) -> None:
        self.assertEqual(
            [(request.index, request.length) for request in InquisitorPoller.IDENTITY_BMS],
            [(0x10, 24)],
        )
        self.assertEqual(
            [(request.index, request.length) for request in InquisitorPoller.PERIODIC_BMS],
            [(0x30, 64), (0x51, 6)],
        )

    def test_timeout_retries_once_then_advances(self) -> None:
        poller = InquisitorPoller(
            response_timeout_s=0.1,
            inter_request_gap_s=0.0,
            retries=1,
            startup_identity=False,
        )
        sent: list[bytes] = []

        poller.tick(0.0, sent.append)
        self.assertEqual(len(sent), 1)

        poller.tick(0.1, sent.append)
        self.assertEqual(len(sent), 2)

        poller.tick(0.2, sent.append)
        self.assertEqual(poller.stats["timeouts"], 1)
        self.assertIsNone(poller.pending)

    def test_reply_requires_valid_checksum_and_exact_payload_length(self) -> None:
        poller = InquisitorPoller(startup_identity=True)
        poller.tick(0.0, lambda _frame: None)

        common = {
            "src": 0x22,
            "dst": 0x3D,
            "cmd": 0x04,
            "index": 0x10,
            "now": 0.1,
        }
        self.assertFalse(
            poller.observe(payload_length=24, checksum_ok=False, **common)
        )
        self.assertFalse(
            poller.observe(payload_length=16, checksum_ok=True, **common)
        )
        self.assertIsNotNone(poller.pending)
        self.assertTrue(
            poller.observe(payload_length=24, checksum_ok=True, **common)
        )
        self.assertIsNone(poller.pending)


class BatteryAnalyzerTests(unittest.TestCase):
    def test_csv_fields_distinguish_reported_and_derived_values(self) -> None:
        self.assertIn("bms_soc_reported_percent", BATTERY_LOG_FIELDS)
        self.assertIn("bms_voltage_soc_derived_percent", BATTERY_LOG_FIELDS)
        self.assertIn(
            "bms_soc_estimate_delta_derived_percentage_points",
            BATTERY_LOG_FIELDS,
        )
        self.assertIn("bms_register_3b_reported_raw", BATTERY_LOG_FIELDS)
        self.assertNotIn("health_percent", BATTERY_LOG_FIELDS)

    def test_soc_estimates_use_design_capacity_and_report_percentage_point_delta(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        analyzer = BatteryAnalyzer(Path(directory.name) / "battery.csv")
        self.addCleanup(analyzer.close)
        analyzer.update(
            {
                "kind": "serial",
                "serial": "BPECV22AAB1001",
                "design_full_capacity_reported_mah": 27900,
                "current_full_capacity_reported_mah": 29400,
            },
            0.0,
            "BMS",
        )
        analyzer.update(
            {
                "kind": "bms_telemetry",
                "voltage_v": 39.85,
                "current_a": 0.0,
                "temperatures_12_c": [19, 19],
                "cells_mv": [3973, 3981, 3980, 3983, 3980, 3991, 3994, 3992, 3994, 3993],
                "bms_soc_reported_raw": 0xFFFF,
                "bms_soc_reported_percent": None,
                "coulomb_capacity_reported_mah": 23932,
                "voltage_capacity_reported_mah": 23375,
                "register_3b_raw": 98,
            },
            1.0,
            "BMS",
        )
        analyzer.update(
            {
                "kind": "pcb_temperature_block",
                "temperatures_34_56_c": [19, 19, 19, 20],
            },
            1.01,
            "BMS",
        )

        sample = analyzer.latest_sample
        self.assertIsNotNone(sample)
        analytics = analyzer.latest_analytics
        self.assertAlmostEqual(
            float(analytics["coulomb_soc_derived_percent"]),
            23932 / 27900 * 100,
        )
        self.assertAlmostEqual(
            float(analytics["voltage_soc_derived_percent"]),
            23375 / 27900 * 100,
        )
        self.assertAlmostEqual(
            float(analytics["soc_estimate_delta_derived_percentage_points"]),
            (23375 - 23932) / 27900 * 100,
        )


if __name__ == "__main__":
    unittest.main()
