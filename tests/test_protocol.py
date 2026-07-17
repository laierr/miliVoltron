"""Fast, hardware-independent tests for core protocol behavior."""

from __future__ import annotations

import unittest

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


if __name__ == "__main__":
    unittest.main()
