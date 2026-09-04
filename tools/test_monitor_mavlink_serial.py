#!/usr/bin/env python
import struct
import unittest

from monitor_mavlink_serial import (
    FIRMWARE_VARIANTS,
    MAVLINK_CRC_EXTRA,
    MavlinkV1Parser,
    decode_frame,
    parse_pmw_burst_statistics_text,
    parse_pmw_config_text,
    parse_pmw_diagnostic_text,
    parse_pmw_direct_statistics_text,
    parse_pmw_direct_text,
    x25_crc,
)


def make_frame(msgid, payload, seq=1, sysid=1, compid=158):
    header = bytes((len(payload), seq, sysid, compid, msgid))
    crc = x25_crc(header + payload + bytes((MAVLINK_CRC_EXTRA[msgid],)))
    return b"\xfe" + header + payload + struct.pack("<H", crc)


class MavlinkMonitorTest(unittest.TestCase):
    def test_v4_firmware_variants_have_distinct_hashes(self):
        self.assertEqual(
            FIRMWARE_VARIANTS["v4-burst"],
            "8aeb7d27cfcecd791bf56710d7b755c638a7e56c4c890f6ac3f7d11d3cca695f",
        )
        self.assertEqual(
            FIRMWARE_VARIANTS["v4-direct"],
            "b2ac668b9e09182fcc1133a6acf500881c50c124bf3c144d10a72105745a0053",
        )
        self.assertNotEqual(FIRMWARE_VARIANTS["v4-burst"], FIRMWARE_VARIANTS["v4-direct"])

    def test_parses_v3_cross_diagnostics(self):
        direct = parse_pmw_direct_text(
            "D S0 M80 O01 X-32768 Y32767 Q255 HFFFF E12"
        )
        stats = parse_pmw_burst_statistics_text("R U11 A22 I3 C999 V7 Q8")
        direct_stats = parse_pmw_direct_statistics_text("T C999 V6 Q7")
        config = parse_pmw_config_text("C S0 D11 A67 F48 N0")

        self.assertEqual(direct["raw_x"], -32768)
        self.assertEqual(direct["raw_y"], 32767)
        self.assertEqual(direct["motion"], 0x80)
        self.assertEqual(direct["shutter"], 0xFFFF)
        self.assertEqual(direct["spi_errors"], 12)
        self.assertEqual(stats["raw_sum"], 11)
        self.assertEqual(stats["raw_max"], 22)
        self.assertEqual(stats["raw_min"], 3)
        self.assertEqual(stats["sample_count"], 999)
        self.assertEqual(direct_stats["sample_count"], 999)
        self.assertEqual(direct_stats["motion_count"], 6)
        self.assertEqual(direct_stats["squal_nonzero_count"], 7)
        self.assertEqual(config["bank0_4d"], 0x11)
        self.assertEqual(config["bank14_65"], 0x67)
        self.assertEqual(config["bank15_48"], 0x48)
        self.assertEqual(config["mismatches"], 0)

    def test_parses_compact_pmw_raw_diagnostic(self):
        decoded = parse_pmw_diagnostic_text(
            "P S3 M80 O01 X-32768 Y32767 Q255 HFFFF E12 R34"
        )

        self.assertEqual(decoded["status"], 3)
        self.assertEqual(decoded["motion"], 0x80)
        self.assertEqual(decoded["observation"], 0x01)
        self.assertEqual(decoded["raw_x"], -32768)
        self.assertEqual(decoded["raw_y"], 32767)
        self.assertEqual(decoded["squal"], 255)
        self.assertEqual(decoded["shutter"], 0xFFFF)
        self.assertEqual(decoded["spi_errors"], 12)
        self.assertEqual(decoded["rejects"], 34)

    def test_decodes_statustext_across_read_boundaries(self):
        payload = bytes((6,)) + b"PMW ID=49 INV=B6 RET=0".ljust(50, b"\0")
        raw = make_frame(253, payload)
        parser = MavlinkV1Parser()

        self.assertEqual(parser.feed(raw[:7]), [])
        frames = parser.feed(raw[7:])

        self.assertEqual(len(frames), 1)
        decoded = decode_frame(frames[0])
        self.assertEqual(decoded["name"], "STATUSTEXT")
        self.assertEqual(decoded["severity"], 6)
        self.assertEqual(decoded["text"], "PMW ID=49 INV=B6 RET=0")

    def test_decodes_optical_flow_signed_axes_and_quality(self):
        payload = struct.pack("<QfffhhBB", 123000, 0.0, 0.0, -1.0, -42, 73, 0, 88)
        parser = MavlinkV1Parser()
        frames = parser.feed(make_frame(100, payload))

        decoded = decode_frame(frames[0])

        self.assertEqual(decoded["name"], "OPTICAL_FLOW")
        self.assertEqual(decoded["flow_x"], -42)
        self.assertEqual(decoded["flow_y"], 73)
        self.assertEqual(decoded["quality"], 88)
        self.assertEqual(decoded["time_usec"], 123000)

    def test_rejects_bad_crc_and_resynchronizes(self):
        bad = bytearray(make_frame(0, struct.pack("<IBBBBB", 0, 18, 8, 0, 4, 3)))
        bad[-1] ^= 0x55
        good = make_frame(0, struct.pack("<IBBBBB", 0, 18, 8, 0, 4, 3), seq=2)
        parser = MavlinkV1Parser()

        frames = parser.feed(bytes(bad) + good)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 2)
        self.assertEqual(parser.crc_errors, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
