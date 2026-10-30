"""Offline receive-parser tests; no UART device or transmitter is used."""
import unittest

from elrs_monitor import CrsfParser, Monitor, crc8, decode_channels, decode_link


def fixture(kind, payload):
    body = bytes([kind]) + payload
    return bytes([0xC8, len(body) + 1]) + body + bytes([crc8(body)])


class ParserTests(unittest.TestCase):
    def test_crc_standard_check_value(self):
        self.assertEqual(crc8(b"123456789"), 0xBC)

    def test_all_bits_and_channel_boundaries(self):
        self.assertEqual(decode_channels(bytes(22)), [0] * 16)
        self.assertEqual(decode_channels(bytes([255]) * 22), [2047] * 16)
        channels = [0, 1, 172, 992, 1811, 2047, 512, 1024] * 2
        # Assemble individual bits, independently of decoder integer offsets.
        bits = [bit for channel in channels for bit in [(channel >> j) & 1 for j in range(11)]]
        payload = bytes(sum(bits[i+j] << j for j in range(8)) for i in range(0, 176, 8))
        self.assertEqual(decode_channels(payload), channels)

    def test_fragmentation_at_every_boundary(self):
        frame = fixture(0x16, bytes([255]) * 22)
        for split in range(1, len(frame)):
            parser = CrsfParser()
            self.assertEqual(parser.feed(frame[:split]), [])
            self.assertEqual(parser.feed(frame[split:]), [(0x16, bytes([255])*22)])

    def test_noise_crc_and_multiple_frames(self):
        frame = fixture(0x16, bytes(22))
        bad = frame[:-1] + bytes([frame[-1] ^ 0xFF])
        parser = CrsfParser()
        output = parser.feed(b"noise" + bad + frame + frame)
        self.assertEqual(len(output), 2)
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(parser.bytes_received, 5 + 3 * len(frame))
        self.assertEqual(parser.buffer, b"")

    def test_bogus_long_header_does_not_hide_good_frame(self):
        parser = CrsfParser()
        self.assertEqual(parser.feed(b"\xc8\x3e\x55" + fixture(0x16, bytes(22))),
                         [(0x16, bytes(22))])

    def test_bad_lengths_and_payload(self):
        parser = CrsfParser()
        self.assertEqual(parser.feed(b"\xc8\xff" + fixture(0x16, b"x")), [])
        self.assertEqual(parser.invalid_lengths, 1)
        self.assertEqual(parser.malformed_payloads, 1)
        with self.assertRaises(ValueError):
            decode_channels(b"x")

    def test_telemetry_does_not_keep_rc_alive(self):
        monitor = Monitor()
        self.assertEqual(monitor.snapshot(0, .5, 0)["status"], "WAITING")
        monitor.feed(fixture(0x16, bytes(22)), 1)
        self.assertEqual(monitor.snapshot(1.1, .5, 10)["status"], "RC_STREAM")
        monitor.feed(fixture(0x14, bytes([60, 65, 98, 254, 0, 1, 2, 70, 90, 255])), 2)
        state = monitor.snapshot(2, .5, 0)
        self.assertEqual(state["status"], "STALE")
        self.assertIsNone(state["raw_channels"])
        self.assertEqual(state["link"]["uplink_snr_db"], -2)
        self.assertEqual(state["rc_frames"], 1)
        monitor.feed(fixture(0x16, bytes([255])*22), 3)
        self.assertEqual(monitor.snapshot(3, .5, 10)["raw_channels"], [2047]*16)

    def test_link_signed_values(self):
        link = decode_link(bytes([60, 70, 100, 255, 1, 2, 3, 80, 90, 128]))
        self.assertEqual(link["uplink_rssi1_dbm"], -60)
        self.assertEqual(link["downlink_snr_db"], -128)


if __name__ == "__main__":
    unittest.main()
