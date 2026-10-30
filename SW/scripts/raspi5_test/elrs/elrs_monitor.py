#!/usr/bin/env python3
"""Receive-only UART CRSF monitor. Never sends frames or controls an FC."""
import argparse
import json
import sys
import time
from collections import Counter
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path


def crc8(data):
    """CRC-8/DVB-S2, over CRSF type + payload."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (0xD5 if crc & 0x80 else 0)) & 0xFF
    return crc


class CrsfParser:
    """Streaming parser for receiver->FC sync 0xC8. Bounded partial-frame buffer."""
    def __init__(self):
        self.buffer = bytearray()
        self.bytes_received = self.valid_frames = self.crc_errors = 0
        self.discarded_bytes = self.invalid_lengths = self.malformed_payloads = 0
        self.types = Counter()

    def feed(self, data):
        self.bytes_received += len(data)
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0xC8:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            length = self.buffer[1]  # type + payload + CRC (excludes address/length)
            if not 2 <= length <= 62:
                del self.buffer[0]
                self.invalid_lengths += 1
                continue
            total = length + 2
            if len(self.buffer) < total:
                # A noise header can claim an incomplete long frame. Prefer a later
                # complete CRC-valid frame, otherwise retain this partial frame.
                recovered = False
                for offset in range(1, len(self.buffer) - 3):
                    size = self.buffer[offset + 1]
                    end = offset + size + 2
                    if (self.buffer[offset] == 0xC8 and 2 <= size <= 62
                            and end <= len(self.buffer)
                            and crc8(self.buffer[offset+2:end-1]) == self.buffer[end-1]):
                        self.discarded_bytes += offset
                        del self.buffer[:offset]
                        recovered = True
                        break
                if recovered:
                    continue
                break
            if crc8(self.buffer[2:total-1]) != self.buffer[total-1]:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            packet_type = self.buffer[2]
            payload = bytes(self.buffer[3:total-1])
            del self.buffer[:total]
            self.valid_frames += 1
            self.types[packet_type] += 1
            if ((packet_type == 0x16 and len(payload) != 22)
                    or (packet_type == 0x14 and len(payload) != 10)):
                self.malformed_payloads += 1
                continue
            result.append((packet_type, payload))
        return result


def decode_channels(payload):
    if len(payload) != 22:
        raise ValueError("RC_CHANNELS_PACKED requires 22 payload bytes")
    packed = int.from_bytes(payload, "little")
    return [(packed >> (11 * i)) & 0x7FF for i in range(16)]


def decode_link(payload):
    if len(payload) != 10:
        raise ValueError("LINK_STATISTICS requires 10 payload bytes")
    signed = lambda value: value if value < 128 else value - 256
    return {"uplink_rssi1_dbm": -payload[0], "uplink_rssi2_dbm": -payload[1],
            "uplink_lq_percent": payload[2], "uplink_snr_db": signed(payload[3]),
            "active_antenna": payload[4], "rf_mode": payload[5],
            "tx_power_code": payload[6], "downlink_rssi_dbm": -payload[7],
            "downlink_lq_percent": payload[8], "downlink_snr_db": signed(payload[9])}


class Monitor:
    def __init__(self):
        self.parser = CrsfParser()
        self.channels = self.link = None
        self.last_rc = self.last_link = None
        self.rc_frames = 0

    def feed(self, data, now):
        for kind, payload in self.parser.feed(data):
            if kind == 0x16:
                self.channels = decode_channels(payload)
                self.last_rc = now
                self.rc_frames += 1
            elif kind == 0x14:
                self.link = decode_link(payload)
                self.last_link = now

    def snapshot(self, now, stale_seconds, rc_hz):
        age = None if self.last_rc is None else now - self.last_rc
        status = "WAITING" if age is None else "RC_STREAM" if age <= stale_seconds else "STALE"
        # Do not present old sticks as current input when channel frames stop.
        current = self.channels if status == "RC_STREAM" else None
        return {"utc": datetime.now(timezone.utc).isoformat(), "status": status,
                "rc_age_ms": None if age is None else round(age * 1000, 1),
                "rc_hz": round(rc_hz, 1), "rc_frames": self.rc_frames,
                "raw_channels": current,
                "approx_us": None if current is None else [round(1500 + (x-992)*0.625, 1) for x in current],
                "link": self.link,
                "link_age_ms": None if self.last_link is None else round((now-self.last_link)*1000, 1),
                "rx_bytes": self.parser.bytes_received, "valid_frames": self.parser.valid_frames,
                "crc_errors": self.parser.crc_errors,
                "invalid_lengths": self.parser.invalid_lengths,
                "malformed_payloads": self.parser.malformed_payloads,
                "discarded_bytes": self.parser.discarded_bytes,
                "frame_types": {f"0x{k:02X}": v for k, v in sorted(self.parser.types.items())}}


def render(snapshot, port, baud):
    age = snapshot["rc_age_ms"]
    lines = [f"ELRS RX MONITOR  {port} @ {baud}  [receive only]",
             f"{snapshot['status']} | RC age: {age if age is not None else '--'} ms"
             f" | RC frames/s: {snapshot['rc_hz']}",
             f"bytes={snapshot['rx_bytes']} valid={snapshot['valid_frames']}"
             f" CRC errors={snapshot['crc_errors']} malformed={snapshot['malformed_payloads']}",
             "CH     RAW     approx us (display only; not PWM output)"]
    for i in range(16):
        if snapshot["raw_channels"] is None:
            lines.append(f"CH{i+1:02d}    ----       ----")
        else:
            lines.append(f"CH{i+1:02d}    {snapshot['raw_channels'][i]:4d}       {snapshot['approx_us'][i]:6.1f}")
    lines.append(f"Link statistics: {snapshot['link']} | age={snapshot['link_age_ms']} ms")
    lines.append("RC_STREAM means UART channel frames are arriving, not proof of a live RF link.")
    lines.append("Channels are unnamed: verify transmitter channel order and switches yourself. Ctrl+C exits.")
    print("\033[H\033[2J" + "\n".join(lines), end="\n", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=420000)
    parser.add_argument("--refresh", type=float, default=5, help="Display/log updates per second")
    parser.add_argument("--stale-ms", type=float, default=500, help="Local display threshold, not FC failsafe")
    parser.add_argument("--seconds", type=float, default=0, help="0 = until Ctrl+C")
    parser.add_argument("--json", action="store_true", help="JSONL instead of terminal table")
    parser.add_argument("--log", type=Path, help="Append display snapshots as JSONL (not every frame)")
    args = parser.parse_args()
    if min(args.baud, args.refresh, args.stale_ms) <= 0 or args.seconds < 0:
        parser.error("baud/refresh/stale-ms must be positive; seconds must be >= 0")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("Install pyserial: sudo apt install python3-serial") from exc
    if args.log:
        args.log.parent.mkdir(parents=True, exist_ok=True)
    monitor = Monitor()
    # Configure before opening: no RTS/CTS, no DTR, no software flow control.
    # pyserial uses O_RDWR, but this program has no write path. Leave Pi TX unconnected.
    with serial.Serial(port=None, baudrate=args.baud, timeout=0.02,
                       xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True) as uart:
        uart.dtr = False
        uart.rts = False
        uart.port = args.port
        uart.open()
        with (args.log.open("a", buffering=1) if args.log else nullcontext(None)) as logfile:
            start = previous_report = time.monotonic()
            previous_count = 0
            next_report = start
            try:
                while args.seconds == 0 or time.monotonic() - start < args.seconds:
                    data = uart.read(min(4096, max(1, uart.in_waiting)))
                    now = time.monotonic()
                    if data:
                        monitor.feed(data, now)
                    if now >= next_report:
                        hz = (monitor.rc_frames-previous_count) / max(now-previous_report, 1e-9)
                        snapshot = monitor.snapshot(now, args.stale_ms / 1000, hz)
                        line = json.dumps(snapshot)
                        if args.json or not sys.stdout.isatty():
                            print(line, flush=True)
                        else:
                            render(snapshot, args.port, args.baud)
                        if logfile:
                            logfile.write(line + "\n")
                        previous_count, previous_report = monitor.rc_frames, now
                        next_report = now + 1 / args.refresh
            except KeyboardInterrupt:
                pass
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
