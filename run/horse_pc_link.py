"""HorseLink PC V1 framing and session client for the H743 USART3 test.

Adapted from the existing rc2026_deom F429 UART client. The two boards
share the protocol. This client only exposes TROT, status, and stand.
"""
from __future__ import annotations

import binascii
import secrets
import time

STATES = ("LOCKED", "WAIT_ONLINE", "CONFIGURE", "SET_STAND", "READY", "ENABLE",
          "RISING", "STAND_SETTLE", "RUN", "STOPPING", "CLEAR_FAULT", "FAULT", "JUMPING", "LYING")
GAITS = ("trot", "walk4", "reference")
PHASES = ("IDLE", "STARTING", "WALKING", "STOPPED", "PREEMPTED", "TIMEOUT", "FAULT", "DENIED", "STOPPING")
RESULTS = ("OK", "PENDING", "REJECTED", "STALE", "BAD_COMMAND")


class LinkError(RuntimeError):
    pass


def encode(body: bytes) -> bytes:
    if not 1 <= len(body) <= 26 or any(c < 32 or c > 126 or c == 42 for c in body):
        raise ValueError("invalid HorseLink body")
    return body + f"*{binascii.crc_hqx(body, 0xFFFF):04X}\n".encode("ascii")


def decode(line: bytes) -> bytes:
    if not 7 <= len(line) <= 32 or not line.endswith(b"\n") or line[-6:-5] != b"*":
        raise ValueError("invalid frame length or delimiter")
    body = line[:-6]
    if any(c < 32 or c > 126 or c == 42 for c in body):
        raise ValueError("invalid body characters")
    if any(c not in b"0123456789ABCDEFabcdef" for c in line[-5:-1]):
        raise ValueError("invalid CRC digits")
    if int(line[-5:-1], 16) != binascii.crc_hqx(body, 0xFFFF):
        raise ValueError("CRC mismatch")
    return body


class FrameReader:
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False
        self.errors = 0

    def feed(self, data: bytes):
        frames = []
        for c in data:
            if c == 10:
                if not self.discard:
                    try:
                        frames.append(decode(bytes(self.buffer) + b"\n"))
                    except ValueError:
                        self.errors += 1
                else:
                    self.errors += 1
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) >= 31 or c < 32 or c > 126:
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(c)
        return frames


def _hex(data: bytes) -> int:
    if not data or any(c not in b"0123456789ABCDEF" for c in data):
        raise ValueError("invalid hexadecimal field")
    return int(data, 16)


def parse_reply(body: bytes) -> dict:
    if body[:2] == b"Y1" and len(body) == 18:
        return {"type": "hello", "nonce": _hex(body[2:10]), "session": _hex(body[10:18])}
    if body[:2] == b"A1" and len(body) == 16:
        result = _hex(body[14:16])
        if result >= len(RESULTS):
            raise ValueError("unknown result")
        return {"type": "ack", "session": _hex(body[2:10]), "seq": _hex(body[10:14]),
                "result": RESULTS[result]}
    if body[:2] == b"U1" and len(body) == 26:
        state, gait, phase, fault, online, locked = [_hex(body[i:i+2]) for i in range(14, 26, 2)]
        if state >= len(STATES) or gait >= len(GAITS) or phase >= len(PHASES) or locked > 1:
            raise ValueError("invalid controller state")
        return {"type": "status", "session": _hex(body[2:10]), "seq": _hex(body[10:14]),
                "state": STATES[state], "gait": GAITS[gait], "phase": PHASES[phase],
                "fault": fault, "online_mask": online, "remote_locked": bool(locked)}
    raise ValueError("unknown reply")


class HorsePcLink:
    def __init__(self, port):
        self.serial = port
        self.reader = FrameReader()
        self.session = 0
        self.seq = 0

    def _exchange(self, body: bytes, match):
        wire = encode(body)
        for _ in range(3):
            if self.serial.write(wire) != len(wire):
                raise LinkError("incomplete serial write; action outcome is unconfirmed")
            deadline = time.monotonic() + 0.1
            while time.monotonic() < deadline:
                size = min(max(self.serial.in_waiting, 1), 128)
                for raw in self.reader.feed(self.serial.read(size)):
                    try:
                        reply = parse_reply(raw)
                    except ValueError:
                        self.reader.errors += 1
                        continue
                    if match(reply):
                        return reply
        raise LinkError("reply timeout; action outcome is unconfirmed (no automatic reconnect)")

    def connect(self):
        nonce = secrets.randbits(32) or 1
        reply = self._exchange(f"H1{nonce:08X}".encode(),
            lambda p: (p["type"] == "hello" and p["nonce"] == nonce and p["session"] != 0) or
                      (p["type"] == "ack" and p["session"] == 0 and p["seq"] == 0 and p["result"] == "REJECTED"))
        if reply["type"] == "ack":
            raise LinkError("another PC session holds the link; no control was acquired")
        self.session = reply["session"]
        self.seq = 0
        return reply

    def command(self, op: str):
        if op not in ("T", "K", "Z"):
            raise ValueError("unknown command")
        if not self.session:
            raise LinkError("connect before requesting control")
        self.seq = (self.seq + 1) & 0xFFFF
        reply = self._exchange(f"{op}1{self.session:08X}{self.seq:04X}".encode(),
            lambda p: p["type"] in ("ack", "status") and p["session"] == self.session and p["seq"] == self.seq
                      and (op == "K" or p["type"] == "ack"))
        if reply["type"] == "ack" and reply["result"] not in ("OK", "PENDING"):
            raise LinkError(f"controller returned {reply['result']}; do not replay TROT automatically")
        if op == "K" and reply["type"] != "status":
            raise LinkError("query did not return controller status")
        return reply

    def status(self):
        return self.command("K")
