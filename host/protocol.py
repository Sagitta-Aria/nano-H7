"""Protocol v1: bounded ASCII frames with sequence IDs and CCITT-FALSE CRC."""
from dataclasses import dataclass
import binascii
import re

MAX_FRAME = 384


@dataclass(frozen=True)
class Reply:
    sequence: int
    body: str

    @property
    def ok(self):
        return self.body.split(" ", 1)[0] == "OK"

    @property
    def fields(self):
        return dict(token.split("=", 1) for token in self.body.split()[1:] if "=" in token)


def encode(sequence: int, body: str) -> bytes:
    """Encode one request/reply. Control characters and framing delimiters fail."""
    if not 1 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence out of range")
    if not body or any(ord(c) < 32 or ord(c) > 126 or c == "*" for c in body):
        raise ValueError("invalid body")
    payload = f"{sequence} {body}".encode("ascii")
    frame = b"@" + payload + f"*{binascii.crc_hqx(payload, 0xFFFF):04X}\n".encode("ascii")
    if len(frame) >= MAX_FRAME:
        raise ValueError("frame too long")
    return frame


def decode(frame: bytes) -> Reply:
    """Validate a complete newline-terminated frame before exposing any fields."""
    if len(frame) >= MAX_FRAME or not frame.endswith(b"\n"):
        raise ValueError("incomplete or oversized frame")
    # Python 3.8 compatible: str.removesuffix is 3.9+, and JetPack 5 ships 3.8.
    line = frame[:-2] if frame[:-1].endswith(b"\r") else frame[:-1]
    match = re.fullmatch(rb"@([0-9]+) ([\x20-\x29\x2B-\x7E]+)\*([0-9A-Fa-f]{4})", line)
    if not match:
        raise ValueError("invalid frame")
    sequence = int(match[1])
    if not 1 <= sequence <= 0xFFFFFFFF:
        raise ValueError("sequence out of range")
    payload = line[1:line.rfind(b"*")]
    if binascii.crc_hqx(payload, 0xFFFF) != int(match[3], 16):
        raise ValueError("CRC mismatch")
    return Reply(sequence, match[2].decode("ascii"))


class Framer:
    """Reassemble fragmented serial input; discard oversized lines until newline."""
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False

    def feed(self, data: bytes):
        frames = []
        for byte in data:
            if byte == 10:
                if self.buffer and not self.discard:
                    frames.append(bytes(self.buffer) + b"\n")
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) >= MAX_FRAME - 2:
                    self.discard = True
                    self.buffer.clear()
                else:
                    self.buffer.append(byte)
        return frames
