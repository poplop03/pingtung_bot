"""Framed serial protocol between the Jetson and the Arduino Mega.

Must match Arduino/mega_bridge/mega_bridge.ino.

    0xAA 0xA5 | type u8 | len u8 | payload[len] | crc8(type, len, payload)

crc8 is the same one the ESP32 link uses (poly 0x07, init 0x00). All
multi-byte values are little-endian.

    type  dir          payload
    0x01  Jetson->Mega int16 m1, int16 m2          wheel PWM, -255..255
    0x02  Jetson->Mega int32 s1, int32 s2          relative stepper moves
    0x03  Jetson->Mega uint8 deg                   servo angle, 0..180
    0x81  Mega->Jetson int32 rem1, int32 rem2,     steps still to go,
                       uint8 servo, uint8 flags    servo pos, bit0 = failsafe
"""

import struct

from wheel_control.motor_link import crc8

SYNC = bytes([0xAA, 0xA5])
MAX_LEN = 32                # longest payload either side will ever send

T_DRIVE = 0x01
T_STEP = 0x02
T_SERVO = 0x03
T_STATUS = 0x81

STATUS_FMT = '<iiBB'
STATUS_LEN = struct.calcsize(STATUS_FMT)
FLAG_FAILSAFE = 0x01


def encode(ftype: int, payload: bytes) -> bytes:
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + bytes([crc8(body)])


def encode_drive(m1: int, m2: int) -> bytes:
    m1 = max(-255, min(255, int(m1)))
    m2 = max(-255, min(255, int(m2)))
    return encode(T_DRIVE, struct.pack('<hh', m1, m2))


def encode_step(s1: int, s2: int) -> bytes:
    return encode(T_STEP, struct.pack('<ii', int(s1), int(s2)))


def encode_servo(deg: float) -> bytes:
    return encode(T_SERVO, bytes([max(0, min(180, int(round(deg))))]))


def decode_status(payload: bytes):
    """-> (rem1, rem2, servo_deg, flags)"""
    return struct.unpack(STATUS_FMT, payload)


class Decoder:
    """Streaming frame parser. Feed it whatever read() returned."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data: bytes):
        """Return a list of (type, payload) for every complete, valid frame."""
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # keep a trailing 0xAA, it may be the first half of a sync
                del self.buf[:-1 if self.buf.endswith(SYNC[:1]) else len(self.buf)]
                return frames
            del self.buf[:i]
            if len(self.buf) < 4:
                return frames
            n = self.buf[3]
            if n > MAX_LEN:
                self.bad += 1
                del self.buf[:1]
                continue
            if len(self.buf) < 5 + n:
                return frames
            body = bytes(self.buf[2:4 + n])
            if crc8(body) == self.buf[4 + n]:
                frames.append((body[0], body[2:]))
                del self.buf[:5 + n]
            else:
                self.bad += 1
                del self.buf[:1]     # resync on the next 0xAA 0xA5
