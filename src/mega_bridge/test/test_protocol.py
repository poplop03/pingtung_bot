import struct

import pytest

from mega_bridge import protocol as proto
from wheel_control.motor_link import encode as esp32_encode


def status_frame(rem1, rem2, servo, flags):
    return proto.encode(proto.T_STATUS, struct.pack(proto.STATUS_FMT, rem1, rem2, servo, flags))


@pytest.mark.parametrize('frame, ftype, payload', [
    (proto.encode_drive(-120, 255), proto.T_DRIVE, struct.pack('<hh', -120, 255)),
    (proto.encode_step(1600, -800), proto.T_STEP, struct.pack('<ii', 1600, -800)),
    (proto.encode_servo(45.4), proto.T_SERVO, bytes([45])),
])
def test_round_trip(frame, ftype, payload):
    assert proto.Decoder().feed(frame) == [(ftype, payload)]


def test_clamping():
    assert proto.encode_drive(999, -999)[4:8] == struct.pack('<hh', 255, -255)
    assert proto.encode_servo(-10)[4] == 0
    assert proto.encode_servo(200)[4] == 180


def test_drive_payload_matches_esp32_layout():
    # same int16 m1, m2 little-endian payload as the ESP32 link, only framed differently
    assert proto.encode_drive(-37, 200)[4:8] == esp32_encode(-37, 200)[2:6]


def test_status_decode():
    (ftype, payload), = proto.Decoder().feed(status_frame(123456, -7, 90, 1))
    assert ftype == proto.T_STATUS
    assert proto.decode_status(payload) == (123456, -7, 90, 1)


def test_garbage_and_split_reads():
    stream = b'\x00\xaa\x13hello' + proto.encode_step(5, 6) + b'\xaa' + proto.encode_servo(10)
    dec = proto.Decoder()
    frames = []
    for i in range(len(stream)):             # worst case: one byte per read()
        frames += dec.feed(stream[i:i + 1])
    assert frames == [(proto.T_STEP, struct.pack('<ii', 5, 6)), (proto.T_SERVO, bytes([10]))]


def test_bad_crc_rejected_and_resyncs():
    bad = bytearray(proto.encode_drive(10, 10))
    bad[-1] ^= 0xFF
    dec = proto.Decoder()
    assert dec.feed(bytes(bad) + proto.encode_drive(20, 20)) == \
        [(proto.T_DRIVE, struct.pack('<hh', 20, 20))]
    assert dec.bad == 1


def test_oversize_length_rejected():
    dec = proto.Decoder()
    assert dec.feed(b'\xaa\xa5\x01\xff' + proto.encode_servo(1)) == [(proto.T_SERVO, bytes([1]))]
