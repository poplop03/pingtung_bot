"""Serial link to the ESP32 running the JZ2407DB firmware.

Protocol (must match the firmware):
    0xAA 0xA5 | int16 m1 (LE) | int16 m2 (LE) | crc8(payload, poly 0x07)

A background thread streams the latest command at a fixed rate so the
firmware's 300 ms failsafe never trips while the node is healthy. The ROS
callback only updates a value - it never blocks on the serial port.
"""

import struct
import threading
import time

import serial

SYNC1, SYNC2 = 0xAA, 0xA5


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(m1: int, m2: int) -> bytes:
    m1 = max(-255, min(255, int(m1)))
    m2 = max(-255, min(255, int(m2)))
    payload = struct.pack("<hh", m1, m2)
    return bytes([SYNC1, SYNC2]) + payload + bytes([crc8(payload)])


class MotorLink:
    def __init__(self, port: str, baud: int = 115200, rate_hz: float = 50.0,
                 logger=None):
        self.log = logger
        self.ser = serial.Serial(port, baud, timeout=0.1, write_timeout=0.2)
        time.sleep(2.0)                       # ESP32 resets when the port opens
        self.ser.reset_input_buffer()

        self._cmd = (0, 0)
        self._lock = threading.Lock()
        self._running = True
        self._period = 1.0 / rate_hz
        self.tx_errors = 0

        self._thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._thread.start()

    def set(self, m1: int, m2: int) -> None:
        with self._lock:
            self._cmd = (int(m1), int(m2))

    def get(self):
        with self._lock:
            return self._cmd

    def _tx_loop(self) -> None:
        next_t = time.monotonic()
        while self._running:
            with self._lock:
                m1, m2 = self._cmd
            try:
                self.ser.write(encode(m1, m2))
            except serial.SerialException as exc:
                self.tx_errors += 1
                if self.log:
                    self.log.error(f"serial write failed: {exc}")
                time.sleep(0.1)
            next_t += self._period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()     # we fell behind; resync

    def close(self) -> None:
        self.set(0, 0)
        time.sleep(0.3)                       # let the firmware ramp down
        self._running = False
        self._thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass
