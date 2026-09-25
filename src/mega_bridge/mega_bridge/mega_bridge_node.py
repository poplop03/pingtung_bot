#!/usr/bin/env python3
"""
mega_bridge - the one node that owns the serial port to the Arduino Mega.

    /mega/wheel_pwm (Int16MultiArray [m1, m2]) --.
    /mega/step      (Int32MultiArray [s1, s2]) --+--> USB serial --> Mega
    /mega/gripper   (Float32, degrees)         --'
    /mega/status    (Int32MultiArray)          <----- 10 Hz from the Mega
                    [steps_left1, steps_left2, servo_deg, wheel_failsafe]

/mega/cmd_vel is not handled here: wheel_control turns it into wheel PWM
(with the IMU heading loop) and publishes /mega/wheel_pwm.

Wheel PWM is streamed at tx_rate_hz whatever arrives, and drops to 0 when
/mega/wheel_pwm goes quiet for pwm_timeout_s, so the firmware's 300 ms
failsafe only fires when this node itself is gone. Step and gripper commands
are events: each message is sent exactly once. Steps are RELATIVE - [1600, 0]
twice moves stepper 1 by 3200 in total.

All writes happen on one thread, so frames never interleave on the wire.
"""

import queue
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray, Int32MultiArray

from mega_bridge import protocol as proto


class MegaBridge(Node):

    def __init__(self):
        super().__init__('mega_bridge')

        self.declare_parameters('', [
            ('port', '/dev/ttyACM0'),
            ('baud', 115200),
            ('tx_rate_hz', 50.0),       # must stay well inside the 300 ms failsafe
            ('pwm_timeout_s', 0.5),     # stale /mega/wheel_pwm -> send 0
            ('status_timeout_s', 1.0),  # warn if the Mega stops reporting
        ])
        self.P = {name: self.get_parameter(name).value
                  for name in self._parameters
                  if name not in ('use_sim_time',)}

        try:
            self.ser = serial.Serial(self.P['port'], int(self.P['baud']),
                                     timeout=0.05, write_timeout=0.2)
        except Exception as exc:
            self.get_logger().fatal(f"cannot open {self.P['port']}: {exc}")
            raise
        time.sleep(2.0)                       # the Mega resets when the port opens
        self.ser.reset_input_buffer()

        # ---------------- state shared with the I/O threads ----------------
        self._lock = threading.Lock()
        self._pwm = (0, 0)
        self._pwm_t = float('-inf')           # monotonic time of last wheel_pwm
        self._status_t = time.monotonic()     # grace period before the first warn
        self._events = queue.Queue()          # encoded STEP / SERVO frames
        self._decoder = proto.Decoder()
        self._running = True

        # ---------------- ROS I/O ----------------
        self.create_subscription(Int16MultiArray, '/mega/wheel_pwm', self.on_wheel_pwm, 10)
        self.create_subscription(Int32MultiArray, '/mega/step', self.on_step, 10)
        self.create_subscription(Float32, '/mega/gripper', self.on_gripper, 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/mega/status', 10)
        self.create_timer(1.0, self.check_status_age)

        self._tx = threading.Thread(target=self._tx_loop, daemon=True)
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx.start()
        self._rx.start()

        self.get_logger().info(f"mega_bridge up on {self.P['port']}")

    # ---------------- callbacks ----------------

    def on_wheel_pwm(self, msg: Int16MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn('/mega/wheel_pwm needs [m1, m2]', throttle_duration_sec=2.0)
            return
        with self._lock:
            self._pwm = (int(msg.data[0]), int(msg.data[1]))
            self._pwm_t = time.monotonic()

    def on_step(self, msg: Int32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn(f'/mega/step needs [step1, step2], got {list(msg.data)}')
            return
        s1, s2 = int(msg.data[0]), int(msg.data[1])
        self._events.put(proto.encode_step(s1, s2))
        self.get_logger().info(f'step {s1} {s2}')

    def on_gripper(self, msg: Float32):
        deg = float(msg.data)
        if not 0.0 <= deg <= 180.0:
            self.get_logger().warn(f'gripper {deg:.1f} deg clamped to 0..180')
        self._events.put(proto.encode_servo(deg))

    def check_status_age(self):
        with self._lock:
            age = time.monotonic() - self._status_t
        if age > float(self.P['status_timeout_s']):
            self.get_logger().warn(
                f'no status from the Mega for {age:.1f} s - wrong firmware or port?',
                throttle_duration_sec=5.0)

    # ---------------- serial threads ----------------

    def _tx_loop(self):
        period = 1.0 / float(self.P['tx_rate_hz'])
        timeout = float(self.P['pwm_timeout_s'])
        next_t = time.monotonic()
        while self._running:
            frames = []
            while True:
                try:
                    frames.append(self._events.get_nowait())
                except queue.Empty:
                    break
            with self._lock:
                m1, m2 = self._pwm
                if time.monotonic() - self._pwm_t > timeout:
                    m1 = m2 = 0
            frames.append(proto.encode_drive(m1, m2))
            try:
                self.ser.write(b''.join(frames))
            except OSError as exc:            # SerialException is one too
                self.get_logger().error(f'serial write failed: {exc}',
                                        throttle_duration_sec=2.0)
                time.sleep(0.1)
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()     # we fell behind; resync

    def _rx_loop(self):
        while self._running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except OSError as exc:            # unplugged port: bare OSError from in_waiting
                self.get_logger().error(f'serial read failed: {exc}',
                                        throttle_duration_sec=2.0)
                time.sleep(0.1)
                continue
            for ftype, payload in self._decoder.feed(data):
                if ftype != proto.T_STATUS or len(payload) != proto.STATUS_LEN:
                    continue
                rem1, rem2, servo, flags = proto.decode_status(payload)
                with self._lock:
                    self._status_t = time.monotonic()
                msg = Int32MultiArray()
                msg.data = [rem1, rem2, servo, flags & proto.FLAG_FAILSAFE]
                self.status_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        self._tx.join(timeout=1.0)
        self._rx.join(timeout=1.0)
        try:
            self.ser.write(proto.encode_drive(0, 0))   # firmware ramps to a stop
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MegaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
