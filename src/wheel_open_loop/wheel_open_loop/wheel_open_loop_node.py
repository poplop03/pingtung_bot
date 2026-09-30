#!/usr/bin/env python3
"""
wheel_open_loop - the wheel_control base with the feedback taken out.

    /cmd_vel (geometry_msgs/Twist) --> [mix] --> PWM --> ESP32

No IMU, no PID, no heading hold. The mix is exactly wheel_control's
feedforward path, so running the two back to back shows what the closed
loop adds and nothing else:

    base      = k_lin * v                              # = k_lin in wheel_control
    u         = k_turn * w                             # = k_ff in wheel_control
    pwm_left  = base - u        pwm_right = base + u

Any mismatch between the two motors shows up directly as a curve when
driving "straight", and nothing here corrects it. That is the point.

Kept from wheel_control because they are wiring or safety, not control:
swap_motors / invert_motor*, the max_pwm clamp and the cmd_vel timeout.
Deliberately left out: min_pwm lifting and slew limiting - the PWM sent is
the PWM computed.
"""

import os
import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from wheel_control.motor_link import MotorLink


class WheelOpenLoop(Node):

    def __init__(self):
        super().__init__('wheel_open_loop')

        # ---------------- parameters ----------------
        self.declare_parameters('', [
            # serial
            ('port', '/dev/ttyACM0'),
            ('baud', 115200),
            ('tx_rate_hz', 50.0),

            # mix - keep equal to wheel_control.yaml for a fair comparison
            ('k_lin', 150.0),            # PWM counts per m/s, = k_lin there
            ('k_turn', 35.0),            # PWM counts per rad/s, = k_ff there
            ('max_pwm', 255),

            # plumbing
            ('control_rate_hz', 50.0),
            ('cmd_timeout_s', 0.5),
            ('swap_motors', False),
            ('invert_motor1', False),
            ('invert_motor2', True),
            ('publish_debug', True),
        ])
        self.P = {name: self.get_parameter(name).value
                  for name in self._parameters
                  if name not in ('use_sim_time',)}

        # ---------------- state ----------------
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_t = self.get_clock().now()

        # ---------------- I/O ----------------
        try:
            self.link = MotorLink(self.P['port'], int(self.P['baud']),
                                  float(self.P['tx_rate_hz']), logger=self.get_logger())
        except Exception as exc:
            self.get_logger().fatal(f"cannot open {self.P['port']}: {exc}")
            raise

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        self.dbg_pub = (self.create_publisher(
            Float32MultiArray, '~/debug', 10) if self.P['publish_debug'] else None)

        self.create_timer(1.0 / float(self.P['control_rate_hz']), self.on_control)

        self.get_logger().info(
            f"wheel_open_loop up on {self.P['port']} | k_lin={self.P['k_lin']} | "
            f"k_turn={self.P['k_turn']} | NO feedback")

    # ---------------- callbacks ----------------

    def on_cmd_vel(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def on_control(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_t).nanoseconds * 1e-9 > float(self.P['cmd_timeout_s']):
            left = right = 0
        else:
            v = self.v_cmd
            base = float(self.P['k_lin']) * v
            u = float(self.P['k_turn']) * self.w_cmd
            left = self.clamp(base - u)
            right = self.clamp(base + u)

        self.send(left, right)
        if self.dbg_pub is not None:
            msg = Float32MultiArray()
            msg.data = [float(left), float(right)]     # pwm_l, pwm_r
            self.dbg_pub.publish(msg)

    # ---------------- helpers ----------------

    def clamp(self, pwm: float) -> int:
        hi = float(self.P['max_pwm'])
        return int(max(-hi, min(hi, pwm)))

    def send(self, left: int, right: int):
        m1, m2 = (right, left) if self.P['swap_motors'] else (left, right)
        if self.P['invert_motor1']:
            m1 = -m1
        if self.P['invert_motor2']:
            m2 = -m2
        self.link.set(m1, m2)

    def destroy_node(self):
        try:
            self.link.close()
        finally:
            super().destroy_node()


def _hold_off_signals(deadline_s: float = 3.0) -> None:
    """Let the cleanup finish, but never let the process outlive Ctrl+C.

    Under ros2 launch, Ctrl+C reaches the node twice (terminal and launch), and
    the second KeyboardInterrupt would cut destroy_node() short. Ignore it,
    make SIGTERM kill at once, and exit hard if the cleanup hangs.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    watchdog = threading.Timer(deadline_s, os._exit, (1,))
    watchdog.daemon = True
    watchdog.start()


def main(args=None):
    rclpy.init(args=args)
    node = WheelOpenLoop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _hold_off_signals()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
