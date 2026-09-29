#!/usr/bin/env python3
"""
gantry_test - run the gantry and gripper through a fixed sequence via mega_bridge.

    gantry_test --/mega/step, /mega/gripper--> mega_bridge --> Mega
                <--------- /mega/status ------

Each cycle, with every enabled part:
    gripper open
    axis 1  +axis1_steps, then back   (-axis1_steps)
    axis 2  +axis2_steps, then back   (-axis2_steps)
    gripper close, then open

Steps are relative, so the gantry ends where it started. The next command is
sent only once /mega/status shows the previous one finished: steps_left == 0
for a stepper, servo_deg at the target for the gripper. A move that has not
finished within move_timeout_s aborts the test. The node exits when done.

There is no homing, so start with the gantry away from the ends of travel,
and try small step counts first to check the direction.

    ros2 run mega_bridge gantry_test
    ros2 run mega_bridge gantry_test --ros-args -p axis1_steps:=200 -p test_axis2:=false
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray

FRESH_STATUS = 2    # status messages to wait for after a command before trusting "done"


class GantryTest(Node):

    def __init__(self):
        super().__init__('gantry_test')

        self.declare_parameters('', [
            ('axis1_steps', 800),
            ('axis2_steps', 800),
            ('test_axis1', True),
            ('test_axis2', True),
            ('test_gripper', True),
            ('gripper_open', 90.0),     # degrees
            ('gripper_closed', 20.0),   # degrees
            ('cycles', 1),
            ('move_timeout_s', 30.0),
            ('start_timeout_s', 10.0),  # wait this long for mega_bridge to report
        ])
        self.P = {name: self.get_parameter(name).value
                  for name in self._parameters
                  if name not in ('use_sim_time',)}

        self.step_pub = self.create_publisher(Int32MultiArray, '/mega/step', 10)
        self.grip_pub = self.create_publisher(Float32, '/mega/gripper', 10)
        self.create_subscription(Int32MultiArray, '/mega/status', self.on_status, 10)

        self.status = None
        self.status_count = 0
        self.plan = self.build_plan()
        self.current = None           # (label, check) of the command in flight
        self.sent_count = 0           # status_count when it was sent
        self.sent_t = 0.0
        self.start_t = time.monotonic()
        self.done = False
        self.ok = False

        self.get_logger().info(f'{len(self.plan)} commands queued, waiting for /mega/status')
        self.create_timer(0.05, self.tick)

    # ---------------------------------------------------------------- plan
    def build_plan(self):
        """List of (label, send_fn, check_fn). check_fn(status) -> finished?"""
        plan = []
        p = self.P

        def move(axis, steps):
            data = [steps, 0] if axis == 1 else [0, steps]
            return (f'axis {axis} {steps:+d} steps',
                    lambda: self.step_pub.publish(Int32MultiArray(data=data)),
                    lambda s: s[0] == 0 and s[1] == 0)

        def grip(label, deg):
            return (f'gripper {label} ({deg:.0f} deg)',
                    lambda: self.grip_pub.publish(Float32(data=float(deg))),
                    lambda s: abs(s[2] - deg) <= 1)

        for _ in range(int(p['cycles'])):
            if p['test_gripper']:
                plan.append(grip('open', p['gripper_open']))
            for axis in (1, 2):
                n = int(p[f'axis{axis}_steps'])
                if p[f'test_axis{axis}'] and n != 0:
                    plan += [move(axis, n), move(axis, -n)]
            if p['test_gripper']:
                plan += [grip('close', p['gripper_closed']),
                         grip('open', p['gripper_open'])]
        return plan

    # ---------------------------------------------------------------- I/O
    def on_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self.status = list(msg.data)
            self.status_count += 1

    def tick(self):
        if self.done:
            return
        now = time.monotonic()

        # wait for mega_bridge to be up and subscribed before the first command
        if self.status is None or self.step_pub.get_subscription_count() == 0 \
                or self.grip_pub.get_subscription_count() == 0:
            if now - self.start_t > self.P['start_timeout_s']:
                self.finish(False, 'no /mega/status or no subscriber - is mega_bridge running?')
            return

        if self.current is not None:
            label, check = self.current
            fresh = self.status_count - self.sent_count >= FRESH_STATUS
            if fresh and check(self.status):
                self.get_logger().info(f'  done in {now - self.sent_t:.1f} s   status {self.status}')
                self.current = None
            elif now - self.sent_t > self.P['move_timeout_s']:
                self.finish(False, f'"{label}" did not finish, status {self.status}')
                return
            else:
                return

        if not self.plan:
            self.finish(True, 'all commands finished, gantry back at start')
            return

        label, send, check = self.plan.pop(0)
        self.get_logger().info(f'-> {label}')
        send()
        self.current = (label, check)
        self.sent_count = self.status_count
        self.sent_t = now

    def finish(self, ok, text):
        self.done = True
        self.ok = ok
        (self.get_logger().info if ok else self.get_logger().error)(
            ('PASS: ' if ok else 'FAIL: ') + text)


def main(args=None):
    rclpy.init(args=args)
    node = GantryTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = node.ok
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
