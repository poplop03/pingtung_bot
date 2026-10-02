#!/usr/bin/env python3
"""
pick_fruit - drive forward, and hit every red/yellow fruit that reaches the hit line.

    /vision/fruit/line_hit (FruitLineHit) ──┐                 ┌──► /mega/cmd_vel (Twist)
    /mega/status (Int32MultiArray)        ──┼──► controller ──┼──► /mega/goto, /mega/step
    /vision/debug_image (Image) ──► web UI ◄┘   (state        └──► /mega/gripper
                                                 machine)          /mega/set_home (service)

The web UI (http://<robot-ip>:8080/) shows the camera, START/STOP, the forward
PWM, teleop keys, gantry jog and the saved "ready" and "hit" gantry
positions. See controller.py for the state machine.

Everything that changes the controller runs on the ROS thread: HTTP requests
are queued and answered from the control timer, so there are no locks around
the controller itself.

The ready/hit positions and the forward PWM are saved to settings_file whenever
they change and loaded at startup.
"""

import json
import os
import queue
import signal
import threading
import time
from concurrent.futures import Future, TimeoutError as FutureTimeout

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int16MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

from pingtung_vision_interfaces.msg import FruitLineHit

from pick_fruit.controller import IDLE, Config, GantryStatus, PickController
from pick_fruit.web import FrameStore, WebServer

REPLY_TIMEOUT_S = 2.0


class PickFruitNode(Node):

    def __init__(self):
        super().__init__('pick_fruit')

        defaults = Config()
        self.declare_parameters('', [
            ('control_rate_hz', 20.0),
            ('image_topic', '/vision/debug_image'),
            ('host', '0.0.0.0'),
            ('port', 8080),
            ('jpeg_quality', 75),
            ('settings_file', '~/.ros/pick_fruit.json'),
        ] + [(name, value, ParameterDescriptor(dynamic_typing=True))   # 50 or 50.0 in the yaml
             for name, value in vars(defaults).items()])
        cfg = Config(**{name: type(value)(self.get_parameter(name).value)
                        for name, value in vars(defaults).items()})

        self._settings_file = os.path.expanduser(
            str(self.get_parameter('settings_file').value))
        saved = self._load_settings()
        self.ctrl = PickController(cfg, saved.get('positions'),
                                   saved.get('forward_pwm'))

        self._bridge = CvBridge()
        self._frames = FrameStore()
        self._quality = int(self.get_parameter('jpeg_quality').value)
        self._requests: 'queue.Queue[tuple[dict, Future]]' = queue.Queue()
        self._state_lock = threading.Lock()
        self._state = self.ctrl.snapshot(time.monotonic())
        self._last_mode = self.ctrl.mode
        self._wheel_pwm = None        # what wheel_control actually sends, for the UI

        self.cmd_pub = self.create_publisher(Twist, '/mega/cmd_vel', 10)
        self.goto_pub = self.create_publisher(Int32MultiArray, '/mega/goto', 10)
        self.step_pub = self.create_publisher(Int32MultiArray, '/mega/step', 10)
        self.grip_pub = self.create_publisher(Float32, '/mega/gripper', 10)
        self.home_client = self.create_client(Trigger, '/mega/set_home')

        self.create_subscription(Int32MultiArray, '/mega/status', self.on_status, 10)
        self.create_subscription(FruitLineHit, '/vision/fruit/line_hit', self.on_fruit, 10)
        self.create_subscription(Int16MultiArray, '/mega/wheel_pwm', self.on_wheel_pwm, 10)
        image_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        image_topic = str(self.get_parameter('image_topic').value)
        self.create_subscription(Image, image_topic, self.on_image, image_qos)
        self.create_timer(1.0 / float(self.get_parameter('control_rate_hz').value), self.tick)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._web = WebServer(host, port, self._frames, self._get_state, self._submit)
        self._web.start()
        self.get_logger().info(f'web UI at http://{host}:{port}/ (camera: {image_topic})')

    # ---------------- ROS inputs ----------------

    def on_status(self, msg: Int32MultiArray):
        try:
            status = GantryStatus.from_array(msg.data)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        self.ctrl.on_status(status, time.monotonic())

    def on_fruit(self, msg: FruitLineHit):
        self.ctrl.on_fruit(msg.valid, msg.line_hit == FruitLineHit.HIT, time.monotonic())

    def on_wheel_pwm(self, msg: Int16MultiArray):
        self._wheel_pwm = list(msg.data)

    def on_image(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            self._frames.put(encoded.tobytes())

    # ---------------- control loop ----------------

    def tick(self):
        now = time.monotonic()
        while True:
            try:
                body, future = self._requests.get_nowait()
            except queue.Empty:
                break
            self._handle(body, future, now)

        base, commands = self.ctrl.tick(now)
        if base is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = float(base[0]), float(base[1])
            self.cmd_pub.publish(twist)
        for kind, arg in commands:
            if kind == 'gripper':
                self.grip_pub.publish(Float32(data=float(arg)))
            else:
                pub = self.goto_pub if kind == 'goto' else self.step_pub
                pub.publish(Int32MultiArray(data=[int(arg[0]), int(arg[1])]))

        if self.ctrl.mode != self._last_mode:
            # separate calls: rclpy refuses one call site logging at two severities
            text = f'{self._last_mode} -> {self.ctrl.mode}: {self.ctrl.message}'
            if self.ctrl.error:
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)
            self._last_mode = self.ctrl.mode

        state = self.ctrl.snapshot(now)
        state['wheel_pwm'] = self._wheel_pwm
        with self._state_lock:
            self._state = state

    def _handle(self, body: dict, future: Future, now: float):
        ctrl = self.ctrl
        name = body.get('cmd')
        try:
            if name == 'start':
                result = ctrl.start(now)
            elif name == 'stop':
                result = ctrl.stop(now, 'stopped by user')
            elif name == 'drive':
                result = ctrl.drive(float(body.get('linear', 0.0)),
                                    float(body.get('angular', 0.0)), now)
            elif name == 'jog':
                s1, s2 = body['steps']
                result = ctrl.jog(int(s1), int(s2), now)
            elif name == 'goto':
                result = ctrl.goto(str(body.get('name')), now)
            elif name == 'gripper':
                result = ctrl.gripper(float(body['deg']), now)
            elif name == 'speed':
                result = ctrl.set_forward_pwm(float(body['value']))
                self._save_settings()
            elif name == 'save':
                result = ctrl.save_position(str(body.get('name')), now)
                if result[0]:
                    self._save_settings()
            elif name == 'set_home':
                self._set_home(future)
                return
            else:
                result = (False, f'unknown command {name!r}')
        except (KeyError, TypeError, ValueError) as exc:
            result = (False, f'bad {name} command: {exc}')
        if name != 'drive' and result[1]:
            self.get_logger().info(f'{name}: {result[1]}')
        future.set_result(result)

    def _set_home(self, future: Future):
        if self.ctrl.mode != IDLE:
            future.set_result((False, 'stop the run first'))
            return
        if not self.home_client.service_is_ready():
            future.set_result((False, '/mega/set_home not available - is mega_bridge running?'))
            return

        def done(call):
            try:
                res = call.result()
                future.set_result((res.success, res.message))
            except Exception as exc:     # noqa: BLE001 - report anything to the UI
                future.set_result((False, f'set_home failed: {exc}'))

        self.home_client.call_async(Trigger.Request()).add_done_callback(done)

    # ---------------- web server side (HTTP threads) ----------------

    def _get_state(self) -> dict:
        with self._state_lock:
            return self._state

    def _submit(self, body: dict):
        future: Future = Future()
        self._requests.put((body, future))
        try:
            return future.result(timeout=REPLY_TIMEOUT_S)
        except FutureTimeout:
            return False, 'robot did not answer'

    # ---------------- settings file ----------------

    def _load_settings(self) -> dict:
        try:
            with open(self._settings_file) as f:
                data = json.load(f)
            self.get_logger().info(f'loaded {self._settings_file}: {data}')
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            self.get_logger().warn(
                f'no {self._settings_file} yet: jog the gantry and save a hit position')
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'cannot read {self._settings_file}: {exc}')
        return {}

    def _save_settings(self):
        data = {'positions': {k: (list(v) if v else None)
                              for k, v in self.ctrl.positions.items()},
                'forward_pwm': self.ctrl.forward_pwm}
        try:
            os.makedirs(os.path.dirname(self._settings_file) or '.', exist_ok=True)
            tmp = self._settings_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self._settings_file)
        except OSError as exc:
            self.get_logger().error(f'cannot save {self._settings_file}: {exc}')

    def destroy_node(self):
        self._web.shutdown()
        # leave the base stopped: wheel_control would time out anyway, but be explicit.
        # After Ctrl-C the context is already shut down and publishing would raise.
        if rclpy.ok():
            self.cmd_pub.publish(Twist())
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
    node = PickFruitNode()
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
