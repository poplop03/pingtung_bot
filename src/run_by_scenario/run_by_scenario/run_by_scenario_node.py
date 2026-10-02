#!/usr/bin/env python3
"""
run_by_scenario - record a teleop drive, save it as a scenario, play it back.

    web UI (WASD or joystick, PWM, record, load, start) ──► controller ──► /mega/cmd_vel (Twist)
    /mega/status    (Int32MultiArray) ──► health, abort playback when lost
    /mega/wheel_pwm (Int16MultiArray) ──► shown in the UI

The web UI (http://<robot-ip>:8080/) has only the teleop (W/A/S/D or an
on-screen joystick), the linear and angular PWM, RECORD / STOP and a scenario
list with LOAD / START. Scenarios are JSON files
in scenario_dir, one per name; see scenario.py for the format.

Everything that changes the controller runs on the ROS thread: HTTP requests
are queued and answered from the control timer, so there are no locks around
the controller itself.
"""

import json
import os
import queue
import signal
import threading
import time
from concurrent.futures import Future, TimeoutError as FutureTimeout
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray

from run_by_scenario.scenario import (
    IDLE, Config, ScenarioController, ScenarioStore, valid_name)
from run_by_scenario.web import WebServer

REPLY_TIMEOUT_S = 2.0


class RunByScenarioNode(Node):

    def __init__(self):
        super().__init__('run_by_scenario')

        defaults = Config()
        self.declare_parameters('', [
            ('control_rate_hz', 50.0),
            ('host', '0.0.0.0'),
            ('port', 8080),
            ('scenario_dir', '~/.ros/scenarios'),
            ('settings_file', '~/.ros/run_by_scenario.json'),
        ] + [(name, value, ParameterDescriptor(dynamic_typing=True))   # 50 or 50.0 in the yaml
             for name, value in vars(defaults).items()])
        cfg = Config(**{name: type(value)(self.get_parameter(name).value)
                        for name, value in vars(defaults).items()})

        self.store = ScenarioStore(str(self.get_parameter('scenario_dir').value))
        self._settings_file = os.path.expanduser(
            str(self.get_parameter('settings_file').value))
        saved = self._load_settings()
        self.ctrl = ScenarioController(cfg, saved.get('linear_pwm', saved.get('forward_pwm')),
                                       saved.get('angular_pwm'), saved.get('control_mode'))
        last = saved.get('loaded')
        if isinstance(last, str) and valid_name(last) and self.store.exists(last):
            self._load(last)
        self._overwrite = False

        self._requests: 'queue.Queue[tuple[dict, Future]]' = queue.Queue()
        self._state_lock = threading.Lock()
        self._scenarios = self.store.names()
        self._wheel_pwm = None
        self._state = self._make_state(time.monotonic())
        self._last_mode = self.ctrl.mode

        self.cmd_pub = self.create_publisher(Twist, '/mega/cmd_vel', 10)
        self.create_subscription(Int32MultiArray, '/mega/status', self.on_status, 10)
        self.create_subscription(Int16MultiArray, '/mega/wheel_pwm', self.on_wheel_pwm, 10)
        self.create_timer(1.0 / float(self.get_parameter('control_rate_hz').value), self.tick)

        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._web = WebServer(host, port, self._get_state, self._submit)
        self._web.start()
        self.get_logger().info(
            f'web UI at http://{host}:{port}/ | scenarios in {self.store.directory}')

    # ---------------- ROS inputs ----------------

    def on_status(self, msg: Int32MultiArray):
        # [steps_left1, steps_left2, servo_deg, wheel_failsafe, pos1, pos2]
        if len(msg.data) < 4:
            self.get_logger().warn('short /mega/status', throttle_duration_sec=5.0)
            return
        self.ctrl.on_status(bool(msg.data[3]), time.monotonic())

    def on_wheel_pwm(self, msg: Int16MultiArray):
        self._wheel_pwm = list(msg.data)

    # ---------------- control loop ----------------

    def tick(self):
        now = time.monotonic()
        while True:
            try:
                body, future = self._requests.get_nowait()
            except queue.Empty:
                break
            self._handle(body, future, now)

        base = self.ctrl.tick(now)
        if base is not None:
            twist = Twist()
            twist.linear.x, twist.angular.z = float(base[0]), float(base[1])
            self.cmd_pub.publish(twist)

        if self.ctrl.mode != self._last_mode:
            self._save_recording()
            # separate calls: rclpy refuses one call site logging at two severities
            text = f'{self._last_mode} -> {self.ctrl.mode}: {self.ctrl.message}'
            if self.ctrl.error:
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)
            self._last_mode = self.ctrl.mode

        state = self._make_state(now)
        with self._state_lock:
            self._state = state

    def _handle(self, body: dict, future: Future, now: float):
        ctrl = self.ctrl
        name = body.get('cmd')
        try:
            if name == 'drive':
                result = ctrl.drive(float(body.get('linear', 0.0)),
                                    float(body.get('angular', 0.0)), now)
            elif name == 'stop':
                result = ctrl.stop(now, 'stopped by user')
                self._save_recording()
            elif name == 'pwm':
                result = ctrl.set_pwm(float(body['linear']), float(body['angular']))
                self._save_settings()
            elif name == 'control':
                result = ctrl.set_control_mode(str(body.get('mode')), now)
                self._save_settings()
            elif name == 'record':
                result = self._record(str(body.get('name', '')),
                                      bool(body.get('overwrite', False)), now)
            elif name == 'load':
                result = self._load(str(body.get('name', '')))
            elif name == 'start':
                result = ctrl.start(now)
            elif name == 'delete':
                result = self._delete(str(body.get('name', '')))
            else:
                result = (False, f'unknown command {name!r}')
        except (KeyError, TypeError, ValueError) as exc:
            result = (False, f'bad {name} command: {exc}')
        if name != 'drive' and result[1]:
            self.get_logger().info(f'{name}: {result[1]}')
        future.set_result(result)

    # ---------------- scenarios ----------------

    def _record(self, name: str, overwrite: bool, now: float):
        if not valid_name(name):
            return False, f'bad name {name!r}: use letters, digits, _ and -'
        if self.store.exists(name) and not overwrite:
            return False, f'"{name}" already exists'
        self._overwrite = overwrite
        return self.ctrl.record(name, datetime.now().isoformat(timespec='seconds'), now)

    def _save_recording(self):
        scenario = self.ctrl.pop_recorded()
        if scenario is None:
            return
        try:
            if self.store.exists(scenario.name) and not self._overwrite:
                raise OSError(f'"{scenario.name}" was created meanwhile, not overwriting')
            path = self.store.save(scenario)
        except OSError as exc:
            self.ctrl.message, self.ctrl.error = f'cannot save: {exc}', True
            self.get_logger().error(self.ctrl.message)
            return
        self.ctrl.message += f', saved to {path}'
        self._scenarios = self.store.names()
        self._save_settings()

    def _load(self, name: str):
        if self.ctrl.mode != IDLE:
            return False, f'stop first ({self.ctrl.mode})'
        try:
            scenario = self.store.load(name, self.ctrl.cfg.k_ff)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(f'cannot load {name!r}: {exc}')
            return False, f'cannot load "{name}": {exc}'
        result = self.ctrl.load(scenario)
        self._save_settings()
        return result

    def _delete(self, name: str):
        if self.ctrl.mode != IDLE:
            return False, f'stop first ({self.ctrl.mode})'
        try:
            self.store.delete(name)
        except (OSError, ValueError) as exc:
            return False, f'cannot delete "{name}": {exc}'
        if self.ctrl.loaded is not None and self.ctrl.loaded.name == name:
            self.ctrl.loaded = None
            self._save_settings()
        self._scenarios = self.store.names()
        return True, f'deleted "{name}"'

    # ---------------- web server side (HTTP threads) ----------------

    def _make_state(self, now: float) -> dict:
        state = self.ctrl.snapshot(now)
        state['wheel_pwm'] = self._wheel_pwm
        state['scenarios'] = self._scenarios
        return state

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
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            pass
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'cannot read {self._settings_file}: {exc}')
        return {}

    def _save_settings(self):
        data = {'linear_pwm': self.ctrl.linear_pwm,
                'angular_pwm': self.ctrl.angular_pwm,
                'control_mode': self.ctrl.control_mode,
                'loaded': self.ctrl.loaded.name if self.ctrl.loaded else None}
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
        # leave the base stopped. After Ctrl-C the context is already shut
        # down and publishing would raise.
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
    node = RunByScenarioNode()
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
