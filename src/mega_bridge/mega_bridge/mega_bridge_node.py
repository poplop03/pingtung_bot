#!/usr/bin/env python3
"""
mega_bridge - the one node that owns the serial port to the Arduino Mega.

    /mega/wheel_pwm (Int16MultiArray [m1, m2]) --.
    /mega/step      (Int32MultiArray [s1, s2]) --+--> USB serial --> Mega
    /mega/goto      (Int32MultiArray [x, y])   --|
    /mega/gripper   (Float32, degrees)         --'
    /mega/status    (Int32MultiArray)          <----- 10 Hz from the Mega
                    [steps_left1, steps_left2, servo_deg, wheel_failsafe, pos1, pos2]
    /mega/set_home  (std_srvs/Trigger)         current gantry position becomes home

pos1/pos2 are steps from home. The firmware only reports steps_left, so the
position is tracked here: the Mega's target starts at 0 when it resets (which
it does when this node opens the port), every /mega/step goes through this
node, and pos = sum of steps sent - steps_left - home. There are no limit
switches: jog the gantry to home by hand or with /mega/step, then call
/mega/set_home. It is refused while either axis is moving.

/mega/goto takes an ABSOLUTE position [x, y] in steps from home (x = axis 1,
the same numbers as pos1/pos2 in /mega/status). It is turned into the relative
step that takes the current target there, so it is correct even while the
gantry is still moving, and sending the same [x, y] twice moves only once.

The position is saved to home_file whenever the gantry stops, and read back
at startup, so home survives a restart of this node - as long as nothing
moves the gantry while the node is down. Delete the file to forget it.

/mega/cmd_vel is not handled here: wheel_control turns it into wheel PWM
(with the IMU heading loop) and publishes /mega/wheel_pwm.

Wheel PWM is streamed at tx_rate_hz whatever arrives, and drops to 0 when
/mega/wheel_pwm goes quiet for pwm_timeout_s, so the firmware's 300 ms
failsafe only fires when this node itself is gone. Step and gripper commands
are events: each message is sent exactly once. Steps are RELATIVE - [1600, 0]
twice moves stepper 1 by 3200 in total.

All writes happen on one thread, so frames never interleave on the wire.
"""

import os
import queue
import signal
import threading
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, Int16MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

from mega_bridge import protocol as proto

FRESH_STATUS = 2    # statuses to wait for after a step before trusting steps_left == 0


class MegaBridge(Node):

    def __init__(self):
        super().__init__('mega_bridge')

        self.declare_parameters('', [
            ('port', '/dev/ttyACM0'),
            ('baud', 115200),
            ('tx_rate_hz', 50.0),       # must stay well inside the 300 ms failsafe
            ('pwm_timeout_s', 0.5),     # stale /mega/wheel_pwm -> send 0
            ('status_timeout_s', 1.0),  # warn if the Mega stops reporting
            ('home_file', '~/.ros/mega_bridge_home.txt'),   # '' = do not persist
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
        self._rem = None                      # last steps_left from the Mega
        self._target = [0, 0]                 # sum of steps sent since the Mega reset
        self._home = [0, 0]                   # target value that is home
        self._status_n = 0                    # statuses received
        self._step_n = -FRESH_STATUS          # _status_n when the last step was queued
        self._saved = None                    # position last written to home_file
        self._home_file = os.path.expanduser(self.P['home_file']) if self.P['home_file'] else ''
        self._load_home()
        self._events = queue.Queue()          # encoded STEP / SERVO frames
        self._decoder = proto.Decoder()
        self._running = True

        # ---------------- ROS I/O ----------------
        self.create_subscription(Int16MultiArray, '/mega/wheel_pwm', self.on_wheel_pwm, 10)
        self.create_subscription(Int32MultiArray, '/mega/step', self.on_step, 10)
        self.create_subscription(Int32MultiArray, '/mega/goto', self.on_goto, 10)
        self.create_subscription(Float32, '/mega/gripper', self.on_gripper, 10)
        self.status_pub = self.create_publisher(Int32MultiArray, '/mega/status', 10)
        self.create_service(Trigger, '/mega/set_home', self.on_set_home)
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
        with self._lock:
            self._queue_step(s1, s2)
        self.get_logger().info(f'step {s1} {s2}')

    def on_goto(self, msg: Int32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn(f'/mega/goto needs [x, y], got {list(msg.data)}')
            return
        x, y = int(msg.data[0]), int(msg.data[1])
        with self._lock:
            # relative to where the last command leaves the gantry, not where it is now
            s1 = x - (self._target[0] - self._home[0])
            s2 = y - (self._target[1] - self._home[1])
            if s1 or s2:
                self._queue_step(s1, s2)
        self.get_logger().info(f'goto {x} {y} (step {s1} {s2})')

    def _queue_step(self, s1, s2):
        """Call with self._lock held."""
        self._target[0] += s1
        self._target[1] += s2
        self._step_n = self._status_n
        self._events.put(proto.encode_step(s1, s2))

    def on_gripper(self, msg: Float32):
        deg = float(msg.data)
        if not 0.0 <= deg <= 180.0:
            self.get_logger().warn(f'gripper {deg:.1f} deg clamped to 0..180')
        self._events.put(proto.encode_servo(deg))

    def on_set_home(self, request, response):
        with self._lock:
            if self._rem is None:
                response.success = False
                response.message = 'no status from the Mega yet'
                return response
            if not self._idle():
                response.success = False
                response.message = (f'gantry is moving (steps left {self._rem[0]}, {self._rem[1]}),'
                                    ' wait for it to stop')
                return response
            old = self._position()
            self._home = list(self._target)   # _rx_loop saves it with the next status
        response.success = True
        response.message = f'home set, was at {old[0]}, {old[1]} steps from the old home'
        self.get_logger().info(f'set_home: {response.message}')
        return response

    # ---------------- position / home ----------------

    def _position(self):
        """Steps from home. Call with self._lock held and _rem known."""
        return [self._target[i] - self._rem[i] - self._home[i] for i in (0, 1)]

    def _idle(self):
        """Both axes stopped. Call with self._lock held."""
        # a status sent before the Mega got the last step still reads 0 left
        return (self._rem is not None and not any(self._rem)
                and self._status_n - self._step_n >= FRESH_STATUS)

    def _load_home(self):
        # The Mega just reset, so its target is 0 and the gantry is at the
        # position saved last time: put home that far behind 0.
        if not self._home_file:
            return
        try:
            with open(self._home_file) as f:
                pos = [int(v) for v in f.read().split()[:2]]
            if len(pos) != 2:
                raise ValueError('expected two integers')
        except FileNotFoundError:
            self.get_logger().warn(
                f'no saved home in {self._home_file}: home is where the gantry is now. '
                'Jog it to home and call /mega/set_home')
            return
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'cannot read {self._home_file} ({exc}), home not restored')
            return
        self._home = [-pos[0], -pos[1]]
        self._saved = pos
        self.get_logger().info(f'home restored: gantry at {pos[0]}, {pos[1]} steps from home')

    def _save_home(self, pos):
        """Only called from _rx_loop, so writes never overlap."""
        if not self._home_file or pos == self._saved:
            return
        self._saved = pos
        try:
            os.makedirs(os.path.dirname(self._home_file) or '.', exist_ok=True)
            tmp = self._home_file + '.tmp'
            with open(tmp, 'w') as f:
                f.write(f'{pos[0]} {pos[1]}\n')
            os.replace(tmp, self._home_file)          # never leave a half-written file
        except OSError as exc:
            self.get_logger().error(f'cannot save home to {self._home_file}: {exc}',
                                    throttle_duration_sec=5.0)

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
                    self._rem = (rem1, rem2)
                    self._status_n += 1
                    pos1, pos2 = self._position()
                    idle = self._idle()
                if idle:
                    self._save_home([pos1, pos2])       # no-op unless it changed
                msg = Int32MultiArray()
                msg.data = [rem1, rem2, servo, flags & proto.FLAG_FAILSAFE, pos1, pos2]
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
    node = MegaBridge()
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
