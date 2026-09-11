#!/usr/bin/env python3
"""
wheel_control - differential base driven open-loop in speed, closed-loop in yaw.

    /cmd_vel  (geometry_msgs/Twist)  ---.
                                        +--> [heading hold] -> [yaw-rate PI] -> PWM
    /imu/data (sensor_msgs/Imu)     ---'

WHY TWO LOOPS

A yaw-RATE loop alone cannot drive straight. Holding w = 0 means "do not
rotate right now"; any error the loop has not yet corrected has already been
integrated into heading and is never recovered. The robot stops curving but
keeps the heading it drifted to. So the rate loop is wrapped in a slow
heading-hold loop that integrates the commanded and measured yaw and feeds
the difference back as a small rate bias. That outer loop is what makes
"forward" mean a straight line.

CONTROL LAW (per cycle)

    yaw_ref  += w_d * dt
    yaw_meas += w_meas * dt
    w_cmd     = w_d + clamp(k_heading * wrap(yaw_ref - yaw_meas), +/- max_corr)

    u         = k_ff * w_cmd + PI(w_cmd - w_meas)      # differential PWM
    base      = base_pwm * sign(v)                     # 0 when v == 0

    pwm_left  = base - u        pwm_right = base + u

Sign convention is REP-103: +x forward, +z up, so +w is counter-clockwise
(turning left), which means the RIGHT wheel speeds up. If your robot turns
the wrong way, flip `imu_yaw_sign` first, then `swap_motors` - flipping both
hides the problem instead of fixing it.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from wheel_control.motor_link import MotorLink
from wheel_control.pid import PID

# operating modes
MODE_STOP, MODE_DRIVE, MODE_SPIN = 'stop', 'drive', 'spin'


def wrap_pi(angle: float) -> float:
    """Wrap to (-pi, pi]. Without this, heading error explodes after a few turns."""
    return math.atan2(math.sin(angle), math.cos(angle))


class WheelControl(Node):

    def __init__(self):
        super().__init__('wheel_control')

        # ---------------- parameters ----------------
        self.declare_parameters('', [
            # serial
            ('port', '/dev/ttyUSB0'),
            ('baud', 115200),
            ('tx_rate_hz', 50.0),

            # control
            ('control_rate_hz', 50.0),
            ('base_pwm', 80),            # open-loop forward/back effort
            ('min_pwm', 45),             # below this the wheels stall (stiction)
            ('max_pwm', 255),
            ('deadband_mode', 'floor'),  # 'floor' | 'rescale'
            ('pwm_slew_per_s', 400.0),   # limits current surge on a step command

            # yaw-rate loop (units: PWM counts per rad/s)
            ('kp', 45.0),
            ('ki', 60.0),
            ('kd', 0.0),                 # start at 0 - see the README
            ('kd_tau', 0.08),
            ('k_ff', 35.0),              # feedforward, does most of the work
            ('i_limit', 90.0),
            ('u_limit', 175.0),          # 255 - base_pwm, the real headroom

            # heading-hold outer loop
            ('heading_hold', True),
            ('k_heading', 1.2),          # rad/s of correction per rad of error
            ('max_heading_corr', 0.6),   # rad/s

            # IMU
            ('imu_topic', '/imu/data'),
            ('imu_yaw_sign', 1.0),       # -1.0 if the BNO055 is mounted inverted
            ('gyro_deadband', 0.01),     # rad/s, below this treat as zero
            ('calib_samples', 200),      # stationary samples for bias estimate
            ('imu_timeout_s', 0.3),

            # plumbing
            ('cmd_timeout_s', 0.5),
            ('swap_motors', False),
            ('invert_motor1', False),
            ('invert_motor2', False),
            ('publish_debug', True),
        ])
        self.P = {name: self.get_parameter(name).value
                  for name in self._parameters
                  if name not in ('use_sim_time',)}

        # ---------------- state ----------------
        self.v_cmd = 0.0            # m/s-ish; only its sign is used
        self.w_cmd_in = 0.0         # rad/s, from /cmd_vel
        self.last_cmd_t = self.get_clock().now()

        self.w_meas = 0.0           # bias-corrected yaw rate, rad/s
        self.last_imu_t = None
        self.imu_ok = False

        self.gyro_bias = 0.0
        self.calib_buf = []
        self.calibrated = False

        self.yaw_ref = 0.0
        self.yaw_meas = 0.0
        self.mode = MODE_STOP
        self.prev_mode = MODE_STOP

        self.pwm_l = 0.0
        self.pwm_r = 0.0
        self.stationary_since = None

        self.pid = PID(
            kp=float(self.P['kp']), ki=float(self.P['ki']), kd=float(self.P['kd']),
            out_min=-float(self.P['u_limit']), out_max=float(self.P['u_limit']),
            tau=float(self.P['kd_tau']), i_limit=float(self.P['i_limit']),
        )

        # ---------------- I/O ----------------
        try:
            self.link = MotorLink(self.P['port'], int(self.P['baud']),
                                  float(self.P['tx_rate_hz']), logger=self.get_logger())
        except Exception as exc:
            self.get_logger().fatal(f"cannot open {self.P['port']}: {exc}")
            raise

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Imu, self.P['imu_topic'], self.on_imu,
                                 QoSPresetProfiles.SENSOR_DATA.value)

        self.dbg_pub = (self.create_publisher(
            Float32MultiArray, '~/debug', 10) if self.P['publish_debug'] else None)

        self.dt = 1.0 / float(self.P['control_rate_hz'])
        self.create_timer(self.dt, self.on_control)

        self.get_logger().info(
            f"wheel_control up on {self.P['port']} | base_pwm={self.P['base_pwm']} | "
            f"hold the robot still for gyro bias calibration "
            f"({self.P['calib_samples']} samples)")

    # ---------------- callbacks ----------------

    def on_cmd_vel(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd_in = float(msg.angular.z)
        self.last_cmd_t = self.get_clock().now()

    def on_imu(self, msg: Imu):
        raw = float(msg.angular_velocity.z) * float(self.P['imu_yaw_sign'])
        self.last_imu_t = self.get_clock().now()
        self.imu_ok = True

        if not self.calibrated:
            self.calib_buf.append(raw)
            if len(self.calib_buf) >= int(self.P['calib_samples']):
                self.gyro_bias = sum(self.calib_buf) / len(self.calib_buf)
                spread = max(self.calib_buf) - min(self.calib_buf)
                self.calibrated = True
                self.calib_buf.clear()
                self.get_logger().info(
                    f"gyro bias = {self.gyro_bias:+.4f} rad/s "
                    f"({math.degrees(self.gyro_bias):+.2f} deg/s), "
                    f"noise spread {math.degrees(spread):.2f} deg/s")
                if abs(self.gyro_bias) > 0.05:
                    self.get_logger().warn(
                        "large gyro bias - was the robot moving during calibration?")
            return

        w = raw - self.gyro_bias
        if abs(w) < float(self.P['gyro_deadband']):
            w = 0.0
        self.w_meas = w

    # ---------------- control loop ----------------

    def on_control(self):
        now = self.get_clock().now()

        # --- safety gates, most severe first ---
        imu_stale = (self.last_imu_t is None or
                     (now - self.last_imu_t).nanoseconds * 1e-9 > float(self.P['imu_timeout_s']))
        cmd_stale = (now - self.last_cmd_t).nanoseconds * 1e-9 > float(self.P['cmd_timeout_s'])

        if imu_stale:
            # No feedback means no closed loop. Driving open-loop on a base PWM
            # with no yaw control is how a robot walks into a wall sideways.
            self.stop('imu timeout' if self.imu_ok else None)
            return
        if not self.calibrated:
            self.stop(None)
            return
        if cmd_stale:
            self.stop(None)
            return

        v, w_d = self.v_cmd, self.w_cmd_in

        # --- mode ---
        moving = abs(v) > 1e-3
        turning = abs(w_d) > 1e-3
        mode = MODE_DRIVE if moving else (MODE_SPIN if turning else MODE_STOP)

        if mode != self.prev_mode:
            # Re-seed the heading reference on every mode change, otherwise the
            # error accumulated while stopped gets dumped into the wheels the
            # instant you command motion.
            self.pid.reset()
            self.yaw_ref = self.yaw_meas
            self.prev_mode = mode
        self.mode = mode

        if mode == MODE_STOP:
            self.stop(None)
            self.maybe_rezero_bias(now)
            return
        self.stationary_since = None

        # --- integrate both yaws ---
        self.yaw_meas = wrap_pi(self.yaw_meas + self.w_meas * self.dt)
        self.yaw_ref = wrap_pi(self.yaw_ref + w_d * self.dt)

        # --- outer loop: heading hold ---
        heading_err = 0.0
        w_sp = w_d
        if self.P['heading_hold']:
            heading_err = wrap_pi(self.yaw_ref - self.yaw_meas)
            corr = float(self.P['k_heading']) * heading_err
            limit = float(self.P['max_heading_corr'])
            corr = max(-limit, min(limit, corr))
            w_sp = w_d + corr

        # --- inner loop: yaw rate ---
        ff = float(self.P['k_ff']) * w_sp
        u = self.pid.update(w_sp, self.w_meas, self.dt, feedforward=ff)

        # --- mixing ---
        base = float(self.P['base_pwm']) * (1.0 if v > 0 else -1.0) if moving else 0.0
        target_l = base - u
        target_r = base + u

        self.pwm_l = self.slew(self.pwm_l, target_l)
        self.pwm_r = self.slew(self.pwm_r, target_r)

        out_l = self.shape(self.pwm_l)
        out_r = self.shape(self.pwm_r)
        self.send(out_l, out_r)
        self.publish_debug(w_d, w_sp, u, heading_err, out_l, out_r)

    # ---------------- helpers ----------------

    def slew(self, current: float, target: float) -> float:
        step = float(self.P['pwm_slew_per_s']) * self.dt
        if target > current + step:
            return current + step
        if target < current - step:
            return current - step
        return target

    def shape(self, pwm: float) -> int:
        """Clamp, then lift the output out of the stiction deadband.

        A wheel commanded 12/255 does nothing but heat the motor, so any
        non-zero command is pushed up to min_pwm.

        'floor' (default) keeps base_pwm meaning exactly what you set - an 80
        stays an 80 - at the cost of a small step as the output crosses
        min_pwm. 'rescale' maps the whole range into [min_pwm, max_pwm] for a
        smooth response, but then base_pwm=80 actually leaves as ~110.
        """
        lo, hi = float(self.P['min_pwm']), float(self.P['max_pwm'])
        mag = min(abs(pwm), hi)
        if mag < 1.0:
            return 0
        if self.P['deadband_mode'] == 'rescale':
            mag = lo + (mag / hi) * (hi - lo)
        else:
            mag = max(mag, lo)
        return int(math.copysign(min(mag, hi), pwm))

    def send(self, left: int, right: int):
        m1, m2 = (right, left) if self.P['swap_motors'] else (left, right)
        if self.P['invert_motor1']:
            m1 = -m1
        if self.P['invert_motor2']:
            m2 = -m2
        self.link.set(m1, m2)

    def stop(self, reason):
        if reason is not None:
            self.get_logger().warn(reason, throttle_duration_sec=2.0)
        self.pwm_l = self.pwm_r = 0.0
        self.pid.reset()
        self.yaw_ref = self.yaw_meas
        self.prev_mode = MODE_STOP
        self.link.set(0, 0)          # firmware runs its own ramped stop

    def maybe_rezero_bias(self, now):
        """Slowly re-estimate gyro bias while genuinely parked.

        BNO055 bias walks with temperature, and the heading loop integrates
        that walk directly into drift. Re-zeroing whenever the robot has been
        commanded to stop for a while keeps it honest.
        """
        if self.stationary_since is None:
            self.stationary_since = now
            return
        if (now - self.stationary_since).nanoseconds * 1e-9 < 1.5:
            return
        raw = self.w_meas + self.gyro_bias
        self.gyro_bias += 0.002 * (raw - self.gyro_bias)   # slow first-order fit

    def publish_debug(self, w_d, w_sp, u, heading_err, out_l, out_r):
        if self.dbg_pub is None:
            return
        msg = Float32MultiArray()
        # w_d, w_setpoint, w_measured, u, P, I, D, heading_err, pwm_l, pwm_r
        msg.data = [float(w_d), float(w_sp), float(self.w_meas), float(u),
                    float(self.pid.p_term), float(self.pid.i_term),
                    float(self.pid.d_term), float(heading_err),
                    float(out_l), float(out_r)]
        self.dbg_pub.publish(msg)

    def destroy_node(self):
        try:
            self.link.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WheelControl()
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
