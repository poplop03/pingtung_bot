#!/usr/bin/env python3
"""
lane_runner - drive the U-shaped course: lane 1, U-turn, lane 2, stop at the goal,
steering past the pigs with the RealSense depth camera.

    /camera/camera/depth/image_rect_raw (Image, 16UC1 mm) ──┐
    /camera/camera/depth/camera_info                      ──┼──► Mission ──► /mega/cmd_vel (Twist)
    /bno055/imu (Imu, yaw rate)                           ──┘       │
                                                        web UI :8080 (START / STOP / views)
    /lane_runner/points (PointCloud2, coloured by class, only while someone listens)

The views and the point cloud run all the time, also when not driving, so the
camera can be mounted and aimed while watching them. While idle the floor is
refitted about once a second, so the camera height and tilt shown are live.

At START the robot must stand still for calib_s: the gyro bias is averaged
and the floor plane is fitted from the depth image (camera height and tilt are
logged). Yaw is the gyro integrated from there, 0 = heading at START.

wheel_control turns /mega/cmd_vel into wheel PWM and closes the yaw-rate loop.
Speeds are PWM, set in the web UI at any time (also mid-run) and kept in
settings_file: linear.x = linear_pwm / k_lin, |angular.z| <= angular_pwm / k_ff.
See mission.py for the course logic and perception.py for the depth processing.
"""

import json
import math
import os
import queue
import signal
import threading
import time
from concurrent.futures import Future, TimeoutError as FutureTimeout
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField

from lane_runner.mission import CALIBRATE, RUNNING, Config, Mission, wrap
from lane_runner.perception import (
    FLOOR, NONE, OBSTACLE, OTHER, Floor, depth_to_points, fit_floor, label_image)

REPLY_TIMEOUT_S = 2.0
VIEW_PERIOD_S = 0.2          # web views are re-rendered at most this often
IDLE_FLOOR_FIT_S = 1.0       # while idle, refit the floor this often

# BGR per label, for the camera view; RGB for the point cloud
LABEL_BGR = {NONE: (0, 0, 0), FLOOR: (80, 170, 60), OBSTACLE: (60, 60, 240), OTHER: (130, 110, 100)}


class LaneRunnerNode(Node):

    def __init__(self):
        super().__init__('lane_runner')
        defaults = Config()
        self.declare_parameters('', [
            ('depth_topic', '/camera/camera/depth/image_rect_raw'),
            ('info_topic', '/camera/camera/depth/camera_info'),
            ('imu_topic', '/bno055/imu'),
            ('imu_yaw_sign', 1.0),          # same as wheel_control's
            ('imu_timeout_s', 0.3),
            ('control_rate_hz', 20.0),
            ('stride', 2),                  # use every n-th depth pixel
            ('floor_fallback_height', 0.22),    # used only if the floor fit fails
            ('floor_fallback_pitch_deg', 12.0),
            ('dry_run', False),             # never publish /mega/cmd_vel
            ('settings_file', '~/.ros/lane_runner.json'),   # PWMs + U-turn side from the UI
            ('host', '0.0.0.0'),
            ('port', 8080),
        ] + [(name, value, ParameterDescriptor(dynamic_typing=True))
             for name, value in vars(defaults).items()])
        self.cfg = Config(**{name: type(value)(self.get_parameter(name).value)
                             for name, value in vars(defaults).items()})
        P = lambda name: self.get_parameter(name).value   # noqa: E731
        self.stride = int(P('stride'))
        self.yaw_sign = float(P('imu_yaw_sign'))
        self.imu_timeout = float(P('imu_timeout_s'))
        self.dry_run = bool(P('dry_run'))
        self.floor = Floor.from_params(float(P('floor_fallback_height')),
                                       float(P('floor_fallback_pitch_deg')))

        self._settings_file = os.path.expanduser(str(P('settings_file')))
        saved = self._load_settings()
        self.mission = Mission(self.cfg)
        self.mission.set_pwm(saved.get('linear_pwm', self.cfg.linear_pwm),
                             saved.get('angular_pwm', self.cfg.angular_pwm))
        if 'min_height' in saved and 'max_height' in saved:
            ok, why = self.mission.set_heights(saved['min_height'], saved['max_height'])
            if not ok:
                self.get_logger().warn(f'saved heights ignored: {why}')
        if isinstance(saved.get('turn_left'), bool):
            self.cfg.turn_left = saved['turn_left']
        self.K = None                       # fx, fy, cx, cy
        self.hfov = math.radians(65)
        self.last_depth = None              # latest depth in metres
        self.yaw = 0.0
        self.gyro_bias = 0.0
        self.imu_t = -math.inf
        self._imu_last = None
        self._calib_gyro = []
        self._calib_t0 = 0.0
        self._last_tick = time.monotonic()
        self._last_state = self.mission.state
        self._zero_until = 0.0

        self._requests: 'queue.Queue[tuple[dict, Future]]' = queue.Queue()
        self._lock = threading.Lock()
        self._state = {}
        self._view = None
        self._camera_view = None
        self._frame = None                  # (points, labels) of the latest depth frame
        self._frame_id = ''
        self._view_t = 0.0
        self._floor_t = 0.0
        self.floor_live = False             # the floor shown was fitted in the last seconds

        self.cmd_pub = self.create_publisher(Twist, '/mega/cmd_vel', 10)
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, P('depth_topic'), self.on_depth, sensor_qos)
        self.create_subscription(CameraInfo, P('info_topic'), self.on_info, sensor_qos)
        self.create_subscription(Imu, P('imu_topic'), self.on_imu, sensor_qos)
        self.cloud_pub = self.create_publisher(PointCloud2, '~/points', sensor_qos)
        self.create_timer(1.0 / float(P('control_rate_hz')), self.tick)

        self._web = WebServer(str(P('host')), int(P('port')), self._get_state,
                              self._get_view, self._get_camera_view, self._submit)
        self._web.start()
        self.get_logger().info(
            f"web UI at http://{P('host')}:{P('port')}/"
            + (' | DRY RUN: nothing is sent to the wheels' if self.dry_run else ''))

    # ---------------- sensors ----------------

    def on_info(self, msg: CameraInfo):
        if self.K is None:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            self.K = (fx, fy, cx, cy)
            self.hfov = 2 * math.atan(msg.width / 2 / fx)
            self.get_logger().info(
                f'depth {msg.width}x{msg.height}, fx={fx:.0f}, hfov={math.degrees(self.hfov):.0f} deg')

    def on_imu(self, msg: Imu):
        now = time.monotonic()
        wz = self.yaw_sign * float(msg.angular_velocity.z)
        if self.mission.state == CALIBRATE:
            self._calib_gyro.append(wz)
        elif self._imu_last is not None:
            self.yaw = wrap(self.yaw + (wz - self.gyro_bias) * min(now - self._imu_last, 0.1))
        self._imu_last = now
        self.imu_t = now

    def on_depth(self, msg: Image):
        if self.K is None:
            return
        if msg.encoding in ('16UC1', 'mono16'):
            d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
            depth = d[:, :msg.width].astype(np.float32) * 0.001
        elif msg.encoding == '32FC1':
            d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
            depth = np.nan_to_num(d[:, :msg.width])
        else:
            self.get_logger().error(f'unsupported depth encoding {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        self.last_depth = depth
        now = time.monotonic()
        fx, fy, cx, cy = self.K
        if self.mission.state not in RUNNING and now - self._floor_t >= IDLE_FLOOR_FIT_S:
            # idle: keep the floor live, so moving the camera shows its new height/tilt
            self._floor_t = now
            floor = fit_floor(depth_to_points(depth, fx, fy, cx, cy, stride=self.stride,
                                              row_from=0.5), iters=100)
            self.floor_live = floor is not None
            if floor is not None:
                self.floor = floor
        c = self.cfg
        pts, labels = label_image(depth, fx, fy, cx, cy, self.floor, c.min_height,
                                  c.max_height, c.max_range, stride=self.stride)
        obst = pts[labels == OBSTACLE]
        self.mission.on_depth(obst @ self.floor.f, obst @ self.floor.l, self.hfov, now)
        self._frame, self._frame_id = (pts, labels), msg.header.frame_id
        if self.cloud_pub.get_subscription_count() > 0:
            self.cloud_pub.publish(self._cloud(pts, labels, msg.header))

    # ---------------- control loop ----------------

    def tick(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        while True:
            try:
                body, future = self._requests.get_nowait()
            except queue.Empty:
                break
            future.set_result(self._handle(body, now))

        m = self.mission
        if m.state in RUNNING and now - self.imu_t > self.imu_timeout:
            m.stop(now, 'stopped: no IMU (/bno055/imu)', error=True)
        if m.state == CALIBRATE and now - self._calib_t0 >= self.cfg.calib_s:
            self._finish_calibration(now)

        out = m.tick(now, self.yaw, dt)
        if m.state != self._last_state:
            if m.state not in RUNNING:
                self._zero_until = now + 0.5
            # separate calls: rclpy refuses one call site logging at two severities
            text = f'{self._last_state} -> {m.state}: {m.message}'
            if m.error:
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)
            self._last_state = m.state
        if out is None and now < self._zero_until:
            out = (0.0, 0.0)
        if out is not None and not self.dry_run:
            twist = Twist()
            twist.linear.x, twist.angular.z = float(out[0]), float(out[1])
            self.cmd_pub.publish(twist)

        state = m.snapshot()
        state.update(yaw_deg=round(math.degrees(self.yaw), 1), dry_run=self.dry_run,
                     turn_left=self.cfg.turn_left,
                     imu_ok=now - self.imu_t < self.imu_timeout,
                     depth_ok=now - m._depth_t < self.cfg.depth_timeout_s,
                     floor={'h': round(self.floor.h, 3), 'pitch': round(self.floor.pitch_deg, 1)},
                     cmd=None if out is None else [round(out[0], 3), round(out[1], 2)])
        state.update(floor_live=self.floor_live and m.state not in RUNNING)
        if now - self._view_t >= VIEW_PERIOD_S:
            self._view_t = now
            view, camera_view = self._render(state), self._render_camera(state)
            with self._lock:
                self._view, self._camera_view = view, camera_view
        with self._lock:
            self._state = state

    def _finish_calibration(self, now):
        m = self.mission
        if len(self._calib_gyro) < 10:
            m.stop(now, 'calibration failed: no IMU data', error=True)
            return
        if self.last_depth is None:
            m.stop(now, 'calibration failed: no depth image', error=True)
            return
        self.gyro_bias = float(np.mean(self._calib_gyro))
        fx, fy, cx, cy = self.K
        floor = fit_floor(depth_to_points(self.last_depth, fx, fy, cx, cy,
                                          stride=self.stride, row_from=0.5))
        if floor is None:
            self.get_logger().warn('floor fit failed - using floor_fallback_height/pitch')
        else:
            self.floor = floor
        self.yaw = 0.0
        self.get_logger().info(
            f'calibrated: gyro bias {math.degrees(self.gyro_bias):.2f} deg/s, camera '
            f'{self.floor.h:.3f} m above the floor, tilted {self.floor.pitch_deg:.1f} deg down')
        m.calibrated(now)

    def _handle(self, body, now):
        cmd = body.get('cmd')
        if cmd == 'start':
            if self.dry_run:
                return False, 'dry run: START is disabled'
            if self.K is None:
                return False, 'no depth camera yet'
            if now - self.imu_t > self.imu_timeout:
                return False, 'no IMU (/bno055/imu)'
            self._calib_gyro, self._calib_t0 = [], now
            return self.mission.start(now)
        if cmd == 'stop':
            return self.mission.stop(now, 'stopped by user')
        if cmd == 'turn':
            if self.mission.state in RUNNING:
                return False, 'stop first'
            self.cfg.turn_left = bool(body.get('left', True))
            self._save_settings()
            return True, f"U-turn to the {'left' if self.cfg.turn_left else 'right'}"
        if cmd == 'heights':
            try:
                result = self.mission.set_heights(float(body['min']), float(body['max']))
            except (KeyError, TypeError, ValueError) as exc:
                return False, f'bad heights command: {exc}'
            if result[0]:
                self._save_settings()
                self.get_logger().info(result[1])
            return result
        if cmd == 'pwm':
            try:
                result = self.mission.set_pwm(float(body['linear']), float(body['angular']))
            except (KeyError, TypeError, ValueError) as exc:
                return False, f'bad pwm command: {exc}'
            self._save_settings()
            self.get_logger().info(result[1])
            return result
        return False, f'unknown command {cmd!r}'

    def _load_settings(self):
        try:
            with open(self._settings_file) as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'cannot read {self._settings_file}: {exc}')
            return {}

    def _save_settings(self):
        data = {'linear_pwm': self.cfg.linear_pwm, 'angular_pwm': self.cfg.angular_pwm,
                'min_height': self.cfg.min_height, 'max_height': self.cfg.max_height,
                'turn_left': self.cfg.turn_left}
        try:
            os.makedirs(os.path.dirname(self._settings_file) or '.', exist_ok=True)
            tmp = self._settings_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self._settings_file)
        except OSError as exc:
            self.get_logger().error(f'cannot save {self._settings_file}: {exc}')

    # ---------------- top-down view for the web UI ----------------

    def _render(self, state):
        """Leg frame around the robot: 2.5 m ahead, 0.5 m behind, +-0.8 m."""
        m, S = self.mission, 160                # px per metre
        w, h = int(1.6 * S), int(3.0 * S)
        img = np.full((h, w, 3), 30, np.uint8)
        px, py, th = m.pose
        g = m.grid

        def to_px(x, y):
            return int(w / 2 - (y - py) * S), int(h - (x - px + 0.5) * S)

        occ = np.argwhere(g.hits >= g.occupied)
        if len(occ):
            gx = g.x_min + (occ[:, 0] + 0.5) * g.res
            gy = g.ys[occ[:, 1]]
            u = (w / 2 - (gy - py) * S).astype(int)
            v = (h - (gx - px + 0.5) * S).astype(int)
            ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
            img[v[ok], u[ok]] = (230, 230, 230)
            for du, dv in ((1, 0), (0, 1), (1, 1)):
                img[np.clip(v[ok] + dv, 0, h - 1), np.clip(u[ok] + du, 0, w - 1)] = (230, 230, 230)
        if m.gap and state['state'].startswith('lane'):
            for yy in m.gap:
                cv2.line(img, to_px(px + 1.5, yy), to_px(px - 0.3, yy), (60, 160, 60), 1)
            cv2.line(img, to_px(px, py), to_px(px + self.cfg.pursuit_dist, m.target_y),
                     (0, 200, 255), 2)
        # robot footprint (camera at the front edge)
        c, s = math.cos(th), math.sin(th)
        L, W = self.cfg.robot_length, self.cfg.robot_width
        corners = [(0, W / 2), (0, -W / 2), (-L, -W / 2), (-L, W / 2)]
        pts = np.array([to_px(px + c * a - s * b, py + s * a + c * b) for a, b in corners])
        cv2.polylines(img, [pts], True, (60, 140, 255), 2)    # BGR orange
        # this frame's points on top: floor dim, obstacles red (robot frame -> leg frame)
        if self._frame is not None:
            fpts, labels = self._frame
            for lab, colour in ((FLOOR, (90, 70, 60)), (OBSTACLE, (60, 60, 240))):
                q = fpts[labels == lab][::2 if lab == FLOOR else 1]
                if not len(q):
                    continue
                rx, ry = q @ self.floor.f, q @ self.floor.l
                gx, gy = px + c * rx - s * ry, py + s * rx + c * ry
                u = (w / 2 - (gy - py) * S).astype(int)
                v = (h - (gx - px + 0.5) * S).astype(int)
                ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
                img[v[ok], u[ok]] = colour
        cv2.putText(img, state['state'], (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return jpg.tobytes() if ok else None

    def _render_camera(self, state):
        """The depth image as the camera sees it, coloured by label, brighter = closer."""
        if self._frame is None:
            return None
        pts, labels = self._frame
        img = np.zeros(labels.shape + (3,), np.uint8)
        for lab, colour in LABEL_BGR.items():
            img[labels == lab] = colour
        shade = np.clip(1.4 - pts[..., 2] / 2.5, 0.35, 1.0)[..., None]
        img = (img * shade).astype(np.uint8)
        scale = max(1, 424 // img.shape[1])
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        n = labels.size
        text = (f"floor {100 * (labels == FLOOR).sum() / n:.0f}%  "
                f"obstacle {100 * (labels == OBSTACLE).sum() / n:.0f}%  "
                f"cam {state['floor']['h']:.2f} m, {state['floor']['pitch']:.0f} deg  "
                f"band {100 * self.cfg.min_height:.0f}-{100 * self.cfg.max_height:.0f} cm")
        cv2.putText(img, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return jpg.tobytes() if ok else None

    def _cloud(self, pts, labels, header):
        """PointCloud2 in the depth camera's optical frame, rgb = label colour."""
        keep = labels != NONE
        p = pts[keep].astype(np.float32)
        rgb = np.zeros(len(p), np.uint32)
        for lab, (b, g, r) in LABEL_BGR.items():
            rgb[labels[keep] == lab] = (r << 16) | (g << 8) | b
        data = np.empty(len(p), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4')])
        data['x'], data['y'], data['z'], data['rgb'] = p[:, 0], p[:, 1], p[:, 2], rgb
        msg = PointCloud2(header=header, height=1, width=len(p), is_bigendian=False,
                          point_step=16, row_step=16 * len(p), is_dense=True)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate(('x', 'y', 'z', 'rgb'))]
        msg.data = data.tobytes()
        return msg

    # ---------------- web server side ----------------

    def _get_state(self):
        with self._lock:
            return self._state

    def _get_view(self):
        with self._lock:
            return self._view

    def _get_camera_view(self):
        with self._lock:
            return self._camera_view

    def _submit(self, body):
        future: Future = Future()
        self._requests.put((body, future))
        try:
            return future.result(timeout=REPLY_TIMEOUT_S)
        except FutureTimeout:
            return False, 'robot did not answer'

    def destroy_node(self):
        self._web.shutdown()
        if rclpy.ok() and not self.dry_run:
            self.cmd_pub.publish(Twist())
        super().destroy_node()


INDEX_HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lane Runner</title>
<style>
  :root { --bg:#111; --panel:#1c1c1c; --line:#333; --fg:#ddd; --dim:#888;
          --go:#2e7d32; --stop:#c62828; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg); font-family:system-ui,sans-serif; font-size:14px; }
  main { max-width:560px; margin:auto; padding:12px 16px; }
  section { background:var(--panel); border:1px solid var(--line); border-radius:6px;
            padding:10px 12px; margin-bottom:12px; }
  .row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  button { background:#2a2a2a; color:var(--fg); border:1px solid #444; border-radius:5px;
           padding:8px 10px; font-size:14px; }
  button:disabled { opacity:.35; }
  .big { font-size:20px; font-weight:600; padding:14px; flex:1; }
  #start { background:var(--go); } #stop { background:var(--stop); }
  #msg { margin-top:8px; padding:6px 8px; border-radius:4px; background:#222; min-height:1.6em; }
  #msg.err { background:#4a1515; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:3px 12px; margin-top:8px; }
  .kv span:nth-child(odd) { color:var(--dim); }
  .num { font-variant-numeric:tabular-nums; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%; margin-right:4px; background:var(--stop); }
  .dot.ok { background:var(--go); }
  #view, #camview { display:block; margin:auto; max-width:100%; border:1px solid var(--line); border-radius:4px; }
  #camview { width:100%; image-rendering:pixelated; }
  .hint { color:var(--dim); font-size:12px; margin-top:6px; }
  h2 { font-size:13px; text-transform:uppercase; letter-spacing:.05em; color:var(--dim); margin:0 0 8px; }
  .numrow { display:grid; grid-template-columns:58px 42px 42px minmax(0,1fr) 42px 42px; gap:5px;
            align-items:center; margin-bottom:6px; }
  .numrow input { width:100%; text-align:center; }
  .step { padding:8px 0; }
  input.pending { border-color:#f9a825; background:#3a2e10; }
  input[type=number], select { background:#222; color:var(--fg); border:1px solid #444;
                               border-radius:4px; padding:6px; font-size:14px; }
</style></head>
<body><main>
  <section>
    <div class="row">
      <button id="start" class="big">START</button>
      <button id="stop" class="big">STOP</button>
    </div>
    <div id="msg"></div>
    <div class="kv">
      <span>state</span><span id="state">-</span>
      <span>U-turn</span><span><select id="turn"><option value="1">left</option>
        <option value="0">right</option></select></span>
      <span>yaw</span><span id="yaw" class="num">-</span>
      <span>travel</span><span id="travel" class="num">-</span>
      <span>ahead</span><span id="front" class="num">-</span>
      <span>command</span><span id="cmd" class="num">-</span>
      <span>camera</span><span id="floor" class="num">-</span>
      <span>health</span><span><span id="imu" class="dot"></span>IMU
        <span id="depth" class="dot"></span>depth</span>
    </div>
    <div class="hint">START: keep the robot still ~1.5 s while it calibrates. Space = STOP.</div>
  </section>
  <section>
    <h2>PWM</h2>
    <div class="numrow"><span>linear</span>
      <button class="step" data-for="linn" data-d="-5">&minus;5</button><button class="step" data-for="linn" data-d="-1">&minus;1</button>
      <input id="linn" type="number" inputmode="numeric" min="0" max="255" step="1" value="0">
      <button class="step" data-for="linn" data-d="1">+1</button><button class="step" data-for="linn" data-d="5">+5</button></div>
    <div class="numrow"><span>angular</span>
      <button class="step" data-for="angn" data-d="-5">&minus;5</button><button class="step" data-for="angn" data-d="-1">&minus;1</button>
      <input id="angn" type="number" inputmode="numeric" min="0" max="100" step="1" value="0">
      <button class="step" data-for="angn" data-d="1">+1</button><button class="step" data-for="angn" data-d="5">+5</button></div>
    <div class="row">
      <button id="pwmset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="pwmcur" class="num">-</span></span>
    </div>
    <div class="hint">Nothing changes until you press Set: an edited value is shown in
      orange until then. Applies at once, also while running, and is kept for next time.
      Linear: forward PWM in the lanes and the U-turn. Angular: the fastest turn
      (dodging pigs and the U-turn pivots). If a pivot stalls, raise angular.</div>
  </section>
  <section>
    <h2>Obstacle height band</h2>
    <div class="numrow"><span>bottom</span>
      <button class="step" data-for="hlon" data-d="-5">&minus;5</button><button class="step" data-for="hlon" data-d="-1">&minus;1</button>
      <input id="hlon" type="number" inputmode="numeric" min="0" max="200" step="1" value="4">
      <button class="step" data-for="hlon" data-d="1">+1</button><button class="step" data-for="hlon" data-d="5">+5</button></div>
    <div class="numrow"><span>top</span>
      <button class="step" data-for="hhin" data-d="-5">&minus;5</button><button class="step" data-for="hhin" data-d="-1">&minus;1</button>
      <input id="hhin" type="number" inputmode="numeric" min="1" max="200" step="1" value="50">
      <button class="step" data-for="hhin" data-d="1">+1</button><button class="step" data-for="hhin" data-d="5">+5</button></div>
    <div class="row">
      <button id="hset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="hcur" class="num">-</span></span>
    </div>
    <div class="hint">Centimetres above the floor. Only points in this band are obstacles
      (red); below it is floor, above it is ignored (grey). Watch the camera view while
      you adjust: raise the bottom if floor noise turns red, lower the top if things
      behind the walls turn red. Keep the bottom under the pigs' height (~9 cm). Applies
      at once, also while running.</div>
  </section>
  <section>
    <h2>Camera</h2>
    <img id="camview" alt="depth camera view">
    <div class="hint">What the depth camera sees. Green = floor, red = obstacle (walls,
      pigs), grey = too high / too far, black = no depth. Brighter = closer. Live also
      when not running: aim the camera so the lane floor and both walls are in view.</div>
  </section>
  <section>
    <h2>Top-down</h2>
    <img id="view" alt="top-down view">
    <div class="hint">Robot (orange) driving up. Red = obstacles in this frame, dim blue =
      floor, white = walls/pigs remembered, green = the free gap, yellow = where it steers.</div>
  </section>
</main>
<script>
const $ = (id) => document.getElementById(id);
async function cmd(body) {
  try {
    const r = await fetch('/api/cmd', {method:'POST', headers:{'Content-Type':'application/json'},
                                       body: JSON.stringify(body)});
    const res = await r.json();
    $('msg').textContent = res.message; $('msg').className = res.ok ? '' : 'err';
  } catch (e) { $('msg').textContent = 'connection lost'; $('msg').className = 'err'; }
}
async function poll() {
  try {
    const s = await (await fetch('/api/state')).json();
    $('state').textContent = s.state + (s.dry_run ? ' (dry run)' : '');
    $('msg').textContent = s.message; $('msg').className = s.error ? 'err' : '';
    $('yaw').textContent = s.yaw_deg + ' deg';
    $('travel').textContent = s.travel + ' m';
    $('front').textContent = s.front === null ? '-' : s.front + ' m';
    $('cmd').textContent = s.cmd ? 'v ' + s.cmd[0] + ', w ' + s.cmd[1] : '-';
    $('floor').textContent = s.floor.h + ' m high, ' + s.floor.pitch + ' deg down'
      + (s.running ? ' (fixed at START)' : s.floor_live ? ' (live)' : ' (no floor found)');
    $('imu').className = 'dot' + (s.imu_ok ? ' ok' : '');
    $('depth').className = 'dot' + (s.depth_ok ? ' ok' : '');
    $('start').disabled = s.running || s.dry_run;
    $('turn').disabled = s.running;
    $('turn').value = s.turn_left ? '1' : '0';
    $('pwmcur').textContent = 'linear ' + s.linear_pwm.toFixed(0) + ', angular ' + s.angular_pwm.toFixed(0);
    $('linn').max = s.max_linear_pwm;
    $('angn').max = s.max_angular_pwm;
    const lo = Math.round(s.min_height * 100), hi = Math.round(s.max_height * 100);
    $('hcur').textContent = lo + ' to ' + hi + ' cm';
    applied.linn = Math.round(s.linear_pwm); applied.angn = Math.round(s.angular_pwm);
    applied.hlon = lo; applied.hhin = hi;
    Object.keys(applied).forEach((id) => {
      if (!loaded[id]) { loaded[id] = true; $(id).value = applied[id]; }
      markPending(id);
    });
  } catch (e) { $('state').textContent = 'no connection'; }
  $('view').src = '/view.jpg?' + Date.now();
  $('camview').src = '/camera.jpg?' + Date.now();
}
// Number boxes with -5/-1/+1/+5 buttons, no sliders: a slider on a phone moves when
// you scroll across it, and that once sent angular 223 mid-run. An edited box stays
// orange until Set sends it; the robot's applied value is loaded once, then after Set.
const applied = {}, loaded = {};
const val = (id) => {
  const e = $(id);
  return Math.max(+e.min, Math.min(+e.max, Math.round(+e.value || 0)));
};
function markPending(id) {
  $(id).classList.toggle('pending', applied[id] !== undefined && val(id) !== applied[id]);
}
document.querySelectorAll('.step').forEach((b) => b.onclick = () => {
  const e = $(b.dataset.for);
  e.value = Math.max(+e.min, Math.min(+e.max, (+e.value || 0) + +b.dataset.d));
  markPending(b.dataset.for);
});
['linn', 'angn', 'hlon', 'hhin'].forEach((id) => {
  $(id).oninput = () => markPending(id);
  $(id).onkeydown = (e) => {
    if (e.key === 'Enter') $(id === 'linn' || id === 'angn' ? 'pwmset' : 'hset').click();
  };
});
$('hset').onclick = async () => {
  const lo = val('hlon'), hi = val('hhin');
  document.activeElement.blur();
  if (hi <= lo) { $('msg').textContent = 'top must be above bottom'; $('msg').className = 'err'; return; }
  await cmd({cmd: 'heights', min: lo / 100, max: hi / 100});
  loaded.hlon = loaded.hhin = false;       // show what the robot applied
};
$('pwmset').onclick = async () => {
  const l = val('linn'), a = val('angn');
  document.activeElement.blur();
  await cmd({cmd: 'pwm', linear: l, angular: a});
  loaded.linn = loaded.angn = false;
};
setInterval(poll, 300); poll();
$('start').onclick = () => cmd({cmd:'start'});
$('stop').onclick = () => cmd({cmd:'stop'});
$('turn').onchange = () => cmd({cmd:'turn', left: $('turn').value === '1'});
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === ' ') { e.preventDefault(); $('stop').click(); }
});
</script></body></html>
"""


class WebServer:
    def __init__(self, host, port, get_state, get_view, get_camera_view, submit):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                if self.path in ('/', '/index.html'):
                    self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
                elif self.path == '/api/state':
                    self._send(200, 'application/json', json.dumps(get_state()).encode())
                elif self.path.startswith(('/view.jpg', '/camera.jpg')):
                    jpg = get_view() if self.path.startswith('/view') else get_camera_view()
                    self._send(200, 'image/jpeg', jpg) if jpg else self._send(503, 'text/plain', b'-')
                else:
                    self._send(404, 'text/plain', b'not found')

            def do_POST(self):  # noqa: N802
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0))) or b'{}')
                    if not isinstance(body, dict):
                        raise ValueError('body must be a JSON object')
                except ValueError as exc:
                    self._send(400, 'application/json',
                               json.dumps({'ok': False, 'message': str(exc)}).encode())
                    return
                ok, message = submit(body)
                self._send(200, 'application/json',
                           json.dumps({'ok': ok, 'message': message}).encode())

            def _send(self, status, ctype, data):
                self.send_response(status)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, format, *args):  # noqa: A002
                pass

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)

    def start(self):
        self._thread.start()

    def shutdown(self):
        self._server.shutdown()
        self._server.server_close()


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
    node = LaneRunnerNode()
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
