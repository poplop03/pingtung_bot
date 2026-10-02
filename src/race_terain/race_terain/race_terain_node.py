#!/usr/bin/env python3
"""
race_terain - drive the all-terrain road to the finish by following the centre
line between the two rows of things that line it (posts, barriers, reflectors).

    /camera/camera/depth/image_rect_raw (Image, 16UC1 mm) ──┐
    /camera/camera/depth/camera_info                      ──┼──► Tracker ──► /mega/cmd_vel (Twist)
    /bno055/imu (Imu, yaw rate)                           ──┘
                                                    web UI :8080 (START / STOP / views)

At START the robot must stand still for calib_s while the gyro bias is
averaged. Yaw is only used to turn the short point memory with the robot.

The ground plane is refitted on every depth frame, because on rough terrain
the robot (and so the camera) keeps pitching and rolling. A fit that jumps
too far from the one taken at START (a hill face filling the view, a dusty
frame) is ignored and the previous ground is kept.

The views run all the time, also when not driving, so the camera can be
aimed and the height band tuned while watching them. See tracker.py for the
steering and perception.py for the depth processing.
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
from sensor_msgs.msg import CameraInfo, Image, Imu

from race_terain.perception import (
    FLOOR, NONE, OBJECT, OTHER, Floor, depth_to_points, fit_floor, label_image)
from race_terain.tracker import CALIBRATE, RUNNING, STRAIGHT, Config, Tracker, wrap

REPLY_TIMEOUT_S = 2.0
VIEW_PERIOD_S = 0.2          # web views are re-rendered at most this often

LABEL_BGR = {NONE: (0, 0, 0), FLOOR: (80, 170, 60), OBJECT: (60, 60, 240), OTHER: (130, 110, 100)}


class RaceTerainNode(Node):

    def __init__(self):
        super().__init__('race_terain')
        defaults = Config()
        self.declare_parameters('', [
            ('depth_topic', '/camera/camera/depth/image_rect_raw'),
            ('info_topic', '/camera/camera/depth/camera_info'),
            ('stride', 2),                       # use every n-th depth pixel
            ('imu_topic', '/bno055/imu'),
            ('imu_yaw_sign', 1.0),               # same as wheel_control's
            ('imu_timeout_s', 0.3),
            ('control_rate_hz', 20.0),
            ('floor_iters', 60),                 # RANSAC iterations per frame
            ('floor_max_change_deg', 20.0),      # per-frame ground fit vs the one at START
            ('floor_max_change_m', 0.10),
            ('floor_fallback_height', 0.22),     # only if no ground was ever found
            ('floor_fallback_pitch_deg', 12.0),
            ('dry_run', False),                  # never publish /mega/cmd_vel
            ('settings_file', '~/.ros/race_terain.json'),
            ('host', '0.0.0.0'),
            ('port', 8080),
        ] + [(name, value, ParameterDescriptor(dynamic_typing=True))
             for name, value in vars(defaults).items()])
        self.cfg = Config(**{name: type(value)(self.get_parameter(name).value)
                             for name, value in vars(defaults).items()})
        P = lambda name: self.get_parameter(name).value   # noqa: E731
        self.stride = int(P('stride'))
        self.floor_iters = int(P('floor_iters'))
        self.floor_max_deg = float(P('floor_max_change_deg'))
        self.floor_max_m = float(P('floor_max_change_m'))
        self.dry_run = bool(P('dry_run'))
        self.yaw_sign = float(P('imu_yaw_sign'))
        self.imu_timeout = float(P('imu_timeout_s'))
        self.floor = Floor.from_params(float(P('floor_fallback_height')),
                                       float(P('floor_fallback_pitch_deg')))
        self.floor_ref = None               # ground at START; per-frame fits must stay near it
        self.floor_ok = False               # this frame's ground fit was used

        self._settings_file = os.path.expanduser(str(P('settings_file')))
        saved = self._load_settings()
        self.tracker = Tracker(self.cfg)
        self.tracker.set_pwm(saved.get('linear_pwm', self.cfg.linear_pwm),
                             saved.get('angular_pwm', self.cfg.angular_pwm))
        if 'min_height' in saved and 'max_height' in saved:
            ok, why = self.tracker.set_heights(saved['min_height'], saved['max_height'])
            if not ok:
                self.get_logger().warn(f'saved heights ignored: {why}')
        if 'min_range' in saved and 'max_range' in saved:
            ok, why = self.tracker.set_range(saved['min_range'], saved['max_range'])
            if not ok:
                self.get_logger().warn(f'saved valid distance ignored: {why}')
        if 'straight_ms' in saved:
            self.tracker.set_straight(saved['straight_ms'])

        self.hfov = math.radians(65)        # replaced by camera_info
        self.cloud_t = -math.inf
        self._depth = None                  # latest depth image (m) for the camera view
        self._K = None                      # its fx, fy, cx, cy
        self.yaw = 0.0
        self.gyro_bias = 0.0
        self.imu_t = -math.inf
        self._imu_last = None
        self._calib_gyro = []
        self._calib_t0 = 0.0
        self._last_tick = time.monotonic()
        self._last_state = self.tracker.state
        self._straight_timer = None         # one-shot: ends the straight phase on time
        self._zero_until = 0.0
        self._requests: 'queue.Queue[tuple[dict, Future]]' = queue.Queue()
        self._lock = threading.Lock()
        self._state = {}
        self._view = None
        self._camera_view = None
        self._frame = None                  # (points, labels) of the latest depth frame
        self._view_t = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/mega/cmd_vel', 10)
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, P('depth_topic'), self.on_depth_image, sensor_qos)
        self.create_subscription(CameraInfo, P('info_topic'), self.on_info, sensor_qos)
        # a deep queue: no IMU message may be dropped while a point cloud is processed
        imu_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, P('imu_topic'), self.on_imu, imu_qos)
        self.create_timer(1.0 / float(P('control_rate_hz')), self.tick)

        self._web = WebServer(str(P('host')), int(P('port')), self._get_state,
                              self._get_view, self._get_camera_view, self._submit)
        self._web.start()
        self.get_logger().info(
            f"web UI at http://{P('host')}:{P('port')}/"
            + (' | DRY RUN: nothing is sent to the wheels' if self.dry_run else ''))

    # ---------------- sensors ----------------

    def on_imu(self, msg: Imu):
        # Integrate on the message's own time stamp: a point cloud can keep this
        # (single) thread busy for a while, and the queued IMU messages then arrive
        # in a burst. Arrival times would squash those steps to nothing.
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = stamp if stamp > 0 else now
        wz = self.yaw_sign * float(msg.angular_velocity.z)
        if self.tracker.state == CALIBRATE:
            self._calib_gyro.append(wz)
        elif self._imu_last is not None and 0.0 < t - self._imu_last < 0.5:
            self.yaw = wrap(self.yaw + (wz - self.gyro_bias) * (t - self._imu_last))
        self._imu_last = t
        self.imu_t = now

    def on_info(self, msg: CameraInfo):
        if self._K is None:
            self._K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.hfov = 2 * math.atan(msg.width / 2 / msg.k[0])
            self.get_logger().info(f'depth {msg.width}x{msg.height}, fx={msg.k[0]:.0f}, '
                                   f'hfov={math.degrees(self.hfov):.0f} deg')

    def on_depth_image(self, msg: Image):
        """Depth image -> point cloud (as lane_runner does) -> ground, objects, tracker."""
        if self._K is None:
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
        now = time.monotonic()
        self.cloud_t = now
        self._depth = depth
        fx, fy, cx, cy = self._K
        # ground: refitted every frame from the lower half of the image
        floor = fit_floor(depth_to_points(depth, fx, fy, cx, cy, stride=4, row_from=0.5),
                          iters=self.floor_iters)
        ref = self.floor_ref if self.tracker.state in RUNNING else None
        self.floor_ok = floor is not None and (
            ref is None or floor.close_to(ref, self.floor_max_deg, self.floor_max_m))
        if self.floor_ok:
            self.floor = floor
        c = self.cfg
        pts, labels = label_image(depth, fx, fy, cx, cy, self.floor, c.min_height,
                                  c.max_height, c.max_range, stride=self.stride,
                                  min_range=c.min_range)
        obj = pts[labels == OBJECT]
        self.tracker.on_depth(obj @ self.floor.f, obj @ self.floor.l, self.hfov, now)
        self._frame = (pts, labels)

    # ---------------- control loop ----------------

    def tick(self):
        now = time.monotonic()
        dt, self._last_tick = min(now - self._last_tick, 0.2), now
        while True:
            try:
                body, future = self._requests.get_nowait()
            except queue.Empty:
                break
            future.set_result(self._handle(body, now))

        t = self.tracker
        if t.state in RUNNING and now - self.imu_t > self.imu_timeout:
            t.stop(now, 'stopped: no IMU (/bno055/imu)', error=True)
        if t.state == CALIBRATE and now - self._calib_t0 >= self.cfg.calib_s:
            self._finish_calibration(now)
        out = t.tick(now, self.yaw, dt)
        if t.state == STRAIGHT and self._straight_timer is None:
            # the 20 Hz tick alone could end the straight phase up to a period late
            self._straight_timer = self.create_timer(max(t.straight_end - now, 0.001),
                                                     self._end_straight)
        elif t.state != STRAIGHT and self._straight_timer is not None:
            self.destroy_timer(self._straight_timer)     # stopped / restarted meanwhile
            self._straight_timer = None
        if self._last_state == STRAIGHT and t.state != STRAIGHT and t.straight_done_ms is not None:
            self.get_logger().info(f'straight phase: {t.straight_done_ms:.0f} ms '
                                   f'(set {self.cfg.straight_ms:.0f} ms) - controller engaged')
            t.straight_done_ms = None
        if t.state != self._last_state:
            if t.state not in RUNNING:
                self._zero_until = now + 0.5
            # separate calls: rclpy refuses one call site logging at two severities
            text = f'{self._last_state} -> {t.state}: {t.message}'
            if t.error:
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)
            self._last_state = t.state
        if out is None and now < self._zero_until:
            out = (0.0, 0.0)
        if out is not None and not self.dry_run:
            twist = Twist()
            twist.linear.x, twist.angular.z = float(out[0]), float(out[1])
            self.cmd_pub.publish(twist)

        state = t.snapshot()
        state.update(dry_run=self.dry_run, imu_ok=now - self.imu_t < self.imu_timeout,
                     yaw_deg=round(math.degrees(self.yaw), 1), depth_ok=now - t._depth_t < self.cfg.depth_timeout_s,
                     floor={'h': round(self.floor.h, 3), 'pitch': round(self.floor.pitch_deg, 1),
                            'ok': self.floor_ok},
                     cmd=None if out is None else [round(out[0], 3), round(out[1], 2)])
        if now - self._view_t >= VIEW_PERIOD_S:
            self._view_t = now
            view, camera_view = self._render(state), self._render_camera(state)
            with self._lock:
                self._view, self._camera_view = view, camera_view
        with self._lock:
            self._state = state

    def _end_straight(self):
        if self._straight_timer is not None:
            self.destroy_timer(self._straight_timer)
            self._straight_timer = None
        self.tick()

    def _finish_calibration(self, now):
        if len(self._calib_gyro) < 10:
            self.tracker.stop(now, 'calibration failed: no IMU data', error=True)
            return
        self.gyro_bias = float(np.mean(self._calib_gyro))
        self.yaw = 0.0
        self.floor_ref = self.floor
        self.get_logger().info(
            f'calibrated: gyro bias {math.degrees(self.gyro_bias):.2f} deg/s, camera '
            f'{self.floor.h:.3f} m above the ground, tilted {self.floor.pitch_deg:.1f} deg down')
        self.tracker.calibrated(now)

    def _handle(self, body, now):
        cmd = body.get('cmd')
        if cmd == 'start':
            if self.dry_run:
                return False, 'dry run: START is disabled'
            if now - self.cloud_t > self.cfg.depth_timeout_s:
                return False, f"no depth image on {self.get_parameter('depth_topic').value}"
            if now - self.imu_t > self.imu_timeout:
                return False, 'no IMU (/bno055/imu)'
            self._calib_gyro, self._calib_t0 = [], now
            self.floor_ref = self.floor
            return self.tracker.start(now)
        if cmd == 'stop':
            return self.tracker.stop(now, 'stopped by user')
        if cmd == 'drive':
            try:
                return self.tracker.drive(float(body.get('linear', 0.0)),
                                          float(body.get('angular', 0.0)), now)
            except (TypeError, ValueError) as exc:
                return False, f'bad drive command: {exc}'

        if cmd == 'straight':
            try:
                result = self.tracker.set_straight(float(body['ms']))
            except (KeyError, TypeError, ValueError) as exc:
                return False, f'bad straight command: {exc}'
            self._save_settings()
            self.get_logger().info(result[1])
            return result
        if cmd == 'range':
            try:
                result = self.tracker.set_range(float(body['min']), float(body['max']))
            except (KeyError, TypeError, ValueError) as exc:
                return False, f'bad range command: {exc}'
            if result[0]:
                self._save_settings()
                self.get_logger().info(result[1])
            return result
        if cmd == 'heights':
            try:
                result = self.tracker.set_heights(float(body['min']), float(body['max']))
            except (KeyError, TypeError, ValueError) as exc:
                return False, f'bad heights command: {exc}'
            if result[0]:
                self._save_settings()
                self.get_logger().info(result[1])
            return result
        if cmd == 'pwm':
            try:
                result = self.tracker.set_pwm(float(body['linear']), float(body['angular']))
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
                'straight_ms': self.cfg.straight_ms,
                'min_range': self.cfg.min_range, 'max_range': self.cfg.max_range}
        try:
            os.makedirs(os.path.dirname(self._settings_file) or '.', exist_ok=True)
            tmp = self._settings_file + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self._settings_file)
        except OSError as exc:
            self.get_logger().error(f'cannot save {self._settings_file}: {exc}')

    # ---------------- views for the web UI ----------------

    def _render(self, state):
        """Top-down, robot frame: 3 m ahead, 0.5 m behind, +-1.5 m. Robot drives up."""
        t, c, S = self.tracker, self.cfg, 100           # px per metre
        w, h = int(3.0 * S), int(3.5 * S)
        img = np.full((h, w, 3), 30, np.uint8)

        def to_px(x, y):
            return int(w / 2 - y * S), int(h - (x + 0.5) * S)

        def dots(xs, ys, colour, size=0):
            u = (w / 2 - np.asarray(ys) * S).astype(int)
            v = (h - (np.asarray(xs) + 0.5) * S).astype(int)
            ok = (u >= size) & (u < w - size) & (v >= size) & (v < h - size)
            for du in range(-size, size + 1):
                for dv in range(-size, size + 1):
                    img[v[ok] + dv, u[ok] + du] = colour

        m = t.cmap
        if m is not None:                                # clearance: brighter = more room
            v, u = np.mgrid[0:h, 0:w]
            room = m.clearance(h / S - 0.5 - (v + 0.5) / S, (w / 2 - (u + 0.5)) / S)
            shade = np.where(room < 0, 30, 25 + 60 * np.clip(room / max(c.road_width / 2, 1e-3), 0, 1))
            img[:] = shade.astype(np.uint8)[..., None]
        for x in (1, 2):                                 # 1 m marks
            cv2.line(img, to_px(x, 1.5), to_px(x, -1.5), (70, 70, 70), 1)
        if m is not None and m.n_objects:                # everything used, incl. memory: white
            ox, oy = np.nonzero(m.dist <= 0)
            dots(m.x_min + (ox + 0.5) * m.res, -m.y_half + (oy + 0.5) * m.res, (230, 230, 230), 1)
        if self._frame is not None:                      # this frame: red
            fpts, labels = self._frame
            q = fpts[labels == OBJECT]
            if len(q):
                dots(q @ self.floor.f, q @ self.floor.l, (60, 60, 240))
        if len(t.path) > 1:
            cv2.polylines(img, [np.array([to_px(a, b) for a, b in t.path])], False, (0, 220, 255), 2)
        if state['running']:
            cv2.line(img, to_px(0, 0), to_px(*t.target), (0, 140, 255), 1)
            cv2.circle(img, to_px(*t.target), 5, (0, 140, 255), -1)
        L, W = c.robot_length, c.robot_width
        corners = [(0, W / 2), (0, -W / 2), (-L, -W / 2), (-L, W / 2)]
        cv2.polylines(img, [np.array([to_px(a, b) for a, b in corners])], True, (60, 140, 255), 2)
        cv2.putText(img, state['state'], (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        return jpg.tobytes() if ok else None

    def _render_camera(self, state):
        """The depth image, colour = distance; things beside the road red, no depth black."""
        depth, K = self._depth, self._K
        if depth is not None and K is not None:
            c = self.cfg
            _, labels = label_image(depth, *K, self.floor, c.min_height, c.max_height, c.max_range,
                                    min_range=c.min_range)
            near = np.clip(depth / max(c.max_range, 0.1), 0, 1)
            img = cv2.applyColorMap((255 * (1 - near)).astype(np.uint8), cv2.COLORMAP_TURBO)
            img = (img * 0.55).astype(np.uint8)                 # dim, so the red stands out
            img[labels == OBJECT] = (40, 40, 255)
            img[labels == NONE] = 0
            if img.shape[1] < 424:
                img = cv2.resize(img, None, fx=424 / img.shape[1], fy=424 / img.shape[1],
                                 interpolation=cv2.INTER_NEAREST)
            text = (f"cam {state['floor']['h']:.2f} m, {state['floor']['pitch']:.0f} deg  "
                    f"band {100 * c.min_height:.0f}-{100 * c.max_height:.0f} cm")
            cv2.putText(img, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            return jpg.tobytes() if ok else None
        return None

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
<title>Race Terrain</title>
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
  .pad { display:grid; grid-template-columns:repeat(3, 72px); gap:6px; justify-content:center; }
  .pad button { height:52px; touch-action:none; user-select:none; }
  button.held { background:#1565c0; }
  input[type=number] { background:#222; color:var(--fg); border:1px solid #444;
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
      <span>road</span><span><span id="near" class="dot"></span><span id="path" class="num">-</span></span>
      <span>yaw</span><span id="yaw" class="num">-</span>
      <span>travel</span><span id="travel" class="num">-</span>
      <span>command</span><span id="cmd" class="num">-</span>
      <span>camera</span><span id="floor" class="num">-</span>
      <span>health</span><span><span id="imu" class="dot"></span>IMU
        <span id="depth" class="dot"></span>depth
        <span id="gnd" class="dot"></span>ground fit</span>
    </div>
    <div class="hint">Put the robot on the road between the two rows and press START; keep it
      still ~1 s while the gyro calibrates. It stops by itself when the rows end. Space = STOP.</div>
  </section>
  <section>
    <h2>Teleop</h2>
    <div class="pad">
      <span></span><button data-drive="1,0">W &uarr;</button><span></span>
      <button data-drive="0,1">A &larr;</button>
      <button data-drive="-1,0">S &darr;</button>
      <button data-drive="0,-1">D &rarr;</button>
    </div>
    <div class="hint">Hold a button or W/A/S/D to drive, at the linear / angular PWM below.
      Only while the run is stopped. Let go and the robot stops.</div>
  </section>
  <section>
    <h2>Straight at START</h2>
    <div class="numrow"><span>ms</span>
      <button class="step" data-for="strn" data-d="-500">&minus;500</button><button class="step" data-for="strn" data-d="-100">&minus;100</button>
      <input id="strn" type="number" inputmode="numeric" min="0" max="60000" step="100" value="0">
      <button class="step" data-for="strn" data-d="100">+100</button><button class="step" data-for="strn" data-d="500">+500</button></div>
    <div class="row">
      <button id="strset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="strcur" class="num">-</span></span>
    </div>
    <div class="hint">After START (and the 1 s calibration) the robot first drives straight
      ahead for this many milliseconds at the linear PWM, then follows the centre line.
      0 = follow the centre line at once. Kept for next time.</div>
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
    <div class="hint">Nothing changes until you press Set (an edited value is orange until
      then). Applies at once, also while running, and is kept for next time.
      Linear: forward PWM. Angular: the fastest steering turn.</div>
  </section>
  <section>
    <h2>Height band</h2>
    <div class="numrow"><span>bottom</span>
      <button class="step" data-for="hlon" data-d="-5">&minus;5</button><button class="step" data-for="hlon" data-d="-1">&minus;1</button>
      <input id="hlon" type="number" inputmode="numeric" min="0" max="200" step="1" value="5">
      <button class="step" data-for="hlon" data-d="1">+1</button><button class="step" data-for="hlon" data-d="5">+5</button></div>
    <div class="numrow"><span>top</span>
      <button class="step" data-for="hhin" data-d="-5">&minus;5</button><button class="step" data-for="hhin" data-d="-1">&minus;1</button>
      <input id="hhin" type="number" inputmode="numeric" min="1" max="200" step="1" value="60">
      <button class="step" data-for="hhin" data-d="1">+1</button><button class="step" data-for="hhin" data-d="5">+5</button></div>
    <div class="row">
      <button id="hset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="hcur" class="num">-</span></span>
    </div>
    <div class="hint">Centimetres above the ground. Only points in this band count as the
      things beside the road (red). Raise the bottom if bumps and stones on the road turn
      red; keep it below the height of the things beside the road.</div>
  </section>
  <section>
    <h2>Valid distance</h2>
    <div class="numrow"><span>near</span>
      <button class="step" data-for="rlon" data-d="-10">&minus;10</button><button class="step" data-for="rlon" data-d="-5">&minus;5</button>
      <input id="rlon" type="number" inputmode="numeric" min="0" max="999" step="5" value="0">
      <button class="step" data-for="rlon" data-d="5">+5</button><button class="step" data-for="rlon" data-d="10">+10</button></div>
    <div class="numrow"><span>far</span>
      <button class="step" data-for="rhin" data-d="-10">&minus;10</button><button class="step" data-for="rhin" data-d="-5">&minus;5</button>
      <input id="rhin" type="number" inputmode="numeric" min="10" max="1000" step="5" value="250">
      <button class="step" data-for="rhin" data-d="5">+5</button><button class="step" data-for="rhin" data-d="10">+10</button></div>
    <div class="row">
      <button id="rset">Set</button>
      <span class="hint" style="margin:0">applied: <span id="rcur" class="num">-</span></span>
    </div>
    <div class="hint">Centimetres ahead of the camera. Only points between near and far are
      used: lower far to ignore things behind the course (people, the tent), raise near to
      ignore the robot's own parts or noise right in front. Applies at once, also while
      running, and is kept for next time.</div>
  </section>
  <section>
    <h2>Camera</h2>
    <img id="camview" alt="depth camera view">
    <div class="hint">The depth image: colour = distance (red-orange near, blue far), bright
      red = things beside the road (in the height band), black = no depth.</div>
  </section>
  <section>
    <h2>Top-down</h2>
    <img id="view" alt="top-down view">
    <div class="hint">Robot (orange) driving up, lines every 1 m. Red = objects in this frame,
      white = all objects used (incl. the last ~2 s), brighter ground = more room, yellow =
      the centre line it follows, orange dot = where it steers.</div>
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
    $('near').className = 'dot' + (s.objects_near ? ' ok' : '');
    $('path').textContent = (s.objects_near ? 'rows in view' : 'no rows nearby')
      + ', centre line ' + s.path_len + ' m ahead';
    $('yaw').textContent = s.yaw_deg + ' deg';
    $('travel').textContent = s.travel + ' m';
    $('cmd').textContent = s.cmd ? 'v ' + s.cmd[0] + ', w ' + s.cmd[1] : '-';
    $('floor').textContent = s.floor.h + ' m high, ' + s.floor.pitch + ' deg down';
    $('imu').className = 'dot' + (s.imu_ok ? ' ok' : '');
    $('depth').className = 'dot' + (s.depth_ok ? ' ok' : '');
    $('gnd').className = 'dot' + (s.floor.ok ? ' ok' : '');
    $('start').disabled = s.running || s.dry_run;
    $('pwmcur').textContent = 'linear ' + s.linear_pwm.toFixed(0) + ', angular ' + s.angular_pwm.toFixed(0);
    $('linn').max = s.max_linear_pwm;
    $('angn').max = s.max_angular_pwm;
    const lo = Math.round(s.min_height * 100), hi = Math.round(s.max_height * 100);
    $('hcur').textContent = lo + ' to ' + hi + ' cm';
    applied.linn = Math.round(s.linear_pwm); applied.angn = Math.round(s.angular_pwm);
    applied.hlon = lo; applied.hhin = hi;
    applied.strn = Math.round(s.straight_ms);
    const rlo = Math.round(s.min_range * 100), rhi = Math.round(s.max_range * 100);
    applied.rlon = rlo; applied.rhin = rhi;
    $('rcur').textContent = rlo + ' to ' + rhi + ' cm';
    $('strcur').textContent = Math.round(s.straight_ms) + ' ms';
    Object.keys(applied).forEach((id) => {
      if (!loaded[id]) { loaded[id] = true; $(id).value = applied[id]; }
      markPending(id);
    });
  } catch (e) { $('state').textContent = 'no connection'; }
  $('view').src = '/view.jpg?' + Date.now();
  $('camview').src = '/camera.jpg?' + Date.now();
}
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
const SET_BUTTON = {linn: 'pwmset', angn: 'pwmset', hlon: 'hset', hhin: 'hset',
                    strn: 'strset', rlon: 'rset', rhin: 'rset'};
Object.keys(SET_BUTTON).forEach((id) => {
  $(id).oninput = () => markPending(id);
  $(id).onkeydown = (e) => { if (e.key === 'Enter') $(SET_BUTTON[id]).click(); };
});
$('rset').onclick = async () => {
  const lo = val('rlon'), hi = val('rhin');
  document.activeElement.blur();
  if (hi <= lo) { $('msg').textContent = 'far must be above near'; $('msg').className = 'err'; return; }
  await cmd({cmd: 'range', min: lo / 100, max: hi / 100});
  loaded.rlon = loaded.rhin = false;
};
$('strset').onclick = async () => {
  const ms = val('strn');
  document.activeElement.blur();
  await cmd({cmd: 'straight', ms: ms});
  loaded.strn = false;
};
$('hset').onclick = async () => {
  const lo = val('hlon'), hi = val('hhin');
  document.activeElement.blur();
  if (hi <= lo) { $('msg').textContent = 'top must be above bottom'; $('msg').className = 'err'; return; }
  await cmd({cmd: 'heights', min: lo / 100, max: hi / 100});
  loaded.hlon = loaded.hhin = false;
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
// ---- teleop: resend the held direction every 100 ms (server deadman 0.3 s) ----
let drive = null, driveTimer = null;
function sendDrive() {
  if (drive) fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: drive[0], angular: drive[1]})}).catch(() => {});
}
function startDrive(v) {
  drive = v; sendDrive();
  if (!driveTimer) driveTimer = setInterval(sendDrive, 100);
}
function stopDrive() {
  if (!drive) return;
  drive = null; clearInterval(driveTimer); driveTimer = null;
  document.querySelectorAll('.held').forEach((b) => b.classList.remove('held'));
  fetch('/api/cmd', {method: 'POST', body: JSON.stringify(
    {cmd: 'drive', linear: 0, angular: 0})}).catch(() => {});
}
document.querySelectorAll('[data-drive]').forEach((b) => {
  const v = b.dataset.drive.split(',').map(Number);
  b.addEventListener('pointerdown', (e) => {
    b.setPointerCapture(e.pointerId); b.classList.add('held'); startDrive(v);
  });
  ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(
    (ev) => b.addEventListener(ev, stopDrive));
});
window.addEventListener('blur', stopDrive);
document.addEventListener('visibilitychange', stopDrive);
const DRIVE_KEYS = {w: [1, 0], s: [-1, 0], a: [0, 1], d: [0, -1]};
const held = new Set();
function keyDrive() {
  let lin = 0, ang = 0;
  held.forEach((k) => { lin += DRIVE_KEYS[k][0]; ang += DRIVE_KEYS[k][1]; });
  if (lin || ang) startDrive([lin, ang]); else stopDrive();
}
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (k === ' ') { e.preventDefault(); stopDrive(); $('stop').click(); return; }
  if (e.repeat) return;
  if (k in DRIVE_KEYS) { held.add(k); keyDrive(); }
});
document.addEventListener('keyup', (e) => {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (held.delete(k)) keyDrive();
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
                try:
                    self.end_headers()
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass                     # the browser went away mid-reply

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
    node = RaceTerainNode()
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
