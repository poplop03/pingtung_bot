"""Drive the centre line between the two rows of things beside the road. No ROS in here.

    IDLE ─START─► CALIBRATE ─► STRAIGHT ─► RUN ──nothing beside the road for lost_hold_s──► DONE
      ▲           (still: gyro   straight_ms    │
      │            bias)         (0 = skip)     │
      └──────── STOP / no IMU / no depth ┘

The course has straights AND a hairpin U-turn, so the centre is not fitted as
a straight line per side: in the middle of a hairpin the outer row stands
right across the camera's view and the inner row is beside or behind the
robot. Instead, every tick:

  1. Object points (the posts, barriers, reflectors) of the latest depth frame
     plus remembered ones go into a top-down ClearanceMap around the robot:
     the distance from every spot to the nearest object. Remembered points
     are kept in an odometry frame, so the inner posts of the U-turn are
     still there long after they left the camera's view. The odometry is
     predicted from the IMU yaw and the commanded speed, then corrected by
     matching each new frame's points to the remembered ones (a 2D rigid
     fit: position and the gyro's drift). Posts make good landmarks; along
     a featureless barrier the match cannot tell how far along it the robot
     is, and the prediction stands. Remembered points
     inside the camera's current view are not used: it sees them anew.
  2. trace_center() walks forward from the robot, always towards the most
     clearance (up to half a road width): that is the centre line between the
     two sides, and it bends with the road.
  3. Pure pursuit: aim at the point pursuit_dist along that line,
     angular.z = k_yaw * angle to it, slower while turning hard. If the line
     is blocked right away, turn in place towards the side with more room.

Speeds are PWM, like everywhere on this robot (no wheel encoders):
    linear.x  = linear_pwm / k_lin,  |angular.z| <= angular_pwm / k_ff
wheel_control closes the yaw-rate loop on the IMU.

Time is any monotonic clock in seconds, passed in by the caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from scipy.spatial import cKDTree

from race_terain.perception import ClearanceMap, trace_center

IDLE = 'idle'
CALIBRATE = 'calibrate'
STRAIGHT = 'straight'
RUN = 'run'
DONE = 'done'
RUNNING = (CALIBRATE, STRAIGHT, RUN)


@dataclass
class Config:
    k_lin: float = 150.0            # wheel_control's PWM per unit linear.x - must match
    k_ff: float = 33.0              # wheel_control's PWM per rad/s - must match
    linear_pwm: float = 50.0        # forward PWM
    angular_pwm: float = 35.0       # turn PWM limit
    max_linear_pwm: float = 255.0   # UI limits
    max_angular_pwm: float = 100.0
    speed_scale: float = 1.0        # real speed / (pwm / k_lin), for the point memory only

    road_width: float = 0.8         # m, between the inner faces of the two rows: MEASURE
    robot_width: float = 0.30       # m
    robot_length: float = 0.35      # m, camera to the robot's rear (drawing only)
    margin: float = 0.05            # m of air kept each side of the robot

    trace_length: float = 1.5       # m of centre line traced ahead
    trace_step: float = 0.1         # m
    trace_max_turn_deg: float = 25.0    # per step; 25 deg / 0.1 m = ~0.23 m tightest radius
    memory_s: float = 8.0           # remember objects this long (for the U-turn's inner posts) ...
    memory_range: float = 3.0       # ... and only within this many metres of the robot
    voxel: float = 0.03             # m, memory resolution
    match_max_m: float = 0.25       # largest per-frame correction the scan match may make
    match_max_deg: float = 3.0

    pursuit_dist: float = 0.6       # m along the centre line; shorter = tighter bends
    k_yaw: float = 2.5              # rad/s per rad of angle to the aim point
    slow_angle_deg: float = 35.0    # at this angle to the aim point, drive at min_speed_frac
    min_speed_frac: float = 0.5
    blocked_turn_frac: float = 0.7  # turn-in-place rate when blocked, fraction of the max

    finish_radius: float = 1.2      # m, nothing beside the road this close ...
    lost_hold_s: float = 1.5        # ... for this long = finished
    min_run_s: float = 3.0          # never "finished" sooner than this after START

    min_height: float = 0.06        # m above the ground to count as a thing beside the road
    max_height: float = 0.60
    min_range: float = 0.0          # m, valid distance of the point cloud: nearer is ignored ...
    max_range: float = 2.5          # m, ... and so is farther
    depth_timeout_s: float = 0.5    # no depth for this long while running -> pause
    calib_s: float = 1.0            # still time at START for the gyro bias
    straight_ms: float = 0.0        # after START: drive straight this long first (0 = skip)
    max_straight_ms: float = 60000.0
    teleop_timeout_s: float = 0.3   # teleop deadman: the page must resend a held key within this


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Tracker:

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.state = IDLE
        self.message = 'put the robot on the road and press START'
        self.error = False
        self.pose = (0.0, 0.0, 0.0)        # odometry frame: x, y, yaw
        self._mem = np.zeros((0, 2))       # remembered object points, odometry frame
        self._mem_t = np.zeros(0)          # when each was last seen
        self.match_ok = False              # the last frame was matched to the memory
        self.yaw_corr = 0.0                # odometry yaw = IMU yaw + this (gyro drift, from matching)
        self._frame = (np.zeros(0), np.zeros(0))
        self._hfov = math.radians(65)
        self._depth_t = -math.inf
        self._seen_t = -math.inf           # last time something was within finish_radius
        self._start_t = 0.0
        self._cmd = (0.0, 0.0)
        self.path: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.target = (0.0, 0.0)
        self.cmap: Optional[ClearanceMap] = None
        self.travel = 0.0
        self._travel_xy = (0.0, 0.0)
        self._teleop = (0.0, 0.0)
        self._teleop_t = -math.inf
        self._zero_until = -math.inf
        self._straight_t = 0.0
        self.straight_done_ms: Optional[float] = None   # measured length of the last straight phase

    # ------------------------------------------------------------ user / node

    def start(self, now: float) -> Tuple[bool, str]:
        if self.state in RUNNING:
            return False, f'already running ({self.state})'
        if now - self._depth_t > self.cfg.depth_timeout_s:
            return False, 'no depth image'
        self.state, self._start_t, self.error = CALIBRATE, now, False
        self._teleop_t = -math.inf
        return self._say('calibrating - keep the robot still')

    def calibrated(self, now: float) -> Tuple[bool, str]:
        if self.state != CALIBRATE:
            return False, 'not calibrating'
        self.travel = 0.0
        self.yaw_corr = self.pose[2]       # the node restarts the IMU yaw at 0: keep the memory aligned
        if self.cfg.straight_ms > 0:
            self.state, self._straight_t = STRAIGHT, now
            return self._say(f'straight for {self.cfg.straight_ms:.0f} ms')
        return self._run(now)

    def _run(self, now: float, message: str = 'running') -> Tuple[bool, str]:
        self.state, self._start_t, self._seen_t = RUN, now, now
        return self._say(message)

    @property
    def straight_end(self) -> float:
        """When the straight phase ends (only meaningful in STRAIGHT)."""
        return self._straight_t + self.cfg.straight_ms / 1000.0

    def set_straight(self, ms: float) -> Tuple[bool, str]:
        """Straight-ahead time after START, before the centre-line following."""
        self.cfg.straight_ms = _clamp(float(ms), 0.0, self.cfg.max_straight_ms)
        return True, f'straight {self.cfg.straight_ms:.0f} ms after START'

    def stop(self, now: float, reason: str = 'stopped', error: bool = False) -> Tuple[bool, str]:
        self.state = IDLE
        self._cmd = (0.0, 0.0)
        return self._say(reason, error)

    def drive(self, linear: float, angular: float, now: float) -> Tuple[bool, str]:
        """Teleop while stopped. linear/angular are -1..1, scaled by linear_pwm / angular_pwm."""
        if self.state in RUNNING:
            return False, 'stop the run first'
        c = self.cfg
        lin = _clamp(float(linear), -1.0, 1.0) * c.linear_pwm / c.k_lin
        ang = _clamp(float(angular), -1.0, 1.0) * c.angular_pwm / c.k_ff
        if lin == 0.0 and ang == 0.0:
            if self._teleop_t != -math.inf:
                self._zero_until = now + 0.3
            self._teleop_t = -math.inf
        else:
            self._teleop, self._teleop_t = (lin, ang), now
        return True, ''

    def set_pwm(self, linear: float, angular: float) -> Tuple[bool, str]:
        c = self.cfg
        c.linear_pwm = _clamp(float(linear), 0.0, c.max_linear_pwm)
        c.angular_pwm = _clamp(float(angular), 0.0, c.max_angular_pwm)
        return True, f'PWM linear {c.linear_pwm:.0f}, angular {c.angular_pwm:.0f}'

    def set_range(self, near: float, far: float) -> Tuple[bool, str]:
        """Valid distance of the point cloud, metres ahead of the camera."""
        if not 0.0 <= near < far <= 10.0:
            return False, 'far must be above near (and at most 10 m)'
        self.cfg.min_range, self.cfg.max_range = float(near), float(far)
        return True, f'valid distance {100 * near:.0f} to {100 * far:.0f} cm'

    def set_heights(self, lo: float, hi: float) -> Tuple[bool, str]:
        if not 0.0 <= lo < hi:
            return False, 'top must be above bottom'
        self.cfg.min_height, self.cfg.max_height = float(lo), float(hi)
        return True, f'height band {100 * lo:.0f} to {100 * hi:.0f} cm'

    # ------------------------------------------------------------ inputs

    def on_depth(self, x: np.ndarray, y: np.ndarray, hfov: float, now: float) -> None:
        """Object points of one frame, robot frame (x forward, y left). Runs also when idle."""
        cfg = self.cfg
        self._depth_t, self._hfov = now, hfov
        self._frame = (x, y)
        self.match_ok = self._match(x, y)
        px, py, th = self.pose
        c, s = math.cos(th), math.sin(th)
        pts = np.r_[self._mem, np.stack([px + c * x - s * y, py + s * x + c * y], axis=1)]
        ts = np.r_[self._mem_t, np.full(len(x), now)]
        # one point per voxel, the latest; forget old and far ones
        order = np.argsort(-ts, kind='stable')
        cell = np.floor(pts[order] / cfg.voxel).astype(np.int64)
        _, first = np.unique(cell[:, 0] * 1_000_003 + cell[:, 1], return_index=True)
        keep = order[first]
        pts, ts = pts[keep], ts[keep]
        keep = ((now - ts <= cfg.memory_s)
                & (np.hypot(pts[:, 0] - px, pts[:, 1] - py) <= cfg.memory_range))
        self._mem, self._mem_t = pts[keep], ts[keep]

    def _match(self, x: np.ndarray, y: np.ndarray) -> bool:
        """Correct the pose so this frame's points land on the remembered ones (2D ICP)."""
        if len(x) < 20 or len(self._mem) < 20:
            return False
        px, py, th = self.pose
        step = max(1, len(x) // 400)
        local = np.stack([x[::step], y[::step]], axis=1)
        tree = cKDTree(self._mem)
        cx, cy, ct = px, py, th
        for radius in (0.2, 0.12, 0.08):
            c, s = math.cos(ct), math.sin(ct)
            a = local @ np.array([[c, s], [-s, c]]) + (cx, cy)
            d, i = tree.query(a, distance_upper_bound=radius)
            ok = np.isfinite(d)
            if ok.sum() < 15:
                return False
            a, b = a[ok], self._mem[i[ok]]
            # rigid fit b = R a + t (Kabsch), rotation about the robot
            ac, bc = a - (cx, cy), b - (cx, cy)
            ma, mb = ac.mean(axis=0), bc.mean(axis=0)
            h = (ac - ma).T @ (bc - mb)
            dth = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
            c, s = math.cos(dth), math.sin(dth)
            t = mb - np.array([[c, -s], [s, c]]) @ ma
            cx, cy, ct = cx + t[0], cy + t[1], ct + dth
        if (math.hypot(cx - px, cy - py) > self.cfg.match_max_m
                or abs(wrap(ct - th)) > math.radians(self.cfg.match_max_deg)):
            return False
        self.pose = (cx, cy, ct)
        self.yaw_corr = wrap(self.yaw_corr + ct - th)
        return True

    def _objects(self) -> Tuple[np.ndarray, np.ndarray]:
        """This frame's points plus remembered ones the camera cannot see now, robot frame."""
        fx, fy = self._frame
        pts = self._mem[self._mem_t < self._depth_t]
        if not len(pts):
            return fx, fy
        px, py, th = self.pose
        c, s = math.cos(th), math.sin(th)
        dx, dy = pts[:, 0] - px, pts[:, 1] - py
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        in_view = (rx > 0.2) & (np.abs(ry) < rx * math.tan(self._hfov / 2) * 0.9)
        return np.r_[fx, rx[~in_view]], np.r_[fy, ry[~in_view]]

    # ------------------------------------------------------------ control

    def tick(self, now: float, yaw: float, dt: float) -> Optional[Tuple[float, float]]:
        """(linear.x, angular.z) while running, else None. yaw: IMU, rad."""
        c = self.cfg
        # predict the odometry with what was commanded last tick (the scan match corrects it)
        v = self._cmd[0] * c.speed_scale
        px, py, _ = self.pose
        if self.state == RUN:
            self.travel += math.hypot(px - self._travel_xy[0], py - self._travel_xy[1])
        self._travel_xy = (px, py)
        yaw = wrap(yaw + self.yaw_corr)
        self.pose = (px + v * dt * math.cos(yaw), py + v * dt * math.sin(yaw), yaw)

        ox, oy = self._objects()
        self.cmap = ClearanceMap(ox, oy, x_max=max(c.max_range, c.trace_length) + 0.1)
        self.path = trace_center(self.cmap, c.road_width / 2, c.robot_width / 2 + c.margin,
                                 c.trace_length, c.trace_step, c.trace_max_turn_deg)
        if self.cmap.objects_within(c.finish_radius):
            self._seen_t = now

        if self.state == CALIBRATE:
            return self._send((0.0, 0.0))
        if self.state == STRAIGHT:
            left = self.straight_end - now
            if left > 0:
                # angular 0 while moving: wheel_control locks the heading it had when the
                # motion started and holds it on the IMU, so this is a straight line
                self._say(f'straight, {1000 * left:.0f} ms left')
                return self._send((c.linear_pwm / c.k_lin, 0.0))
            self.straight_done_ms = 1000 * (now - self._straight_t)
            self._run(now)
        if self.state != RUN:
            # stopped: teleop, with a deadman and a short burst of zeros after it
            if now - self._teleop_t <= c.teleop_timeout_s:
                return self._send(self._teleop)
            if self._teleop_t != -math.inf:
                self._teleop_t, self._zero_until = -math.inf, now + 0.3
            if now < self._zero_until:
                return self._send((0.0, 0.0))
            self._cmd = (0.0, 0.0)
            return None
        if now - self._depth_t > c.depth_timeout_s:
            self._say('paused: no depth image')
            return self._send((0.0, 0.0))
        if now - self._seen_t > c.lost_hold_s and now - self._start_t >= c.min_run_s:
            self.state = DONE
            self._say(f'finished: nothing beside the road any more ({self.travel:.1f} m)')
            return self._send((0.0, 0.0))

        w_max = c.angular_pwm / c.k_ff
        v_full = c.linear_pwm / c.k_lin
        if len(self.path) < 3:
            # blocked right ahead: turn in place towards the side with more room
            room = self.cmap.clearance(np.array([0.3, 0.3]), np.array([0.3, -0.3]))
            w = w_max * c.blocked_turn_frac * (1.0 if room[0] >= room[1] else -1.0)
            self.target = (0.0, 0.0)
            self._say('blocked ahead - turning ' + ('left' if w > 0 else 'right'))
            return self._send((0.0, w))

        tx, ty = self.target = self._aim_point(c.robot_width / 2 + c.margin)
        alpha = math.atan2(ty, tx)
        w = _clamp(c.k_yaw * alpha, -w_max, w_max)
        frac = 1.0 - (1.0 - c.min_speed_frac) * min(1.0, abs(alpha) / math.radians(c.slow_angle_deg))
        self._say(f'following the centre, {math.degrees(alpha):+.0f} deg, {self.travel:.1f} m')
        return self._send((v_full * frac, w))

    def snapshot(self) -> dict:
        c = self.cfg
        return {
            'state': self.state, 'running': self.state in RUNNING,
            'straight_ms': c.straight_ms,
            'message': self.message, 'error': self.error,
            'objects_near': self.cmap is not None and self.cmap.objects_within(c.finish_radius),
            'path_len': round((len(self.path) - 1) * c.trace_step, 1),
            'travel': round(self.travel, 1),
            'match_ok': self.match_ok,
            'linear_pwm': c.linear_pwm, 'angular_pwm': c.angular_pwm,
            'max_linear_pwm': c.max_linear_pwm, 'max_angular_pwm': c.max_angular_pwm,
            'min_height': c.min_height, 'max_height': c.max_height,
            'min_range': c.min_range, 'max_range': c.max_range,
        }

    def _aim_point(self, safe: float) -> Tuple[float, float]:
        """Point pursuit_dist along the centre line, or nearer if the straight
        line to it would cut the inside of a bend (less than `safe` clearance)."""
        c = self.cfg
        k = min(len(self.path) - 1, max(1, int(round(c.pursuit_dist / c.trace_step))))
        for j in range(k, 1, -1):
            tx, ty = self.path[j]
            s = np.linspace(0.0, 1.0, max(2, int(math.hypot(tx, ty) / 0.03)))
            if self.cmap.clearance(s * tx, s * ty).min() >= safe:
                return tx, ty
        return self.path[1]

    def _send(self, cmd):
        self._cmd = cmd
        return cmd

    def _say(self, message: str, error: bool = False) -> Tuple[bool, str]:
        self.message, self.error = message, error
        return not error, message
