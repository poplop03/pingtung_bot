"""Course state machine: lane 1 -> U-turn -> lane 2 -> goal. No ROS in here.

    IDLE ─start─► CALIBRATE ─► LANE1 ─end wall─► PIVOT1 ─► CROSS ─► PIVOT2 ─► LANE2 ─end wall─► DONE
                  (still: gyro     ▲ follow the free gap: centre of the lane, or past a pig
                   bias, floor)    └ STOP / fault from any state ─► IDLE

Headings are absolute yaw from the IMU, 0 = the heading at START:
lane 1 = 0, cross = ±90°, lane 2 = ±180° (+ = turn_left).

Speed is set in PWM, like everywhere else on this robot (no wheel encoders),
and may be changed while running (set_pwm):
    linear_pwm   forward, lanes and cross-over   linear.x  = pwm / k_lin
    angular_pwm  the fastest turn, steering and pivots   angular.z <= pwm / k_ff

Steering in a lane: the depth obstacles go into a top-down LaneGrid in the
leg frame (x along the lane). The robot's pose in it is dead-reckoned from
the IMU yaw and the speed wheel_control is asked for (linear_pwm / k_lin,
times speed_scale) - rough, but only needed for the ~1 m a pig spends beside
or behind the camera. Each tick takes the free gap the robot fits through from
one robot length behind the camera to look_ahead in front, and aims at its
centre (pure pursuit on the IMU heading). With no pig that is the middle of
the lane; with a pig it is the free side.

The end of a lane is something that spans the whole visible lane width
closer than end_dist (wall_ahead). A pig never does: it sits on one side.

Time is any monotonic clock in seconds, passed in by the caller.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from lane_runner.perception import LaneGrid, front_distance, wall_ahead

IDLE = 'idle'
CALIBRATE = 'calibrate'
LANE1 = 'lane1'
PIVOT1 = 'pivot1'
CROSS = 'cross'
PIVOT2 = 'pivot2'
LANE2 = 'lane2'
DONE = 'done'
RUNNING = (CALIBRATE, LANE1, PIVOT1, CROSS, PIVOT2, LANE2)


@dataclass
class Config:
    k_lin: float = 150.0            # wheel_control's PWM per unit linear.x - must match
    k_ff: float = 33.0              # wheel_control's PWM per rad/s (turn feedforward) - must match
    linear_pwm: float = 40.0        # base PWM forward (lanes and cross-over)
    angular_pwm: float = 30.0       # turn PWM limit (steering and pivots)
    max_linear_pwm: float = 255.0   # limits for the UI
    max_angular_pwm: float = 100.0  # ~3 rad/s with k_ff 33; steering never needs more
    pivot_min_pwm: float = 12.0     # turn PWM a pivot never drops below, to beat track friction
    speed_scale: float = 1.0        # real speed / (pwm / k_lin); >1 = robot faster than k_lin says
    turn_left: bool = True          # U-turn direction: lane 2 is on the robot's left

    robot_width: float = 0.30       # m
    robot_length: float = 0.35      # m, camera to the rear: how long a passed pig is remembered
    margin: float = 0.06            # m of air kept each side of the robot
    lane_width: float = 0.75        # m, inside, wall to wall
    divider: float = 0.03           # m, thickness of the wall between the lanes

    look_ahead: float = 1.2         # m of lane considered for the gap
    pursuit_dist: float = 0.6       # m, aim point distance; shorter = sharper swerve
    max_steer_deg: float = 35.0     # max heading off the lane direction
    k_yaw: float = 2.5              # rad/s of turn per rad of heading error
    pivot_tol_deg: float = 4.0
    pivot_settle_s: float = 0.3
    pivot_timeout_s: float = 10.0

    end_dist: float = 0.45          # m, a wall across the lane this close = end of lane
    end_frames: int = 3             # depth frames in a row that must agree
    end_min_travel: float = 0.5     # m, ignore "end" right after a leg starts
    emergency_dist: float = 0.22    # m, obstacle in the robot's path this close: turn in place only
    cross_front_stop: float = 0.25  # m, stop the cross-over early if a wall is this close
    cross_timeout_s: float = 10.0

    min_height: float = 0.04        # m above the floor to count as an obstacle
    max_height: float = 0.50        # m, ignore the tent, people behind the walls...
    max_range: float = 2.0          # m
    near_range: float = 0.18        # m, the D415 sees nothing closer
    depth_timeout_s: float = 0.5    # no depth for this long -> pause
    calib_s: float = 1.5            # still time at START for gyro bias + floor fit


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Mission:

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.state = IDLE
        self.message = 'press START with the robot at the start of lane 1'
        self.error = False
        self._t_state = 0.0
        self._depth_t = -math.inf
        self._x = self._y = np.zeros(0)
        self._hfov = math.radians(65)
        self.front = math.inf
        self.end_count = 0
        self._reset_leg(0.0)

    # ------------------------------------------------------------ user / node

    def start(self, now: float) -> Tuple[bool, str]:
        if self.state in RUNNING:
            return False, f'already running ({self.state})'
        self._enter(CALIBRATE, now)
        return self._say('calibrating - keep the robot still')

    def calibrated(self, now: float) -> Tuple[bool, str]:
        """The node has the gyro bias and the floor; yaw is 0 now."""
        if self.state != CALIBRATE:
            return False, 'not calibrating'
        self._enter(LANE1, now)
        return self._say('lane 1')

    def stop(self, now: float, reason: str = 'stopped', error: bool = False) -> Tuple[bool, str]:
        self._enter(IDLE, now)
        return self._say(reason, error)

    def set_pwm(self, linear: float, angular: float) -> Tuple[bool, str]:
        """Change the speeds; applies at once, also while running."""
        c = self.cfg
        c.linear_pwm = _clamp(float(linear), 0.0, c.max_linear_pwm)
        c.angular_pwm = _clamp(float(angular), 0.0, c.max_angular_pwm)
        return True, f'PWM linear {c.linear_pwm:.0f}, angular {c.angular_pwm:.0f}'

    def set_heights(self, low: float, high: float) -> Tuple[bool, str]:
        """The band above the floor that counts as an obstacle, in metres; applies at once."""
        low, high = float(low), float(high)
        if not (math.isfinite(low) and math.isfinite(high)):
            return False, 'heights must be numbers'
        low, high = _clamp(low, 0.0, 2.0), _clamp(high, 0.0, 2.0)
        if high - low < 0.01:
            return False, 'the top must be at least 1 cm above the bottom'
        self.cfg.min_height, self.cfg.max_height = low, high
        return True, f'obstacles: {100 * low:.0f} to {100 * high:.0f} cm above the floor'

    @property
    def max_rate(self) -> float:
        """rad/s that angular_pwm allows."""
        return self.cfg.angular_pwm / self.cfg.k_ff

    @property
    def min_rate(self) -> float:
        return min(self.cfg.pivot_min_pwm, self.cfg.angular_pwm) / self.cfg.k_ff

    def on_depth(self, x: np.ndarray, y: np.ndarray, hfov: float, now: float) -> None:
        """Obstacle points of one depth frame, robot frame (x forward, y left)."""
        c = self.cfg
        self._x, self._y, self._hfov, self._depth_t = x, y, hfov, now
        self.front = front_distance(x, y, c.robot_width / 2 + c.margin / 2)
        end = wall_ahead(x, y, c.end_dist, hfov, c.lane_width / 2)
        self.end_count = self.end_count + 1 if end else 0
        if self.state in (LANE1, LANE2):
            self.grid.update(self.pose, x, y, hfov, c.near_range, c.max_range)

    # ------------------------------------------------------------ periodic

    def tick(self, now: float, yaw: float, dt: float) -> Optional[Tuple[float, float]]:
        """(linear.x, angular.z) to send, or None when idle."""
        if self.state not in RUNNING:
            return None
        if self.state == CALIBRATE:
            return 0.0, 0.0
        c = self.cfg
        if now - self._depth_t > c.depth_timeout_s and self.state != PIVOT1 and self.state != PIVOT2:
            self._say('paused: no depth image', True)
            return 0.0, 0.0
        s = 1.0 if c.turn_left else -1.0
        if self.state in (LANE1, LANE2):
            leg = 0.0 if self.state == LANE1 else s * math.pi
            return self._lane(now, wrap(yaw - leg), dt)
        if self.state == PIVOT1:
            return self._pivot(now, yaw, s * math.pi / 2, CROSS)
        if self.state == PIVOT2:
            return self._pivot(now, yaw, wrap(s * math.pi), LANE2)
        return self._cross(now, wrap(yaw - s * math.pi / 2), dt)

    # ------------------------------------------------------------ states

    def _lane(self, now: float, th: float, dt: float) -> Tuple[float, float]:
        c = self.cfg
        v = c.linear_pwm / c.k_lin
        self._advance(v, th, dt)
        px, py, _ = self.pose

        if self.travel >= c.end_min_travel and self.end_count >= c.end_frames:
            nxt = PIVOT1 if self.state == LANE1 else DONE
            self._enter(nxt, now)
            self._say('end of lane 1 - U-turn' if nxt == PIVOT1 else 'goal reached')
            return 0.0, 0.0

        clearance = c.robot_width / 2 + c.margin
        gap = None
        for ahead in (c.look_ahead, c.look_ahead / 2, c.end_dist):   # the end wall closes far gaps
            gap = self.grid.pick(self.grid.gaps(px - c.robot_length, px + ahead, clearance), py)
            if gap:
                break
        if gap:
            self.gap = gap
            self.target_y = 0.5 * (gap[0] + gap[1])
        psi = math.atan2(self.target_y - py, c.pursuit_dist)
        psi = _clamp(psi, -math.radians(c.max_steer_deg), math.radians(c.max_steer_deg))
        err = wrap(psi - th)
        w = _clamp(c.k_yaw * err, -self.max_rate, self.max_rate)

        blocked = self.front < c.emergency_dist and self.end_count == 0
        self._v = 0.0 if blocked else v
        if blocked:
            self._say(f'{self.state}: obstacle {self.front:.2f} m ahead - turning in place')
            turn = w if abs(w) > 1e-3 else (self.target_y - py) or 1.0
            return 0.0, math.copysign(max(abs(w), self.min_rate), turn)
        side = 'centre' if abs(self.target_y) < 0.05 else ('left' if self.target_y > 0 else 'right')
        self._say(f'{self.state}: {self.travel:.1f} m, keeping {side} ({self.target_y:+.2f} m)')
        return v * max(0.3, math.cos(err)), w

    def _pivot(self, now: float, yaw: float, target: float, nxt: str) -> Tuple[float, float]:
        c = self.cfg
        err = wrap(target - yaw)
        if now - self._t_state > c.pivot_timeout_s:
            self.stop(now, f'{self.state}: turn not finished in {c.pivot_timeout_s:.0f} s', True)
            return 0.0, 0.0
        if abs(err) < math.radians(c.pivot_tol_deg):
            if self._settle_t is None:
                self._settle_t = now
            if now - self._settle_t >= c.pivot_settle_s:
                self._enter(nxt, now)
                self._say('crossing to lane 2' if nxt == CROSS else 'lane 2')
            return 0.0, 0.0
        self._settle_t = None
        rate = _clamp(c.k_yaw * abs(err), self.min_rate, self.max_rate)
        self._say(f'{self.state}: {math.degrees(err):+.0f} deg to go')
        return 0.0, math.copysign(rate, err)

    def _cross(self, now: float, th: float, dt: float) -> Tuple[float, float]:
        c = self.cfg
        v = c.linear_pwm / c.k_lin
        self._advance(v, th, dt)
        distance = c.lane_width + c.divider
        if self.travel >= distance or self.front < c.cross_front_stop:
            self._enter(PIVOT2, now)
            return 0.0, 0.0
        if now - self._t_state > c.cross_timeout_s:
            self.stop(now, 'cross-over took too long', True)
            return 0.0, 0.0
        self._say(f'crossing: {self.travel:.2f} / {distance:.2f} m')
        return v, _clamp(-c.k_yaw * th, -self.max_rate, self.max_rate)

    # ------------------------------------------------------------ internals

    def _advance(self, v: float, th: float, dt: float) -> None:
        d = self._v * self.cfg.speed_scale * dt       # speed commanded last tick
        px, py, _ = self.pose
        self.pose = (px + d * math.cos(th), py + d * math.sin(th), th)
        self.travel += abs(d)
        self._v = v

    def _reset_leg(self, now: float) -> None:
        c = self.cfg
        self.grid = LaneGrid(-1.0, 8.0, c.lane_width)
        self.pose = (0.0, 0.0, 0.0)
        self.travel = 0.0
        self.target_y = 0.0
        self.gap: Optional[Tuple[float, float]] = None
        self._v = 0.0
        self._settle_t: Optional[float] = None

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self._t_state = now
        self.end_count = 0
        self._reset_leg(now)

    def _say(self, message: str, error: bool = False) -> Tuple[bool, str]:
        self.message, self.error = message, error
        return not error, message

    def snapshot(self) -> dict:
        return {
            'state': self.state,
            'message': self.message,
            'error': self.error,
            'running': self.state in RUNNING,
            'pose': [round(v, 3) for v in self.pose],
            'travel': round(self.travel, 2),
            'target_y': round(self.target_y, 3),
            'gap': [round(g, 3) for g in self.gap] if self.gap else None,
            'front': None if math.isinf(self.front) else round(self.front, 2),
            'end_count': self.end_count,
            'linear_pwm': self.cfg.linear_pwm,
            'angular_pwm': self.cfg.angular_pwm,
            'max_linear_pwm': self.cfg.max_linear_pwm,
            'max_angular_pwm': self.cfg.max_angular_pwm,
            'min_height': self.cfg.min_height,
            'max_height': self.cfg.max_height,
        }
