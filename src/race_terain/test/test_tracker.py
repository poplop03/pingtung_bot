"""Closed-loop check of the tracker on a made-up course of posts: straight, hairpin U-turn, straight.

The camera is idealised: every post surface point within 65 deg / 2.5 m is
seen. Run:  cd src/race_terain && python3 -m pytest test
"""

import math

import numpy as np
import pytest

from race_terain.tracker import CALIBRATE, DONE, RUN, STRAIGHT, Config, Tracker

HFOV = math.radians(65)


def course(road=0.8, straight=2.5, spacing=0.25, post_r=0.04, turn_left=True):
    """Post centres. Robot starts at (0, 0) heading +x, U-turn at x = straight."""
    s = 1.0 if turn_left else -1.0
    r_mid = road / 2 + 0.15                 # centre-line radius of the U-turn
    cy = s * r_mid                          # U-turn centre
    posts = []
    for x in np.arange(-0.5, straight, spacing):
        posts += [(x, road / 2), (x, -road / 2)]                     # first straight
        posts += [(x, cy + s * (r_mid + road / 2)),                  # second straight
                  (x, cy + s * (r_mid - road / 2))]
    for a in np.arange(-math.pi / 2, math.pi / 2 + 1e-6, spacing / (r_mid + road / 2)):
        R = r_mid + road / 2
        posts.append((straight + R * math.cos(a), cy + s * R * math.sin(a)))
    for a in np.arange(-math.pi / 2, math.pi / 2 + 1e-6, spacing / max(r_mid - road / 2, 0.05)):
        R = r_mid - road / 2
        posts.append((straight + R * math.cos(a), cy + s * R * math.sin(a)))
    posts = np.array(posts)
    # surface points of each post
    ang = np.linspace(0, 2 * math.pi, 12, endpoint=False)
    pts = (posts[:, None, :] + post_r * np.stack([np.cos(ang), np.sin(ang)], -1)[None]).reshape(-1, 2)
    return posts, pts


def visible(pts, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    rx, ry = c * dx + s * dy, -s * dx + c * dy
    keep = (rx > 0.18) & (rx < 2.5) & (np.abs(np.arctan2(ry, rx)) < HFOV / 2)
    return rx[keep], ry[keep]


@pytest.mark.parametrize('turn_left', [True, False])
@pytest.mark.parametrize('speed, drift, road', [
    (1.0, 0.0, 0.8),
    (0.6, 0.03, 0.6),       # robot slower than k_lin says, gyro drifting 1.7 deg/s, narrow road
    (1.5, 0.03, 1.0),       # faster than it thinks
])
def test_drives_straight_hairpin_straight(turn_left, speed, drift, road):
    """speed: real / commanded speed. drift: gyro error, rad/s."""
    cfg = Config(road_width=road)
    posts, pts = course(road=road, turn_left=turn_left)
    t = Tracker(cfg)
    pose = [0.0, 0.0, 0.0]
    dt, now, imu_yaw = 0.05, 0.0, 0.0
    t.on_depth(*visible(pts, pose), HFOV, now)
    assert t.start(now)[0]
    t.tick(now, 0.0, dt)
    assert t.state == CALIBRATE
    t.calibrated(now)
    min_clear = math.inf
    for _ in range(int(60 / dt)):
        now += dt
        t.on_depth(*visible(pts, pose), HFOV, now)
        out = t.tick(now, imu_yaw, dt)
        if t.state != RUN:
            break
        v, w = out
        v *= speed
        pose[2] += w * dt
        imu_yaw += (w + drift) * dt
        pose[0] += v * dt * math.cos(pose[2])
        pose[1] += v * dt * math.sin(pose[2])
        min_clear = min(min_clear, float(np.min(np.hypot(posts[:, 0] - pose[0],
                                                          posts[:, 1] - pose[1]))))
    assert t.state == DONE, t.message
    # robot half width 0.15 + post radius 0.04
    assert min_clear > 0.19, f'came within {min_clear:.2f} m of a post centre'
    # it came back along the second straight, past the start line, on the right side
    assert pose[0] < 0.0 and (pose[1] > 0.6 if turn_left else pose[1] < -0.6), pose


def test_straight_phase_then_controller():
    """After START + calibration: (linear_pwm / k_lin, 0) for exactly straight_ms, then the controller."""
    cfg = Config(straight_ms=1500.0, linear_pwm=60.0, k_lin=150.0)
    posts, pts = course()
    t = Tracker(cfg)
    pose = [0.0, 0.0, 0.0]
    t.on_depth(*visible(pts, pose), HFOV, 0.0)
    assert t.start(0.0)[0]
    t.calibrated(1.0)
    assert t.state == STRAIGHT
    assert t.straight_end == pytest.approx(2.5)
    for now in np.arange(1.0, 2.5 - 1e-9, 0.01):
        t.on_depth(*visible(pts, pose), HFOV, now)
        assert t.tick(now, 0.0, 0.01) == (pytest.approx(0.4), 0.0)
        assert t.state == STRAIGHT
    t.tick(2.5, 0.0, 0.01)
    assert t.state == RUN and t.straight_done_ms == pytest.approx(1500.0)


def test_straight_zero_skips_to_controller():
    t = Tracker(Config(straight_ms=0.0))
    posts, pts = course()
    t.on_depth(*visible(pts, [0.0, 0.0, 0.0]), HFOV, 0.0)
    t.start(0.0)
    t.calibrated(1.0)
    assert t.state == RUN
