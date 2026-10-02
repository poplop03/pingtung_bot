import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from sim import CX, CY, FX, FY, HFOV, Course  # noqa: E402

from lane_runner.mission import DONE, IDLE, LANE1, Config, Mission, wrap  # noqa: E402
from lane_runner.perception import (  # noqa: E402
    FLOOR, NONE, OBSTACLE, Floor, LaneGrid, depth_to_points, fit_floor, label_image,
    obstacles, wall_ahead)

PIGS = [(1.5, 1, 'left'), (2.8, 1, 'right'), (2.0, 2, 'left')]


def run(course, speed_real=1.0, gyro_drift=0.0, start=(0.0, 0.0, 0.0), cam=None,
        t_max=120.0, dt=0.05, **cfg):
    """Drive the whole course. Returns (mission, trail, collision)."""
    cfg = Config(**cfg)
    cam = cam or {}
    m = Mission(cfg)
    x, y, yaw = start
    m.start(0.0)
    floor = fit_floor(depth_to_points(course.depth(x, y, yaw, **cam), FX, FY, CX, CY,
                                      row_from=0.5))
    assert floor is not None
    m.calibrated(0.0)
    t, trail = 0.0, []
    yaw_imu = 0.0
    while t < t_max:
        t += dt
        pts = depth_to_points(course.depth(x, y, yaw, **cam), FX, FY, CX, CY)
        m.on_depth(*obstacles(pts, floor, cfg.min_height, cfg.max_height, cfg.max_range), HFOV, t)
        out = m.tick(t, wrap(yaw_imu), dt)
        if out is None:
            break
        v, w = out
        x += v * speed_real * math.cos(yaw) * dt
        y += v * speed_real * math.sin(yaw) * dt
        yaw += w * dt
        yaw_imu += (w + gyro_drift) * dt          # the IMU drifts, the robot does not
        trail.append((x, y, yaw))
        if course.collides(x, y, yaw):
            return m, trail, True
    return m, trail, False


def assert_finished(course, m, trail, collided):
    assert not collided, f'collision at {trail[-1]} in {m.state}'
    assert m.state == DONE, f'{m.state}: {m.message}'
    x, y, yaw = trail[-1]
    assert x < 0.6 and abs(y - course.y2) < 0.15, f'ended at {trail[-1]}'   # goal end of lane 2


# ------------------------------------------------------------ perception

def test_floor_fit_recovers_camera_height_and_tilt():
    c = Course()
    for h, pitch in ((0.22, 12.0), (0.30, 25.0), (0.15, 5.0)):
        pts = depth_to_points(c.depth(0, 0, 0, cam_h=h, pitch_deg=pitch), FX, FY, CX, CY,
                              row_from=0.5)
        floor = fit_floor(pts)
        assert floor.h == pytest.approx(h, abs=0.01)
        assert floor.pitch_deg == pytest.approx(pitch, abs=1.0)


def test_floor_from_params_matches_fit():
    c = Course()
    fitted = fit_floor(depth_to_points(c.depth(0, 0, 0), FX, FY, CX, CY, row_from=0.5))
    given = Floor.from_params(0.22, 12.0)
    assert np.allclose(fitted.n, given.n, atol=0.02)


def test_obstacles_are_walls_and_pig_not_floor():
    c = Course(pigs=[(0.8, 1, 'left')])
    floor = Floor.from_params(0.22, 12.0)
    x, y = obstacles(depth_to_points(c.depth(0, 0, 0), FX, FY, CX, CY), floor, 0.04, 0.5, 2.0)
    assert len(x) > 100
    pig = (np.abs(x + 0.15 - 0.8) < 0.12) & (y > 0.2) & (y < 0.37)    # camera is 0.15 ahead
    assert pig.sum() > 10
    walls = (np.abs(y) > 0.36)
    assert walls.sum() > 10
    assert (np.abs(y[~pig & ~walls]) < 0.3).sum() < 0.02 * len(x)    # nothing in the free lane


def test_label_image_matches_obstacles():
    c = Course(pigs=[(0.8, 1, 'left')])
    floor = Floor.from_params(0.22, 12.0)
    depth = c.depth(0, 0, 0)
    pts, labels = label_image(depth, FX, FY, CX, CY, floor, 0.04, 0.5, 2.0, stride=2)
    x, _ = obstacles(depth_to_points(depth, FX, FY, CX, CY, stride=2), floor, 0.04, 0.5, 2.0)
    assert (labels == OBSTACLE).sum() == len(x)
    assert (labels == NONE).sum() == (depth[::2, ::2] == 0).sum()
    assert (labels[-10:] == FLOOR).mean() > 0.9          # bottom rows: the lane floor


def test_wall_ahead_sees_end_wall_but_not_a_pig():
    c = Course(pigs=[(1.0, 1, 'left')])
    floor = Floor.from_params(0.22, 12.0)

    def seen(x_robot):
        pts = depth_to_points(c.depth(x_robot, 0, 0), FX, FY, CX, CY)
        ox, oy = obstacles(pts, floor, 0.04, 0.5, 2.0)
        return wall_ahead(ox, oy, 0.45, HFOV, 0.375)

    assert seen(c.length - 0.15 - 0.35)          # camera 0.35 m from the end wall
    assert not seen(c.length - 0.15 - 0.9)       # still far
    assert not seen(1.0 - 0.10 - 0.15 - 0.25)    # pig 0.25 m ahead, on the left


def test_grid_gap_avoids_pig_and_ignores_space_behind_walls():
    g = LaneGrid(-1, 3, 0.75)
    # walls at +-0.375, a pig against the left wall at x 1.0..1.2
    wx = np.linspace(0.3, 2.0, 60)
    xs = np.concatenate([wx, wx, np.full(10, 1.1)])
    ys = np.concatenate([np.full(60, 0.38), np.full(60, -0.38), np.linspace(0.26, 0.36, 10)])
    for _ in range(3):
        g.update((0, 0, 0), xs, ys, HFOV, 0.18, 2.0)
    gap = g.pick(g.gaps(0.8, 1.4, 0.21), 0.0)
    assert gap[0] == pytest.approx(-0.16, abs=0.03) and gap[1] == pytest.approx(0.04, abs=0.03)
    # a robot position outside the lane on the right must not pick the strip behind the wall
    assert g.pick(g.gaps(0.8, 1.4, 0.21), -0.6) is None or \
        g.pick(g.gaps(0.8, 1.4, 0.21), -0.6)[1] < -0.45


def test_grid_forgets_what_moved_away():
    g = LaneGrid(-1, 3, 0.75)
    g.update((0, 0, 0), np.array([1.0] * 5), np.array([0.0] * 5), HFOV, 0.18, 2.0)
    assert (g.hits >= g.occupied).any()
    for _ in range(8):
        g.update((0, 0, 0), np.zeros(0), np.zeros(0), HFOV, 0.18, 2.0)
    assert not (g.hits >= g.occupied).any()


# ------------------------------------------------------------ whole course

def test_full_course_with_pigs():
    c = Course(pigs=PIGS)
    m, trail, collided = run(c)
    assert_finished(c, m, trail, collided)


@pytest.mark.parametrize('speed_real', [0.6, 1.5])
def test_full_course_when_k_lin_is_wrong(speed_real):
    c = Course(pigs=PIGS)
    m, trail, collided = run(c, speed_real=speed_real)
    assert_finished(c, m, trail, collided)


def test_full_course_off_centre_start_drifting_gyro_other_camera_mount():
    c = Course(pigs=[(1.5, 1, 'left'), (2.2, 1, 'right'), (1.5, 2, 'right')])
    m, trail, collided = run(c, start=(0.0, -0.15, math.radians(8)),
                             gyro_drift=math.radians(0.3), cam={'cam_h': 0.30, 'pitch_deg': 22})
    assert_finished(c, m, trail, collided)


def test_stop_and_restart():
    m = Mission(Config())
    assert m.tick(0.0, 0.0, 0.05) is None
    m.start(0.0)
    assert m.tick(0.1, 0.0, 0.05) == (0.0, 0.0)          # calibrating: hold still
    m.calibrated(0.2)
    assert m.state == LANE1
    m.stop(0.3, 'user')
    assert m.state == IDLE and m.tick(0.4, 0.0, 0.05) is None
    assert m.start(0.5)[0]


def test_pauses_without_depth():
    m = Mission(Config())
    m.start(0.0)
    m.calibrated(0.0)
    m.on_depth(np.zeros(0), np.zeros(0), HFOV, 0.0)
    assert m.tick(0.1, 0.0, 0.05)[0] > 0
    assert m.tick(2.0, 0.0, 0.05) == (0.0, 0.0) and 'depth' in m.message


def test_pwm_changes_apply_mid_run():
    m = Mission(Config(k_lin=100.0, k_ff=40.0, linear_pwm=40.0, angular_pwm=20.0))
    m.start(0.0)
    m.calibrated(0.0)
    m.on_depth(np.zeros(0), np.zeros(0), HFOV, 0.0)
    v, _ = m.tick(0.05, 0.0, 0.05)
    assert v == pytest.approx(0.4)                       # 40 / k_lin 100
    _, w = m.tick(0.10, math.radians(60), 0.05)          # far off heading: turn at the limit
    assert abs(w) == pytest.approx(0.5)                  # 20 / k_ff 40
    assert m.set_pwm(60, 36)[0]
    v, w = m.tick(0.15, math.radians(60), 0.05)
    assert abs(w) == pytest.approx(0.9)
    assert m.set_pwm(999, -5)[0] and (m.cfg.linear_pwm, m.cfg.angular_pwm) == (255.0, 0.0)


def test_height_band_is_validated_and_changes_what_counts():
    m = Mission(Config())
    assert m.set_heights(0.02, 0.30)[0]
    assert (m.cfg.min_height, m.cfg.max_height) == (0.02, 0.30)
    assert not m.set_heights(0.20, 0.10)[0]                  # top below bottom: refused
    assert not m.set_heights(float('nan'), 0.3)[0]
    assert (m.cfg.min_height, m.cfg.max_height) == (0.02, 0.30)
    # a band above the pigs (9 cm) sees the walls (25 cm) but not the pig
    c = Course(pigs=[(0.8, 1, 'left')])
    floor = Floor.from_params(0.22, 12.0)
    pts = depth_to_points(c.depth(0, 0, 0), FX, FY, CX, CY)
    x, y = obstacles(pts, floor, 0.12, 0.5, 2.0)
    assert not ((np.abs(x + 0.15 - 0.8) < 0.08) & (y > 0.2) & (y < 0.33)).any()
    assert (np.abs(y) > 0.36).sum() > 10


def test_angular_pwm_is_capped_by_default():
    m = Mission(Config())
    m.set_pwm(40, 223)                   # the value a bumped slider once sent mid-run
    assert m.cfg.angular_pwm == 100.0
    assert m.max_rate == pytest.approx(100.0 / 33.0)
