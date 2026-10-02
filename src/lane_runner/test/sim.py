"""Tiny ray-cast simulator of the U-shaped course for the tests: depth images from a D415.

World frame: x along lane 1 (start -> end), y left, z up, floor at z = 0.
Lane 1 is y in [-lane/2, lane/2]; lane 2 is to its left, past the divider.
"""

import math

import numpy as np

W, H = 212, 120                  # D415 depth at 424x240, decimated by 2
FX = (W / 2) / math.tan(math.radians(65 / 2))
FY = (H / 2) / math.tan(math.radians(40 / 2))
CX, CY = W / 2, H / 2
HFOV = 2 * math.atan(W / 2 / FX)


class Course:
    def __init__(self, lane=0.75, divider=0.03, length=4.0, turn_space=0.9,
                 wall_h=0.25, pigs=()):
        """pigs: (x, lane_index 1|2, side 'left'|'right') placed against that wall."""
        t, e = divider, 0.02
        y2 = lane + t                    # lane 2 is lane 1 shifted by this
        self.lane, self.y2, self.length = lane, y2, length
        box = []
        box.append(((-0.32, -lane / 2 - e, 0), (length, -lane / 2, wall_h)))            # right outer
        box.append(((-0.32, lane / 2, 0), (length - turn_space, lane / 2 + t, wall_h)))  # divider
        box.append(((-0.32, y2 + lane / 2, 0), (length, y2 + lane / 2 + e, wall_h)))    # left outer
        box.append(((length, -lane / 2 - e, 0), (length + e, y2 + lane / 2 + e, wall_h)))  # end
        box.append(((-0.34, -lane / 2 - e, 0), (-0.32, y2 + lane / 2 + e, wall_h)))     # start
        self.walls = list(box)
        self.pigs = []
        for x, lane_i, side in pigs:
            yc = 0.0 if lane_i == 1 else y2
            # lane 2 is driven the other way: its "left" is world -y
            left = (side == 'left') == (lane_i == 1)
            yw = yc + (lane / 2 - 0.01 - 0.05 if left else -lane / 2 + 0.01 + 0.05)
            self.pigs.append(((x - 0.10, yw - 0.05, 0), (x + 0.10, yw + 0.05, 0.09)))
        self.boxes = np.array(self.walls + self.pigs, dtype=float)   # (n, 2, 3)

    def depth(self, x, y, yaw, cam_fwd=0.15, cam_h=0.22, pitch_deg=12.0, noise=0.003,
              rng=None):
        u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
        do = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], -1)
        f, l, up = do[..., 2], -do[..., 0], -do[..., 1]           # optical -> body
        p = math.radians(pitch_deg)
        f, up = f * math.cos(p) + up * math.sin(p), -f * math.sin(p) + up * math.cos(p)
        c, s = math.cos(yaw), math.sin(yaw)
        d = np.stack([c * f - s * l, s * f + c * l, up], -1).reshape(-1, 3)
        o = np.array([x + c * cam_fwd, y + s * cam_fwd, cam_h])
        t = np.full(len(d), np.inf)
        down = d[:, 2] < -1e-9
        t[down] = -o[2] / d[down, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            inv = 1.0 / d
            for lo, hi in self.boxes:
                t1, t2 = (lo - o) * inv, (hi - o) * inv
                tmin = np.nanmax(np.minimum(t1, t2), axis=1)
                tmax = np.nanmin(np.maximum(t1, t2), axis=1)
                hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
                t = np.where(hit & (tmin < t), tmin, t)
        depth = t.reshape(H, W)                  # t is already optical z (do has z = 1)
        if noise:
            rng = rng or np.random.default_rng(1)
            depth = depth * (1 + noise * rng.standard_normal(depth.shape))
        depth[(depth < 0.18) | (depth > 4.0) | ~np.isfinite(depth)] = 0.0
        return depth

    def collides(self, x, y, yaw, length=0.35, width=0.30, which='all'):
        """Does the robot's footprint (centered on x, y) touch any box?"""
        c, s = math.cos(yaw), math.sin(yaw)
        a, b = np.meshgrid(np.linspace(-length / 2, length / 2, 15),
                           np.linspace(-width / 2, width / 2, 11))
        px, py = x + c * a - s * b, y + s * a + c * b
        boxes = self.pigs if which == 'pigs' else self.walls + self.pigs
        for lo, hi in boxes:
            if np.any((px > lo[0]) & (px < hi[0]) & (py > lo[1]) & (py < hi[1])):
                return True
        return False
