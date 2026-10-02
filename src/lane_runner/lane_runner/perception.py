"""Depth image -> obstacles on the floor. No ROS in here, so it can be unit tested.

Frames
    optical  the camera's: x right, y down, z forward (what the depth image gives)
    robot    x forward, y left, z up, origin on the floor under the camera
    leg      x along the current lane, y left, fixed while one leg of the
             course is driven; the robot's pose in it is dead-reckoned

The floor is a plane n·p = h in the optical frame (n = unit normal pointing
DOWN to the floor, h = camera height). It is fitted from the depth image at
START (RANSAC), so the camera's height and tilt never have to be measured.
Anything between min_height and max_height above that plane is an obstacle:
the lane walls and the pigs. The slatted floor's gaps read as below the floor
and are ignored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Floor:
    n: np.ndarray        # unit normal in the optical frame, pointing down (to the floor)
    h: float             # camera height above the floor, m

    def __post_init__(self) -> None:
        self.n = np.asarray(self.n, dtype=float)
        self.n = self.n / np.linalg.norm(self.n)
        f = np.array([0.0, 0.0, 1.0]) - self.n[2] * self.n       # optical z, levelled
        self.f = f / np.linalg.norm(f)                            # robot forward
        self.l = np.cross(self.f, self.n)                         # robot left

    @property
    def pitch_deg(self) -> float:
        """Camera tilt, + = looking down."""
        return math.degrees(math.atan2(self.n[2], self.n[1]))

    @classmethod
    def from_params(cls, height: float, pitch_deg: float) -> 'Floor':
        p = math.radians(pitch_deg)
        return cls(np.array([0.0, math.cos(p), math.sin(p)]), height)


def depth_to_points(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                    stride: int = 1, row_from: float = 0.0) -> np.ndarray:
    """Valid pixels as (N, 3) optical-frame points. row_from: skip the top fraction."""
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    keep = d > 0
    if row_from > 0:
        keep &= v >= row_from * depth_m.shape[0]
    z = d[keep]
    return np.stack([(u[keep] - cx) * z / fx, (v[keep] - cy) * z / fy, z], axis=1)


def fit_floor(points: np.ndarray, iters: int = 200, thresh: float = 0.012,
              max_tilt_deg: float = 45.0, min_inliers: int = 200,
              rng: Optional[np.random.Generator] = None) -> Optional[Floor]:
    """RANSAC plane whose normal is within max_tilt_deg of 'down' (optical +y)."""
    if len(points) < min_inliers:
        return None
    rng = rng or np.random.default_rng(0)
    best, best_n = None, 0
    cos_max = math.cos(math.radians(max_tilt_deg))
    for _ in range(iters):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[1] < 0:
            n = -n
        if n[1] < cos_max:
            continue
        h = float(n @ a)
        if h <= 0.02:
            continue
        inliers = np.abs(points @ n - h) < thresh
        count = int(inliers.sum())
        if count > best_n:
            best, best_n = inliers, count
    if best is None or best_n < min_inliers:
        return None
    # least-squares refine on the inliers
    p = points[best]
    centroid = p.mean(axis=0)
    # normal = smallest eigenvector of the 3x3 scatter (a full SVD of p builds an
    # N x N matrix: 250 ms for 4000 points on the Jetson)
    q = p - centroid
    n = np.linalg.eigh(q.T @ q)[1][:, 0]
    if n[1] < 0:
        n = -n
    return Floor(n, float(n @ centroid))


def obstacles(points: np.ndarray, floor: Floor, min_height: float, max_height: float,
              max_range: float) -> Tuple[np.ndarray, np.ndarray]:
    """Robot-frame (x forward, y left) of the points that stick up from the floor."""
    height = floor.h - points @ floor.n
    x = points @ floor.f
    keep = (height > min_height) & (height < max_height) & (x > 0) & (x < max_range)
    return x[keep], (points @ floor.l)[keep]


NONE, FLOOR, OBSTACLE, OTHER = 0, 1, 2, 3     # pixel labels


def label_image(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                floor: Floor, min_height: float, max_height: float, max_range: float,
                stride: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """Every (strided) pixel as a point and a label, for viewing and the obstacle list.

    Returns (points (h, w, 3) optical frame, labels (h, w) uint8):
    NONE no depth, FLOOR at or below min_height, OBSTACLE what obstacles() keeps,
    OTHER too high or too far.
    """
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    pts = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], axis=-1)
    height = floor.h - pts @ floor.n
    x = pts @ floor.f
    valid = d > 0
    labels = np.full(d.shape, OTHER, np.uint8)
    labels[~valid] = NONE
    labels[valid & (height <= min_height)] = FLOOR
    labels[valid & (height > min_height) & (height < max_height) & (x > 0) & (x < max_range)] = OBSTACLE
    return pts, labels


def front_distance(x: np.ndarray, y: np.ndarray, half_width: float) -> float:
    """Distance to what is straight ahead in a corridor of the robot's width (inf = nothing)."""
    ahead = x[np.abs(y) < half_width]
    return float(np.percentile(ahead, 5)) if len(ahead) >= 5 else math.inf


def wall_ahead(x: np.ndarray, y: np.ndarray, within: float, hfov: float,
               lane_half: float, res: float = 0.02, fraction: float = 0.75) -> bool:
    """True when something closer than `within` spans (almost) the whole visible lane.

    That is the end wall. A pig never does it: pigs sit on one side of the lane.
    Only the width the camera can actually see at that distance is judged.
    """
    half = min(lane_half, within * math.tan(hfov / 2) * 0.9)
    if half < 2 * res:
        return False
    band = (x < within) & (np.abs(y) < half)
    bins = np.arange(-half, half + res, res)
    counts, _ = np.histogram(y[band], bins=bins)
    return (counts >= 2).mean() >= fraction


class LaneGrid:
    """Top-down occupancy of one leg, in the leg frame, forgetting what turns out free."""

    def __init__(self, x_min: float, x_max: float, y_half: float, res: float = 0.02,
                 hit_max: int = 6, occupied: int = 2) -> None:
        self.x_min, self.y_half, self.res = x_min, y_half, res
        self.nx = int(math.ceil((x_max - x_min) / res))
        self.ny = int(math.ceil(2 * y_half / res))
        self.hits = np.zeros((self.nx, self.ny), dtype=np.int8)
        self.hit_max, self.occupied = hit_max, occupied
        self.ys = -y_half + (np.arange(self.ny) + 0.5) * res     # cell centers

    def _ix(self, x):
        return np.floor((np.asarray(x) - self.x_min) / self.res).astype(int)

    def _iy(self, y):
        return np.floor((np.asarray(y) + self.y_half) / self.res).astype(int)

    def update(self, pose: Tuple[float, float, float], x: np.ndarray, y: np.ndarray,
               hfov: float, near: float, far: float) -> None:
        """Add a frame's obstacle points (robot frame); let unseen-but-visible cells decay."""
        px, py, th = pose
        c, s = math.cos(th), math.sin(th)
        seen = np.zeros_like(self.hits, dtype=bool)
        if len(x):
            ix, iy = self._ix(px + c * x - s * y), self._iy(py + s * x + c * y)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            seen[ix[ok], iy[ok]] = True
        # cells the camera saw this frame (inside its wedge) but found nothing in
        i0, i1 = max(self._ix(px - far), 0), min(self._ix(px + far) + 1, self.nx)
        if i1 > i0:
            gx = self.x_min + (np.arange(i0, i1) + 0.5) * self.res
            dx, dy = gx[:, None] - px, self.ys[None, :] - py
            rx, ry = c * dx + s * dy, -s * dx + c * dy
            visible = (rx > near) & (rx < far) & (np.abs(ry) < rx * math.tan(hfov / 2) * 0.9)
            block = self.hits[i0:i1]
            block[visible & ~seen[i0:i1]] -= 1
        np.clip(self.hits, 0, self.hit_max, out=self.hits)
        self.hits[seen] = np.minimum(self.hits[seen] + 2, self.hit_max)

    def gaps(self, x_from: float, x_to: float, clearance: float) -> List[Tuple[float, float]]:
        """Lateral ranges where the robot's CENTER fits between x_from and x_to."""
        i0, i1 = max(self._ix(x_from), 0), min(self._ix(x_to) + 1, self.nx)
        if i1 <= i0:
            self.raw = np.zeros(self.ny, dtype=bool)
            return [(-self.y_half + clearance, self.y_half - clearance)]
        blocked = (self.hits[i0:i1] >= self.occupied).any(axis=0)
        self.raw = blocked.copy()                    # real obstacles, before widening
        k = max(1, int(math.ceil(clearance / self.res)))
        blocked = np.convolve(blocked.astype(int), np.ones(2 * k + 1, dtype=int), 'same') > 0
        blocked[:k] = blocked[-k:] = True            # the grid's edges count as walls
        out, start = [], None
        for j, b in enumerate(blocked):
            if not b and start is None:
                start = j
            elif b and start is not None:
                out.append((self.ys[start], self.ys[j - 1]))
                start = None
        if start is not None:
            out.append((self.ys[start], self.ys[-1]))
        return out


    def pick(self, gaps: List[Tuple[float, float]], y_now: float) -> Optional[Tuple[float, float]]:
        """The gap the robot is in, else the nearest one it can reach.

        Reachable = no real obstacle (a wall, a pig) between the robot and the
        gap. That rules out the never-seen strip behind a wall, which would
        otherwise look like free space.
        """
        def between_blocked(g):
            lo, hi = sorted((y_now, g[0] if y_now < g[0] else g[1]))
            j0, j1 = self._iy(lo), self._iy(hi)
            return self.raw[max(j0, 0):max(min(j1, self.ny), 0)].any()

        def dist(g):
            return 0.0 if g[0] <= y_now <= g[1] else min(abs(y_now - g[0]), abs(y_now - g[1]))

        ok = [g for g in gaps if dist(g) == 0.0 or not between_blocked(g)]
        return min(ok, key=dist) if ok else None
