"""Depth image -> floor, things beside the road, and the two road edges. No ROS in here.

Frames
    optical  the camera's: x right, y down, z forward (what the depth image gives)
    robot    x forward, y left, z up, origin on the ground under the camera

The ground is a plane n·p = h in the optical frame (n = unit normal pointing
DOWN, h = camera height), fitted by RANSAC from the lower half of the depth
image. On rough terrain the robot pitches and rolls, so the node refits it on
every frame. Anything between min_height and max_height above it counts as an
object: the things lining both sides of the road.

The centre of the road is traced on a top-down clearance map (distance from
every spot to the nearest object): see clearance_map() and trace_center().
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Floor:
    n: np.ndarray        # unit normal in the optical frame, pointing down (to the ground)
    h: float             # camera height above the ground, m

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

    def close_to(self, other: 'Floor', max_deg: float, max_dh: float) -> bool:
        angle = math.degrees(math.acos(_clamp(float(self.n @ other.n), -1.0, 1.0)))
        return angle <= max_deg and abs(self.h - other.h) <= max_dh


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


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


def fit_floor(points: np.ndarray, iters: int = 100, thresh: float = 0.02,
              max_tilt_deg: float = 50.0, min_inliers: int = 200,
              rng: Optional[np.random.Generator] = None) -> Optional[Floor]:
    """RANSAC plane whose normal is within max_tilt_deg of 'down' (optical +y)."""
    if len(points) < min_inliers:
        return None
    rng = rng or np.random.default_rng(0)
    cos_max = math.cos(math.radians(max_tilt_deg))
    # all candidate planes at once (a repeated index gives a degenerate plane, dropped)
    a, b, c = points[rng.integers(0, len(points), size=(iters, 3))].transpose(1, 0, 2)
    n = np.cross(b - a, c - a)
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-9
    n, a = n[ok] / norm[ok, None], a[ok]
    n[n[:, 1] < 0] *= -1
    h = np.einsum('ij,ij->i', n, a)
    ok = (n[:, 1] >= cos_max) & (h > 0.02)
    if not ok.any():
        return None
    n, h = n[ok], h[ok]
    counts = (np.abs(points @ n.T - h) < thresh).sum(axis=0)
    k = int(np.argmax(counts))
    if counts[k] < min_inliers:
        return None
    p = points[np.abs(points @ n[k] - h[k]) < thresh]     # least-squares refine on the inliers
    centroid = p.mean(axis=0)
    # normal = smallest eigenvector of the 3x3 scatter (a full SVD of p builds an
    # N x N matrix: 250 ms for 4000 points on the Jetson)
    q = p - centroid
    n = np.linalg.eigh(q.T @ q)[1][:, 0]
    if n[1] < 0:
        n = -n
    return Floor(n, float(n @ centroid))


NONE, FLOOR, OBJECT, OTHER = 0, 1, 2, 3     # pixel labels


def label_image(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                floor: Floor, min_height: float, max_height: float, max_range: float,
                stride: int = 1, min_range: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """Every (strided) pixel as a point and a label.

    Returns (points (h, w, 3) optical frame, labels (h, w) uint8):
    NONE no depth, FLOOR at or below min_height, OBJECT in the height band and
    range, OTHER too high or too far.
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
    labels[valid & (height > min_height) & (height < max_height)
           & (x > min_range) & (x < max_range)] = OBJECT
    return pts, labels


def label_points(pts: np.ndarray, floor: Floor, min_height: float, max_height: float,
                 max_range: float, min_range: float = 0.0) -> np.ndarray:
    """Labels of (N, 3) optical-frame points: FLOOR, OBJECT or OTHER (see label_image)."""
    height = floor.h - pts @ floor.n
    x = pts @ floor.f
    labels = np.full(len(pts), OTHER, np.uint8)
    labels[height <= min_height] = FLOOR
    labels[(height > min_height) & (height < max_height)
           & (x > min_range) & (x < max_range)] = OBJECT
    return labels


def cloud_to_xyz(data: bytes, point_step: int, offsets: Tuple[int, int, int]) -> np.ndarray:
    """PointCloud2 payload (float32 x, y, z at the given byte offsets) -> finite (N, 3) points."""
    raw = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    xyz = np.stack([raw[:, o:o + 4].copy().view(np.float32)[:, 0] for o in offsets], axis=1)
    ok = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
    return xyz[ok].astype(np.float64)


class ClearanceMap:
    """Top-down map around the robot (robot frame): distance from each cell to the nearest object.

    Built from object points with a distance transform. Cells outside the map
    read as -1 (not drivable); nothing seen counts as open space.
    """

    def __init__(self, x: np.ndarray, y: np.ndarray, x_min: float = -0.6, x_max: float = 2.6,
                 y_half: float = 1.5, res: float = 0.03) -> None:
        self.x_min, self.y_half, self.res = x_min, y_half, res
        self.nx = int(math.ceil((x_max - x_min) / res))
        self.ny = int(math.ceil(2 * y_half / res))
        free = np.ones((self.nx, self.ny), np.uint8)
        ix, iy = self._index(x, y)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        free[ix[ok], iy[ok]] = 0
        self.n_objects = int((free == 0).sum())
        if self.n_objects:
            self.dist = cv2.distanceTransform(free, cv2.DIST_L2, 5).astype(np.float32) * res
        else:
            self.dist = np.full((self.nx, self.ny), 1e3, np.float32)

    def _index(self, x, y):
        return (np.floor((np.asarray(x) - self.x_min) / self.res).astype(int),
                np.floor((np.asarray(y) + self.y_half) / self.res).astype(int))

    def clearance(self, x, y) -> np.ndarray:
        ix, iy = self._index(x, y)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        out = np.full(np.shape(ix), -1.0, np.float32)
        out[ok] = self.dist[ix[ok], iy[ok]]
        return out

    def objects_within(self, radius: float) -> bool:
        """Is any object closer than radius to the robot (the origin)?"""
        return bool(self.clearance(np.array([0.0]), np.array([0.0]))[0] < radius)


def trace_center(cmap: ClearanceMap, half_road: float, safe: float, length: float = 1.5,
                 step: float = 0.1, max_turn_deg: float = 25.0, turn_cost: float = 0.05
                 ) -> List[Tuple[float, float]]:
    """The centre line ahead of the robot, as points (x, y) in the robot frame, from (0, 0).

    Each step goes 'step' metres in the direction (within +-max_turn_deg of the
    last one) whose end has the most clearance, counted up to half_road: in a
    corridor that is its middle, in a bend it follows the bend, and with only
    one side in view it keeps half a road width from that side. Ties go
    straight on (turn_cost, m of clearance per rad of turn). The trace ends
    where no direction keeps 'safe' metres of clearance.
    """
    turns = np.radians(np.arange(-max_turn_deg, max_turn_deg + 0.1, 5.0))
    px = py = h = 0.0
    path = [(0.0, 0.0)]
    for _ in range(int(round(length / step))):
        hs = h + turns
        qx, qy = px + step * np.cos(hs), py + step * np.sin(hs)
        c = cmap.clearance(qx, qy)
        score = (np.minimum(c, half_road) + 0.05 * np.minimum(c, 2 * half_road)
                 - turn_cost * np.abs(turns))
        score[c < safe] = -np.inf
        k = int(np.argmax(score))
        if not np.isfinite(score[k]):
            break
        px, py, h = float(qx[k]), float(qy[k]), float(hs[k])
        path.append((px, py))
    return path
