"""Stateful filters that reproduce the original live-camera behaviour."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


class AnimalProbabilityFilter:
    """Exponential moving average for four animal probabilities."""

    def __init__(self, alpha: float = 0.25) -> None:
        self.alpha = float(alpha)
        self._average: Optional[np.ndarray] = None

    def reset(self) -> None:
        self._average = None

    def update(self, probabilities: np.ndarray) -> np.ndarray:
        current = np.asarray(probabilities, dtype=np.float32)
        if current.shape != (4,):
            raise ValueError('animal probabilities must have shape (4,)')
        if self._average is None:
            self._average = current.copy()
        else:
            self._average = (
                (1.0 - self.alpha) * self._average + self.alpha * current
            )
        return self._average.copy()


def animal_id(probabilities: np.ndarray, threshold: float) -> int:
    """Return 0 for unknown or the fixed one-based animal class ID."""
    index = int(np.argmax(probabilities))
    if float(probabilities[index]) < threshold:
        return 0
    return index + 1


class FruitStabilityFilter:
    """Four-frame stability, centroid EMA, and centre-line hit decision."""

    def __init__(
        self,
        stable_frames: int = 4,
        line_tolerance_px: int = 10,
        centroid_alpha: float = 0.30,
        candidate_max_step_ratio: float = 0.18,
    ) -> None:
        self.stable_frames = max(1, int(stable_frames))
        self.line_tolerance_px = max(0, int(line_tolerance_px))
        self.centroid_alpha = float(centroid_alpha)
        self.candidate_max_step_ratio = float(candidate_max_step_ratio)
        self.reset()

    def reset(self) -> None:
        self.positive_streak = 0
        self._smooth_centroid: Optional[np.ndarray] = None
        self._last_candidate_centroid: Optional[np.ndarray] = None

    def update(
        self,
        class_id: int,
        centroid: Optional[Tuple[float, float]],
        processed_shape: Tuple[int, int],
        frame_shape: Tuple[int, int],
    ) -> Tuple[int, Optional[Tuple[int, int]], int]:
        """Return line_hit, centroid in source-frame pixels, and streak."""
        frame_height, frame_width = frame_shape
        mask_height, mask_width = processed_shape
        raw_positive = class_id == 0 and centroid is not None
        if not raw_positive:
            self.reset()
            return 0, None, 0

        current = np.array(
            [
                centroid[0] * frame_width / mask_width,
                centroid[1] * frame_height / mask_height,
            ],
            dtype=np.float32,
        )
        max_step = self.candidate_max_step_ratio * math.hypot(
            frame_width, frame_height
        )
        same_candidate = (
            self._last_candidate_centroid is None
            or float(np.linalg.norm(current - self._last_candidate_centroid))
            <= max_step
        )
        if same_candidate:
            self.positive_streak += 1
        else:
            self.positive_streak = 1
            self._smooth_centroid = None

        self._last_candidate_centroid = current
        if self._smooth_centroid is None:
            self._smooth_centroid = current
        else:
            self._smooth_centroid = (
                (1.0 - self.centroid_alpha) * self._smooth_centroid
                + self.centroid_alpha * current
            )

        if self.positive_streak < self.stable_frames:
            return 0, None, self.positive_streak

        camera_centroid = (
            int(round(float(self._smooth_centroid[0]))),
            int(round(float(self._smooth_centroid[1]))),
        )
        line_hit = int(
            abs(camera_centroid[0] - frame_width // 2)
            <= self.line_tolerance_px
        )
        return line_hit, camera_centroid, self.positive_streak


class SpatialPercentageFilter:
    """EMA for pig/shit spatial percentages, reset on label transition."""

    def __init__(self, alpha: float = 0.25) -> None:
        self.alpha = float(alpha)
        self.reset()

    def reset(self) -> None:
        self._left: Optional[np.ndarray] = None
        self._right: Optional[np.ndarray] = None
        self._label: Optional[str] = None

    def update(
        self, label: str, left: np.ndarray, right: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        left = np.asarray(left, dtype=np.float32)
        right = np.asarray(right, dtype=np.float32)
        if left.shape != (3,) or right.shape != (3,):
            raise ValueError('spatial percentages must have shape (3,)')
        if self._left is None or label != self._label:
            self._left = left.copy()
            self._right = right.copy()
        else:
            self._left = (1.0 - self.alpha) * self._left + self.alpha * left
            self._right = (
                (1.0 - self.alpha) * self._right + self.alpha * right
            )
        self._label = label
        return self._left.copy(), self._right.copy()
