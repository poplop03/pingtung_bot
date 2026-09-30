"""OpenCV red/yellow versus green/empty fruit classification."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class Component:
    mask: np.ndarray
    area_ratio: float
    centre_distance: float
    centroid: Tuple[float, float]
    bbox: Tuple[int, int, int, int]
    score: float


@dataclass
class Result:
    class_id: int
    label: str
    confidence: float
    green_ratio: float
    warm_ratio: float
    area_ratio: float
    centroid: Optional[Tuple[float, float]]
    bbox: Optional[Tuple[int, int, int, int]]
    fruit_mask: np.ndarray
    green_mask: np.ndarray
    warm_mask: np.ndarray


def resize_for_processing(image: np.ndarray, max_side: int = 480) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, max_side / float(max(height, width)))
    if scale == 1.0:
        return image.copy()
    return cv2.resize(
        image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
    )


def clean_mask(mask: np.ndarray) -> np.ndarray:
    size = max(3, int(round(min(mask.shape) * 0.009)))
    if size % 2 == 0:
        size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def colour_masks(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    red_low = cv2.inRange(hsv, (0, 65, 50), (13, 255, 255))
    red_high = cv2.inRange(hsv, (168, 65, 50), (179, 255, 255))
    yellow = cv2.inRange(hsv, (14, 65, 65), (37, 255, 255))
    green = cv2.inRange(hsv, (38, 45, 40), (95, 255, 255))
    warm = clean_mask(red_low | red_high | yellow)
    green = clean_mask(green)
    fruit = clean_mask(warm | green)
    return fruit, green, warm


def roi_bounds(
    width: int, height: int, roi_width: float, roi_height: float
) -> Tuple[int, int, int, int]:
    roi_width = min(1.0, max(0.20, roi_width))
    roi_height = min(1.0, max(0.20, roi_height))
    box_width = max(2, int(round(width * roi_width)))
    box_height = max(2, int(round(height * roi_height)))
    x0 = (width - box_width) // 2
    y0 = (height - box_height) // 2
    return x0, y0, x0 + box_width, y0 + box_height


def hit_line_y(
    width: int, height: int, roi_width: float, roi_height: float
) -> int:
    """Hit-line row: one quarter of the ROI height above its bottom edge."""
    _, y0, _, y1 = roi_bounds(width, height, roi_width, roi_height)
    return y1 - (y1 - y0) // 4


def best_component(
    mask: np.ndarray, roi_width: float = 1.0, roi_height: float = 1.0
) -> Optional[Component]:
    height, width = mask.shape
    image_area = float(height * width)
    x0, y0, x1, y1 = roi_bounds(width, height, roi_width, roi_height)
    roi_mask = np.zeros_like(mask)
    roi_mask[y0:y1, x0:x1] = 255
    search_mask = cv2.bitwise_and(mask, roi_mask)
    contours, _ = cv2.findContours(
        search_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    best: Optional[Component] = None
    for contour in contours:
        area = cv2.contourArea(contour)
        area_ratio = area / image_area
        if area_ratio < 0.0015 or area_ratio > 0.28:
            continue
        x, y, box_width, box_height = cv2.boundingRect(contour)
        touches_roi = (
            x <= x0 + 1
            or y <= y0 + 1
            or x + box_width >= x1 - 1
            or y + box_height >= y1 - 1
        )
        if touches_roi:
            continue

        aspect_ratio = max(
            box_width / max(1.0, float(box_height)),
            box_height / max(1.0, float(box_width)),
        )
        extent = area / max(1.0, float(box_width * box_height))
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull_area if hull_area > 0 else 0.0
        box_area_ratio = (box_width * box_height) / image_area
        if (
            aspect_ratio > 4.5
            or extent < 0.24
            or solidity < 0.52
            or box_area_ratio > 0.52
        ):
            continue

        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            continue
        centre_x = moments['m10'] / moments['m00']
        centre_y = moments['m01'] / moments['m00']
        centre_distance = math.hypot(
            (centre_x - width / 2) / (width / 2),
            (centre_y - height / 2) / (height / 2),
        ) / math.sqrt(2)
        centrality = max(0.0, 1.0 - centre_distance)
        shape_quality = min(1.0, extent / 0.65) * min(1.0, solidity / 0.85)
        score = (
            math.sqrt(area_ratio)
            * (0.15 + 1.85 * centrality**4)
            * (0.55 + 0.45 * shape_quality)
        )
        component_mask = np.zeros_like(mask)
        cv2.drawContours(component_mask, [contour], -1, 255, cv2.FILLED)
        component = Component(
            component_mask,
            area_ratio,
            centre_distance,
            (centre_x, centre_y),
            (x, y, box_width, box_height),
            score,
        )
        if best is None or component.score > best.score:
            best = component
    return best


def classify(
    image: np.ndarray,
    max_side: int = 480,
    roi_width: float = 1.0,
    roi_height: float = 1.0,
) -> Result:
    """Return 0 for red/yellow, or 1 for green/empty."""
    resized = resize_for_processing(image, max_side)
    fruit_mask, green_mask, warm_mask = colour_masks(resized)
    component = best_component(fruit_mask, roi_width, roi_height)
    if (
        component is None
        or component.centre_distance > 0.62
        or component.area_ratio < 0.006
    ):
        empty = np.zeros(resized.shape[:2], dtype=np.uint8)
        return Result(
            1,
            'green/empty',
            1.0,
            0.0,
            0.0,
            0.0,
            None,
            None,
            empty,
            green_mask,
            warm_mask,
        )

    object_pixels = component.mask > 0
    green_pixels = int(np.count_nonzero((green_mask > 0) & object_pixels))
    warm_pixels = int(np.count_nonzero((warm_mask > 0) & object_pixels))
    coloured_pixels = green_pixels + warm_pixels
    if coloured_pixels < 20:
        return Result(
            1,
            'green/empty',
            1.0,
            0.0,
            0.0,
            component.area_ratio,
            None,
            component.bbox,
            component.mask,
            green_mask,
            warm_mask,
        )

    green_ratio = green_pixels / coloured_pixels
    warm_ratio = warm_pixels / coloured_pixels
    class_id = 0 if warm_ratio >= 0.55 else 1
    label = 'red/yellow' if class_id == 0 else 'green/empty'
    confidence = warm_ratio if class_id == 0 else green_ratio
    centroid = component.centroid if class_id == 0 else None
    return Result(
        class_id,
        label,
        confidence,
        green_ratio,
        warm_ratio,
        component.area_ratio,
        centroid,
        component.bbox,
        component.mask,
        green_mask,
        warm_mask,
    )
