"""OpenCV pig/shit classifier and spatial percentage calculation."""

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
    circularity: float
    bbox: Tuple[int, int, int, int]
    score: float


@dataclass
class Result:
    label: str
    confidence: float
    object_mask: np.ndarray
    pig_mask: np.ndarray
    dark_mask: np.ndarray
    bbox: Optional[Tuple[int, int, int, int]]
    pink_area: float
    dark_area: float
    pink_fraction: float
    dark_fraction: float
    circularity: float
    pig_evidence: float
    shit_evidence: float


def resize_for_processing(image: np.ndarray, max_side: int = 384) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, max_side / float(max(height, width)))
    if scale == 1.0:
        return image.copy()
    return cv2.resize(
        image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
    )


def clean_mask(mask: np.ndarray) -> np.ndarray:
    size = max(3, int(round(min(mask.shape) * 0.012)))
    if size % 2 == 0:
        size += 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def colour_masks(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    pig_mask = cv2.inRange(hsv, (0, 32, 75), (27, 255, 255))
    pig_mask |= cv2.inRange(hsv, (165, 32, 75), (179, 255, 255))

    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    otsu_value, _ = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    dark_limit = int(min(115, max(35, otsu_value)))
    dark_mask = cv2.inRange(gray, 0, dark_limit)
    return clean_mask(pig_mask), clean_mask(dark_mask)


def best_component(mask: np.ndarray) -> Optional[Component]:
    height, width = mask.shape
    image_area = float(height * width)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    best: Optional[Component] = None
    for contour in contours:
        area = cv2.contourArea(contour)
        area_ratio = area / image_area
        if area_ratio < 0.004 or area_ratio > 0.80:
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
        perimeter = cv2.arcLength(contour, True)
        circularity = (
            0.0 if perimeter == 0 else 4 * math.pi * area / perimeter**2
        )
        x, y, box_width, box_height = cv2.boundingRect(contour)
        touches_border = (
            x <= 1
            or y <= 1
            or x + box_width >= width - 1
            or y + box_height >= height - 1
        )
        centrality = max(0.0, 1.0 - centre_distance)
        score = area_ratio * (0.35 + 1.65 * centrality**2)
        if touches_border:
            score *= 0.25

        component_mask = np.zeros_like(mask)
        cv2.drawContours(
            component_mask, [contour], -1, 255, thickness=cv2.FILLED
        )
        candidate = Component(
            component_mask,
            area_ratio,
            centre_distance,
            circularity,
            (x, y, box_width, box_height),
            score,
        )
        if best is None or candidate.score > best.score:
            best = candidate
    return best


def refine_with_grabcut(image: np.ndarray, component: Component) -> np.ndarray:
    height, width = image.shape[:2]
    x, y, box_width, box_height = component.bbox
    pad_x = max(4, int(box_width * 0.18))
    pad_y = max(4, int(box_height * 0.18))
    x0 = max(1, x - pad_x)
    y0 = max(1, y - pad_y)
    x1 = min(width - 1, x + box_width + pad_x)
    y1 = min(height - 1, y + box_height + pad_y)

    grab_mask = np.full((height, width), cv2.GC_BGD, np.uint8)
    grab_mask[y0:y1, x0:x1] = cv2.GC_PR_BGD
    seed_size = max(3, int(round(min(height, width) * 0.009)))
    if seed_size % 2 == 0:
        seed_size += 1
    seed_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (seed_size, seed_size)
    )
    probable_fg = cv2.dilate(component.mask, seed_kernel, iterations=1)
    sure_fg = cv2.erode(component.mask, seed_kernel, iterations=1)
    grab_mask[probable_fg > 0] = cv2.GC_PR_FGD
    grab_mask[sure_fg > 0] = cv2.GC_FGD
    background_model = np.zeros((1, 65), np.float64)
    foreground_model = np.zeros((1, 65), np.float64)
    try:
        cv2.grabCut(
            image,
            grab_mask,
            None,
            background_model,
            foreground_model,
            2,
            cv2.GC_INIT_WITH_MASK,
        )
        refined = np.where(
            (grab_mask == cv2.GC_FGD) | (grab_mask == cv2.GC_PR_FGD),
            255,
            0,
        ).astype(np.uint8)
        refined = clean_mask(refined)
        count, labels, _, _ = cv2.connectedComponentsWithStats(refined, 8)
        best_label = 0
        best_overlap = 0
        for label in range(1, count):
            overlap = np.count_nonzero(
                (labels == label) & (component.mask > 0)
            )
            if overlap > best_overlap:
                best_label, best_overlap = label, overlap
        if best_label and best_overlap:
            return np.where(labels == best_label, 255, 0).astype(np.uint8)
    except cv2.error:
        pass
    return component.mask.copy()


def fraction_inside(mask: np.ndarray, object_mask: np.ndarray) -> float:
    object_pixels = object_mask > 0
    count = int(np.count_nonzero(object_pixels))
    if count == 0:
        return 0.0
    return float(np.count_nonzero((mask > 0) & object_pixels) / count)


def classify(
    image: np.ndarray,
    use_grabcut: bool = False,
    max_side: int = 384,
) -> Result:
    """Classify one frame; live mode defaults to the original fast path."""
    image = resize_for_processing(image, max_side=max_side)
    pig_mask, dark_mask = colour_masks(image)
    pig = best_component(pig_mask)
    dark = best_component(dark_mask)

    pig_area = pig.area_ratio if pig else 0.0
    dark_area = dark.area_ratio if dark else 0.0
    pig_evidence = 0.0
    if pig and pig.centre_distance < 0.67:
        pig_evidence = min(1.0, pig.area_ratio / 0.16) * (
            1.0 - 0.60 * pig.centre_distance
        )
    dark_evidence = 0.0
    if dark and dark.centre_distance < 0.60:
        shape_bonus = 0.75 + 0.25 * min(1.0, dark.circularity / 0.65)
        dark_evidence = (
            min(1.0, dark.area_ratio / 0.13)
            * (1.0 - 0.65 * dark.centre_distance)
            * shape_bonus
        )

    if max(pig_evidence, dark_evidence) < 0.22:
        empty = np.zeros(image.shape[:2], np.uint8)
        return Result(
            'unknown',
            max(pig_evidence, dark_evidence),
            empty,
            pig_mask,
            dark_mask,
            None,
            pig_area,
            dark_area,
            0.0,
            0.0,
            0.0,
            pig_evidence,
            dark_evidence,
        )

    if pig_evidence >= dark_evidence:
        chosen, initial_label = pig, 'pig'
    else:
        chosen, initial_label = dark, 'shit'
    assert chosen is not None
    object_mask = (
        refine_with_grabcut(image, chosen)
        if use_grabcut
        else chosen.mask.copy()
    )
    pink_fraction = fraction_inside(pig_mask, object_mask)
    dark_fraction = fraction_inside(dark_mask, object_mask)
    if pink_fraction >= 0.22 and pig_evidence >= 0.18:
        label = 'pig'
        confidence = 0.55 * pig_evidence + 0.45 * min(
            1.0, pink_fraction / 0.65
        )
    elif dark_fraction >= 0.35 and dark_evidence >= 0.18:
        label = 'shit'
        confidence = 0.55 * dark_evidence + 0.45 * min(
            1.0, dark_fraction / 0.80
        )
    else:
        label = (
            initial_label
            if max(pig_evidence, dark_evidence) >= 0.45
            else 'unknown'
        )
        confidence = 0.65 * max(pig_evidence, dark_evidence)
    confidence = float(np.clip(confidence, 0.0, 1.0))
    return Result(
        label,
        confidence,
        object_mask,
        pig_mask,
        dark_mask,
        chosen.bbox,
        pig_area,
        dark_area,
        pink_fraction,
        dark_fraction,
        chosen.circularity,
        pig_evidence,
        dark_evidence,
    )


def object_spatial_percentages(
    result: Result,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return Pig/Shit/Empty percentages for left and right display panels."""
    left_percent = np.zeros(3, dtype=np.float32)
    right_percent = np.zeros(3, dtype=np.float32)
    object_pixels = result.object_mask > 0
    if result.label not in {'pig', 'shit'} or not np.any(object_pixels):
        left_percent[2] = 100.0
        right_percent[2] = 100.0
        return left_percent, right_percent

    middle = object_pixels.shape[1] // 2
    left_pixels = int(np.count_nonzero(object_pixels[:, :middle]))
    right_pixels = int(np.count_nonzero(object_pixels[:, middle:]))
    total_pixels = left_pixels + right_pixels
    if total_pixels == 0:
        left_percent[2] = 100.0
        right_percent[2] = 100.0
        return left_percent, right_percent

    left_share = 100.0 * left_pixels / total_pixels
    right_share = 100.0 - left_share
    class_index = 0 if result.label == 'pig' else 1
    left_percent[class_index] = left_share
    right_percent[class_index] = right_share
    left_percent[2] = 100.0 - left_share
    right_percent[2] = 100.0 - right_share
    return left_percent, right_percent
