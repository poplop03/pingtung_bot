"""Optional annotated images; no GUI calls are made on the robot."""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

from pingtung_vision.algorithms.fruit_color import Result as FruitResult
from pingtung_vision.algorithms.fruit_color import hit_line_y
from pingtung_vision.algorithms.fruit_color import roi_bounds


ANIMAL_NAMES = ('dog', 'monkey', 'rabbit', 'turtle')


def draw_animal(
    frame: np.ndarray,
    probabilities: np.ndarray,
    animal_id: int,
    fps: float,
) -> np.ndarray:
    output = frame.copy()
    overlay = output.copy()
    cv2.rectangle(overlay, (10, 10), (350, 160), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.65, output, 0.35, 0, output)
    index = int(np.argmax(probabilities))
    label = 'unknown' if animal_id == 0 else ANIMAL_NAMES[index]
    colour = (0, 255, 0) if animal_id else (0, 0, 255)
    cv2.putText(
        output,
        f'ID {animal_id}: {label} ({probabilities[index] * 100:.1f}%)',
        (22, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.70,
        colour,
        2,
        cv2.LINE_AA,
    )
    for probability_index, name in enumerate(ANIMAL_NAMES):
        cv2.putText(
            output,
            f'{probability_index + 1} {name:7s}: '
            f'{probabilities[probability_index] * 100:5.1f}%',
            (22, 70 + probability_index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.53,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
    _draw_fps(output, fps)
    return output


def draw_fruit(
    frame: np.ndarray,
    result: FruitResult,
    line_hit: int,
    centroid: Optional[Tuple[int, int]],
    streak: int,
    stable_frames: int,
    line_tolerance: int,
    roi_width: float,
    roi_height: float,
    fps: float,
) -> np.ndarray:
    output = frame.copy()
    x0, y0, x1, y1 = roi_bounds(
        output.shape[1], output.shape[0], roi_width, roi_height
    )
    line_y = hit_line_y(
        output.shape[1], output.shape[0], roi_width, roi_height
    )
    cv2.rectangle(output, (x0, y0), (x1 - 1, y1 - 1), (255, 180, 0), 2)
    cv2.line(
        output,
        (0, line_y),
        (output.shape[1], line_y),
        (0, 255, 255),
        3,
    )
    if line_tolerance:
        for y in (line_y - line_tolerance, line_y + line_tolerance):
            cv2.line(output, (0, y), (output.shape[1], y), (0, 130, 130), 1)
    if centroid is not None:
        cv2.drawMarker(
            output, centroid, (255, 0, 255), cv2.MARKER_CROSS, 28, 3
        )
    overlay = output.copy()
    cv2.rectangle(overlay, (10, 10), (430, 135), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.65, output, 0.35, 0, output)
    cv2.putText(
        output,
        f'raw={result.label} line_hit={line_hit}',
        (22, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 0) if line_hit else (220, 220, 220),
        2,
    )
    cv2.putText(
        output,
        f'green={result.green_ratio * 100:.1f}% '
        f'red/yellow={result.warm_ratio * 100:.1f}%',
        (22, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 180, 255),
        2,
    )
    cv2.putText(
        output,
        f'stable={min(streak, stable_frames)}/{stable_frames}',
        (22, 108),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 180, 0),
        2,
    )
    _draw_fps(output, fps)
    return output


def draw_pig_shit(
    frame: np.ndarray,
    label: str,
    confidence: float,
    left: np.ndarray,
    right: np.ndarray,
    fps: float,
) -> np.ndarray:
    output = frame.copy()
    middle = output.shape[1] // 2
    cv2.line(output, (middle, 0), (middle, output.shape[0]), (0, 255, 255), 3)
    _draw_spatial_panel(output, 10, 'LEFT', left)
    _draw_spatial_panel(output, middle + 10, 'RIGHT', right)
    cv2.putText(
        output,
        f'{label} score={confidence:.2f}',
        (middle - 95, output.shape[0] - 48),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (0, 255, 0),
        2,
    )
    _draw_fps(output, fps)
    return output


def _draw_spatial_panel(
    frame: np.ndarray, x: int, title: str, values: np.ndarray
) -> None:
    overlay = frame.copy()
    cv2.rectangle(overlay, (x, 10), (x + 230, 138), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.62, frame, 0.38, 0, frame)
    cv2.putText(
        frame, title, (x + 10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
        (0, 255, 255), 2,
    )
    rows = ('Pig', 'Shit', 'Empty')
    colours = ((80, 230, 80), (0, 165, 255), (235, 235, 235))
    for row, (name, colour) in enumerate(zip(rows, colours)):
        cv2.putText(
            frame,
            f'{name:5s}: {values[row]:5.1f}%',
            (x + 10, 63 + row * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            colour,
            2,
        )


def _draw_fps(frame: np.ndarray, fps: float) -> None:
    cv2.putText(
        frame,
        f'FPS: {fps:.1f}',
        (22, frame.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
