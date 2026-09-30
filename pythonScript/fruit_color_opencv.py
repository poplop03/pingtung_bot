"""Classify fruit colour using OpenCV only.

Output mapping:
    0 = red or yellow fruit
    1 = green fruit or no fruit

The fruit type is intentionally ignored. The algorithm segments red, yellow and
green pixels, selects the connected coloured component nearest the image centre,
and compares green pixels with red/yellow pixels inside that component.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Tuple

import cv2
import numpy as np


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


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


def imread(path: Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot read image: {path}")
    return image


def imwrite(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix if path.suffix else ".jpg"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError(f"Cannot encode image: {path}")
    encoded.tofile(str(path))


def resize_for_processing(image: np.ndarray, max_side: int = 480) -> np.ndarray:
    height, width = image.shape[:2]
    scale = min(1.0, max_side / float(max(height, width)))
    if scale == 1.0:
        return image.copy()
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)


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

    # OpenCV hue range is 0..179. Saturation/value limits suppress the white
    # table, grey shadows and black desk objects.
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
    """Return a centred ROI as x0, y0, x1, y1 (x1/y1 are exclusive)."""
    roi_width = min(1.0, max(0.20, roi_width))
    roi_height = min(1.0, max(0.20, roi_height))
    box_width = max(2, int(round(width * roi_width)))
    box_height = max(2, int(round(height * roi_height)))
    x0 = (width - box_width) // 2
    y0 = (height - box_height) // 2
    return x0, y0, x0 + box_width, y0 + box_height


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

        # Background such as a coloured wall, curtain or table is normally cut
        # by the ROI. Only accept closed components fully contained in the ROI.
        touches_roi = (
            x <= x0 + 1 or y <= y0 + 1
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
        if moments["m00"] == 0:
            continue
        centre_x = moments["m10"] / moments["m00"]
        centre_y = moments["m01"] / moments["m00"]
        centre_distance = math.hypot(
            (centre_x - width / 2) / (width / 2),
            (centre_y - height / 2) / (height / 2),
        ) / math.sqrt(2)
        centrality = max(0.0, 1.0 - centre_distance)
        # sqrt(area) lets a small central chilli compete with larger coloured
        # clutter. Compact, solid components receive a small additional bonus.
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
    resized = resize_for_processing(image, max_side)
    fruit_mask, green_mask, warm_mask = colour_masks(resized)
    component = best_component(fruit_mask, roi_width, roi_height)
    # Conservative negative result: green fruit and an empty frame intentionally
    # share output 1. A minimum area also prevents small red desk clutter from
    # becoming a false red/yellow detection.
    if (
        component is None
        or component.centre_distance > 0.62
        or component.area_ratio < 0.006
    ):
        empty = np.zeros(resized.shape[:2], dtype=np.uint8)
        return Result(1, "green/empty", 1.0, 0.0, 0.0, 0.0, None, None,
                      empty, green_mask, warm_mask)

    object_pixels = component.mask > 0
    green_pixels = int(np.count_nonzero((green_mask > 0) & object_pixels))
    warm_pixels = int(np.count_nonzero((warm_mask > 0) & object_pixels))
    coloured_pixels = green_pixels + warm_pixels
    if coloured_pixels < 20:
        return Result(1, "green/empty", 1.0, 0.0, 0.0, component.area_ratio,
                      None, component.bbox, component.mask, green_mask, warm_mask)

    green_ratio = green_pixels / coloured_pixels
    warm_ratio = warm_pixels / coloured_pixels
    # Only red/yellow is a positive detection. Green and every non-detection
    # intentionally return the same output (1).
    class_id = 0 if warm_ratio >= 0.55 else 1
    label = "red/yellow" if class_id == 0 else "green/empty"
    confidence = warm_ratio if class_id == 0 else green_ratio
    centroid = component.centroid if class_id == 0 else None
    return Result(
        class_id, label, confidence, green_ratio, warm_ratio,
        component.area_ratio, centroid, component.bbox, component.mask,
        green_mask, warm_mask
    )


def make_debug_view(image: np.ndarray, result: Result) -> np.ndarray:
    resized = resize_for_processing(image, result.fruit_mask.shape[0] if image.shape[0] >= image.shape[1]
                                    else result.fruit_mask.shape[1])
    annotated = resized.copy()
    colour = (0, 220, 0) if result.class_id == 1 else (0, 180, 255)
    if result.bbox:
        x, y, width, height = result.bbox
        cv2.rectangle(annotated, (x, y), (x + width, y + height), colour, 3)
    if result.centroid:
        centre = (round(result.centroid[0]), round(result.centroid[1]))
        cv2.drawMarker(annotated, centre, (255, 0, 255), cv2.MARKER_CROSS,
                       22, 3, cv2.LINE_AA)
    text = f"ID {result.class_id}: {result.label}  {result.confidence * 100:.1f}%"
    cv2.putText(annotated, text, (12, 32), cv2.FONT_HERSHEY_SIMPLEX,
                0.72, colour, 2, cv2.LINE_AA)

    def panel(mask: np.ndarray, title: str) -> np.ndarray:
        view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.putText(view, title, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 255), 2, cv2.LINE_AA)
        return view

    top = cv2.hconcat([annotated, panel(result.fruit_mask, "fruit mask")])
    bottom = cv2.hconcat(
        [panel(result.green_mask, "green mask"), panel(result.warm_mask, "red/yellow mask")]
    )
    return cv2.vconcat([top, bottom])


def image_paths(root: Path) -> Iterable[Path]:
    if root.is_file():
        yield root
        return
    yield from sorted(
        path for path in root.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )


def safe_path(path: Path) -> str:
    return str(path).encode("ascii", "backslashreplace").decode("ascii")


def run_paths(input_path: Path, save_dir: Optional[Path], show: bool) -> int:
    paths = list(image_paths(input_path))
    if not paths:
        print(f"No images found in: {input_path}", file=sys.stderr)
        return 2
    summary = {}
    for path in paths:
        try:
            image = imread(path)
            result = classify(image)
            print(
                f"{safe_path(path)}: id={result.class_id:2d} {result.label:10s} "
                f"confidence={result.confidence:.3f} green={result.green_ratio:.3f} "
                f"warm={result.warm_ratio:.3f} area={result.area_ratio:.3f} "
                f"centroid={result.centroid}"
            )
            key = (path.parent.name, result.class_id)
            summary[key] = summary.get(key, 0) + 1
            debug = make_debug_view(image, result)
            if save_dir:
                relative = path.name if input_path.is_file() else path.relative_to(input_path)
                imwrite(save_dir / Path(relative).with_suffix(".jpg"), debug)
            if show:
                cv2.imshow("Fruit colour classifier", debug)
                if cv2.waitKeyEx(0) & 0xFF in (27, ord("q")):
                    break
        except (OSError, ValueError, cv2.error) as error:
            print(f"ERROR {safe_path(path)}: {error}", file=sys.stderr)
    if show:
        cv2.destroyAllWindows()
    print("Summary by source folder and output ID:")
    for (folder, class_id), count in sorted(summary.items()):
        print(f"  {folder:12s} -> {class_id:2d}: {count}")
    return 0


def draw_camera_result(
    frame: np.ndarray,
    result: Result,
    fps: float,
    centroid: Optional[Tuple[int, int]],
    line_hit: int,
    line_tolerance: int,
    roi_width: float,
    roi_height: float,
    stable_count: int,
    stable_frames: int,
) -> np.ndarray:
    output = frame.copy()
    colour = (0, 220, 0) if result.class_id == 1 else (0, 180, 255)
    overlay = output.copy()
    middle_x = output.shape[1] // 2
    roi_x0, roi_y0, roi_x1, roi_y1 = roi_bounds(
        output.shape[1], output.shape[0], roi_width, roi_height
    )
    cv2.rectangle(
        output, (roi_x0, roi_y0), (roi_x1 - 1, roi_y1 - 1),
        (255, 180, 0), 2, cv2.LINE_AA,
    )
    cv2.line(output, (middle_x, 0), (middle_x, output.shape[0]),
             (0, 255, 255), 3, cv2.LINE_AA)
    if line_tolerance > 1:
        cv2.line(output, (middle_x - line_tolerance, 0),
                 (middle_x - line_tolerance, output.shape[0]), (0, 130, 130), 1)
        cv2.line(output, (middle_x + line_tolerance, 0),
                 (middle_x + line_tolerance, output.shape[0]), (0, 130, 130), 1)
    if centroid is not None:
        cv2.drawMarker(output, centroid, (255, 0, 255), cv2.MARKER_CROSS,
                       28, 3, cv2.LINE_AA)
        cv2.putText(output, f"centroid=({centroid[0]}, {centroid[1]})",
                    (centroid[0] + 12, max(25, centroid[1] - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2,
                    cv2.LINE_AA)
    cv2.rectangle(overlay, (10, 10), (460, 176), (0, 0, 0), cv2.FILLED)
    cv2.addWeighted(overlay, 0.65, output, 0.35, 0, output)
    cv2.putText(output, f"OUTPUT {result.class_id}: {result.label}", (22, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.78, colour, 2, cv2.LINE_AA)
    cv2.putText(output, f"Green: {result.green_ratio * 100:5.1f}%", (22, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 220, 0), 2, cv2.LINE_AA)
    cv2.putText(output, f"Red/yellow: {result.warm_ratio * 100:5.1f}%", (22, 101),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 180, 255), 2, cv2.LINE_AA)
    hit_colour = (0, 255, 0) if line_hit else (220, 220, 220)
    cv2.putText(output, f"Line hit: {line_hit}", (22, 130),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, hit_colour, 2, cv2.LINE_AA)
    cv2.putText(
        output, f"Stable: {min(stable_count, stable_frames)}/{stable_frames}",
        (22, 159), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 180, 0), 2,
        cv2.LINE_AA,
    )
    cv2.putText(output, f"FPS: {fps:.1f}", (22, output.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
    return output


def run_camera(
    index: int,
    line_tolerance: int,
    roi_width: float,
    roi_height: float,
    stable_frames: int,
) -> int:
    camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        print(f"Cannot open camera {index}", file=sys.stderr)
        return 2
    smooth_centroid: Optional[np.ndarray] = None
    fps_average: Optional[float] = None
    last_reported: Optional[Tuple[int, int]] = None
    positive_streak = 0
    last_candidate_centroid: Optional[np.ndarray] = None
    try:
        while True:
            start = time.perf_counter()
            ok, frame = camera.read()
            if not ok:
                break
            result = classify(
                frame, max_side=384,
                roi_width=roi_width, roi_height=roi_height,
            )
            camera_centroid: Optional[Tuple[int, int]] = None
            line_hit = 0
            raw_positive = result.class_id == 0 and result.centroid is not None
            if raw_positive:
                mask_height, mask_width = result.fruit_mask.shape
                current_centroid = np.array(
                    [
                        result.centroid[0] * frame.shape[1] / mask_width,
                        result.centroid[1] * frame.shape[0] / mask_height,
                    ],
                    dtype=np.float32,
                )
                max_step = 0.18 * math.hypot(frame.shape[1], frame.shape[0])
                same_candidate = (
                    last_candidate_centroid is None
                    or float(np.linalg.norm(current_centroid - last_candidate_centroid))
                    <= max_step
                )
                if same_candidate:
                    positive_streak += 1
                else:
                    positive_streak = 1
                    smooth_centroid = None
                last_candidate_centroid = current_centroid
                smooth_centroid = (
                    current_centroid if smooth_centroid is None
                    else 0.70 * smooth_centroid + 0.30 * current_centroid
                )
                if positive_streak >= stable_frames:
                    camera_centroid = (
                        int(round(float(smooth_centroid[0]))),
                        int(round(float(smooth_centroid[1]))),
                    )
                    line_hit = int(
                        abs(camera_centroid[0] - frame.shape[1] // 2)
                        <= line_tolerance
                    )
                else:
                    # A candidate is not output 0 until it remains positive for
                    # the requested number of consecutive frames.
                    result.class_id = 1
                    result.label = "green/empty"
                    result.confidence = 1.0 - result.warm_ratio
            else:
                positive_streak = 0
                smooth_centroid = None
                last_candidate_centroid = None
            elapsed = max(time.perf_counter() - start, 1e-6)
            fps_now = 1.0 / elapsed
            fps_average = fps_now if fps_average is None else 0.90 * fps_average + 0.10 * fps_now
            reported = (result.class_id, line_hit)
            if reported != last_reported:
                print(
                    f"color_id={result.class_id}, line_hit={line_hit}, "
                    f"centroid={camera_centroid}",
                    flush=True,
                )
                last_reported = reported
            cv2.imshow("Fruit colour classifier - q to quit",
                       draw_camera_result(
                           frame, result, fps_average, camera_centroid,
                           line_hit, line_tolerance, roi_width, roi_height,
                           positive_streak, stable_frames,
                       ))
            if cv2.pollKey() & 0xFF in (27, ord("q")):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Classify green versus red/yellow fruit")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", type=Path, help="Image or directory")
    source.add_argument("--camera", type=int, help="Camera index, for example 2")
    parser.add_argument("--save-dir", type=Path, help="Directory for debug images")
    parser.add_argument("--show", action="store_true", help="Show image debug window")
    parser.add_argument(
        "--line-tolerance",
        type=int,
        default=10,
        help="Half-width in pixels of the centre-line hit zone (default: 10)",
    )
    parser.add_argument(
        "--roi-width", type=float, default=0.68,
        help="Central ROI width as a frame fraction (default: 0.68)",
    )
    parser.add_argument(
        "--roi-height", type=float, default=0.74,
        help="Central ROI height as a frame fraction (default: 0.74)",
    )
    parser.add_argument(
        "--stable-frames", type=int, default=4,
        help="Consecutive positive frames required for output 0 (default: 4)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print("Output mapping: 0=red/yellow, 1=green/empty")
    if args.camera is not None:
        return run_camera(
            args.camera,
            max(0, args.line_tolerance),
            min(1.0, max(0.20, args.roi_width)),
            min(1.0, max(0.20, args.roi_height)),
            max(1, args.stable_frames),
        )
    return run_paths(args.input, args.save_dir, args.show)


if __name__ == "__main__":
    raise SystemExit(main())
