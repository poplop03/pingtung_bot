import cv2
import numpy as np

from pingtung_vision.algorithms.fruit_color import classify as classify_fruit
from pingtung_vision.algorithms.pig_shit import (
    classify as classify_pig_shit,
)
from pingtung_vision.algorithms.pig_shit import object_spatial_percentages


def test_fruit_red_circle_and_empty_frame():
    red_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.circle(red_frame, (320, 240), 55, (0, 0, 255), cv2.FILLED)
    red = classify_fruit(
        red_frame, max_side=384, roi_width=0.68, roi_height=0.74
    )
    assert red.class_id == 0
    assert red.label == 'red/yellow'
    assert red.centroid is not None
    assert red.warm_ratio > 0.99

    empty = classify_fruit(
        np.zeros_like(red_frame),
        max_side=384,
        roi_width=0.68,
        roi_height=0.74,
    )
    assert empty.class_id == 1
    assert empty.label == 'green/empty'
    assert empty.centroid is None


def test_pig_and_unknown_spatial_outputs():
    frame = np.full((384, 512, 3), 255, dtype=np.uint8)
    pink_hsv = np.uint8([[[10, 150, 220]]])
    pink_bgr = tuple(
        int(value)
        for value in cv2.cvtColor(pink_hsv, cv2.COLOR_HSV2BGR)[0, 0]
    )
    cv2.ellipse(frame, (190, 192), (65, 45), 0, 0, 360, pink_bgr, -1)
    result = classify_pig_shit(frame, use_grabcut=False, max_side=384)
    assert result.label == 'pig'
    left, right = object_spatial_percentages(result)
    assert left[0] > right[0]
    np.testing.assert_allclose(left.sum(), 100.0, atol=1e-5)
    np.testing.assert_allclose(right.sum(), 100.0, atol=1e-5)

    unknown = classify_pig_shit(
        np.full_like(frame, 255), use_grabcut=False, max_side=384
    )
    assert unknown.label == 'unknown'
    left, right = object_spatial_percentages(unknown)
    np.testing.assert_array_equal(left, [0.0, 0.0, 100.0])
    np.testing.assert_array_equal(right, [0.0, 0.0, 100.0])
