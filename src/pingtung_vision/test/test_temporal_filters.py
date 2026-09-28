import numpy as np

from pingtung_vision.temporal_filters import (
    AnimalProbabilityFilter,
    FruitStabilityFilter,
    SpatialPercentageFilter,
    animal_id,
)


def test_animal_ema_and_threshold():
    probability_filter = AnimalProbabilityFilter(alpha=0.25)
    first = probability_filter.update(
        np.array([0.70, 0.10, 0.10, 0.10], dtype=np.float32)
    )
    second = probability_filter.update(
        np.array([0.10, 0.70, 0.10, 0.10], dtype=np.float32)
    )

    np.testing.assert_allclose(first, [0.70, 0.10, 0.10, 0.10])
    np.testing.assert_allclose(second, [0.55, 0.25, 0.10, 0.10])
    assert animal_id(second, 0.65) == 0
    assert animal_id(second, 0.50) == 1


def test_fruit_requires_four_central_frames_and_resets():
    stability_filter = FruitStabilityFilter(
        stable_frames=4, line_tolerance_px=10
    )
    arguments = (0, (50.0, 50.0), (100, 100), (480, 640))

    for expected_streak in range(1, 4):
        hit, centroid, streak = stability_filter.update(*arguments)
        assert hit == 0
        assert centroid is None
        assert streak == expected_streak

    hit, centroid, streak = stability_filter.update(*arguments)
    assert hit == 1
    assert centroid == (320, 240)
    assert streak == 4

    hit, centroid, streak = stability_filter.update(
        1, None, (100, 100), (480, 640)
    )
    assert (hit, centroid, streak) == (0, None, 0)

    hit, _, streak = stability_filter.update(*arguments)
    assert hit == 0
    assert streak == 1


def test_fruit_stable_but_off_centre_is_not_a_hit():
    stability_filter = FruitStabilityFilter(stable_frames=4)
    output = None
    for _ in range(4):
        output = stability_filter.update(
            0, (20.0, 50.0), (100, 100), (480, 640)
        )
    assert output is not None
    assert output[0] == 0
    assert output[1] == (128, 240)


def test_spatial_ema_and_label_transition_reset():
    spatial_filter = SpatialPercentageFilter(alpha=0.25)
    left, right = spatial_filter.update(
        'pig', np.array([80, 0, 20]), np.array([20, 0, 80])
    )
    np.testing.assert_allclose(left, [80, 0, 20])
    np.testing.assert_allclose(right, [20, 0, 80])

    left, right = spatial_filter.update(
        'pig', np.array([40, 0, 60]), np.array([60, 0, 40])
    )
    np.testing.assert_allclose(left, [70, 0, 30])
    np.testing.assert_allclose(right, [30, 0, 70])

    left, right = spatial_filter.update(
        'shit', np.array([0, 10, 90]), np.array([0, 90, 10])
    )
    np.testing.assert_allclose(left, [0, 10, 90])
    np.testing.assert_allclose(right, [0, 90, 10])
