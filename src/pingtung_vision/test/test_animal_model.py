from pathlib import Path

import numpy as np

from pingtung_vision.algorithms.animal import AnimalClassifier
from pingtung_vision.temporal_filters import animal_id


def test_packaged_animal_checkpoint_runs_on_cpu():
    model_path = (
        Path(__file__).resolve().parents[1]
        / 'models'
        / 'animal_mobilenet_v3_v2.pt'
    )
    classifier = AnimalClassifier(model_path, device_name='cpu')
    probabilities = classifier.predict(
        np.zeros((240, 320, 3), dtype=np.uint8)
    )

    assert probabilities.shape == (4,)
    np.testing.assert_allclose(probabilities.sum(), 1.0, atol=1e-5)
    assert animal_id(probabilities, 0.65) in {0, 1, 2, 3, 4}
