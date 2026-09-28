"""MobileNetV3 animal classification runtime."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from torch import nn
from torchvision import transforms
from torchvision.models import mobilenet_v3_small


CLASS_NAMES = ('dog', 'monkey', 'rabbit', 'turtle')
CLASS_TO_INDEX = {name: index for index, name in enumerate(CLASS_NAMES)}


def build_model() -> nn.Module:
    """Construct the exact MobileNetV3-Small architecture used for training."""
    model = mobilenet_v3_small(weights=None)
    in_features = model.classifier[-1].in_features
    model.classifier[2] = nn.Dropout(p=0.35, inplace=True)
    model.classifier[-1] = nn.Linear(in_features, len(CLASS_NAMES))
    return model


class AnimalClassifier:
    """Load the trained checkpoint and return dog/monkey/rabbit/turtle scores."""

    def __init__(self, model_path: Path, device_name: str = 'auto') -> None:
        self.model_path = Path(model_path)
        if device_name == 'auto':
            device_name = 'cuda' if torch.cuda.is_available() else 'cpu'
        if device_name not in {'cpu', 'cuda'}:
            raise ValueError("animal.device must be 'auto', 'cpu', or 'cuda'")
        if device_name == 'cuda' and not torch.cuda.is_available():
            raise RuntimeError('CUDA requested, but torch.cuda.is_available() is false')
        self.device = torch.device(device_name)

        checkpoint = torch.load(
            self.model_path,
            map_location=self.device,
            weights_only=True,
        )
        checkpoint_mapping = checkpoint.get('class_to_index')
        if checkpoint_mapping is not None and checkpoint_mapping != CLASS_TO_INDEX:
            raise ValueError(
                'animal checkpoint class mapping does not match '
                f'{CLASS_TO_INDEX}: {checkpoint_mapping}'
            )
        self.model = build_model()
        self.model.load_state_dict(checkpoint['model_state'])
        self.model.eval().to(self.device)
        self.image_size = int(checkpoint.get('image_size', 160))
        self.transform = transforms.Compose(
            [
                transforms.Resize((self.image_size, self.image_size)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=(0.485, 0.456, 0.406),
                    std=(0.229, 0.224, 0.225),
                ),
            ]
        )

    def predict(self, frame: np.ndarray) -> np.ndarray:
        """Return probabilities in fixed dog, monkey, rabbit, turtle order."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        tensor = self.transform(Image.fromarray(rgb)).unsqueeze(0).to(self.device)
        with torch.inference_mode():
            probabilities = torch.softmax(self.model(tensor), dim=1)[0]
        return probabilities.cpu().numpy().astype(np.float32, copy=False)
