"""
image_loader.py - Load images for the calibration system.

Simple image loading with validation and metadata extraction.
"""

from pathlib import Path
import cv2
import numpy as np

from .data_models import ImageFrame


def load_image(path: Path) -> ImageFrame:
    """Load an image file and return an ImageFrame.

    Args:
        path: Path to the image file (JPEG, PNG, etc.).

    Returns:
        ImageFrame with loaded image data and dimensions.

    Raises:
        FileNotFoundError: If the image file does not exist.
        ValueError: If the image cannot be decoded.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Image file not found: {path}")

    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"Failed to decode image: {path}")

    h, w = image.shape[:2]
    return ImageFrame(
        image=image,
        file_path=path,
        width=w,
        height=h,
    )
