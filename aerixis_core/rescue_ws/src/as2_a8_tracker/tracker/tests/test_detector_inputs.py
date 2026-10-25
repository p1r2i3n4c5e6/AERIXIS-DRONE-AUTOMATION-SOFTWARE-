"""Compare every uint8 intensity after packing, without running a camera."""
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from detector_models import NCNNDetector
from hybrid_tracker import INPUT_SIZE, _letterbox


class PackingTests(unittest.TestCase):
    def test_rgb_normalization_and_letterbox_unchanged(self):
        ramp = np.arange(256, dtype=np.uint8)
        all_values = np.stack([ramp, ramp[::-1], np.roll(ramp, 17)], axis=1)
        images = [np.tile(all_values[None, ...], (256, 1, 1))]
        rng = np.random.default_rng(71)
        images.extend(rng.integers(0, 256, (h, w, 3), dtype=np.uint8)
                      for h, w in [(360, 640), (481, 277), (107, 799)])
        for image in images:
            padded = _letterbox(image)[0]
            reference = cv2.dnn.blobFromImage(
                padded, scalefactor=1./255., size=tuple(INPUT_SIZE), swapRB=True, crop=False)[0]
            actual = np.array(NCNNDetector.make_tensor(padded))
            np.testing.assert_allclose(actual, reference, rtol=0., atol=1e-7)


if __name__ == "__main__":
    unittest.main()
