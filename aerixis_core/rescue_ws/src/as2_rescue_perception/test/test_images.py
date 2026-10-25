from types import SimpleNamespace
import numpy as np
import pytest
from as2_rescue_perception.image_utils import LatestFrame, decode_bgr8


def test_duplicate_and_stale_frames_never_republished():
    cache = LatestFrame()
    frame = np.zeros((2, 2, 3), dtype=np.uint8)
    cache.put(frame, 1.0, 'stamp')
    seq, actual, stamp = cache.take(0, 1.1, 0.5)
    assert actual is frame and stamp == 'stamp'
    assert cache.take(seq, 1.2, 0.5) is None
    assert cache.take(0, 2.0, 0.5) is None
    assert cache.take(0, 0.5, 0.5) is None
    cache.clear()
    assert cache.take(0, 1.1, 0.5) is None
    cache.put(frame, 3.0, 'new')
    assert cache.take(seq, 3.1, 0.5)[2] == 'new'


def test_padded_bgr_image_and_corrupt_buffers():
    msg = SimpleNamespace(encoding='bgr8', width=1, height=2, step=4,
                          data=bytes([1, 2, 3, 0, 4, 5, 6, 0]))
    assert decode_bgr8(msg).tolist() == [[[1, 2, 3]], [[4, 5, 6]]]
    msg.data = b'bad'
    with pytest.raises(ValueError):
        decode_bgr8(msg)
    msg.encoding = 'rgb8'
    with pytest.raises(ValueError):
        decode_bgr8(msg)


def test_calibration_rejects_unknown_or_invalid_intrinsics():
    from as2_rescue_perception.calibration import validate_calibration
    data = dict(image_width=640, image_height=480, distortion_model='plumb_bob',
                camera_matrix={'data': [500, 0, 320, 0, 500, 240, 0, 0, 1]},
                rectification_matrix={'data': [1, 0, 0, 0, 1, 0, 0, 0, 1]},
                projection_matrix={'data': [500, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0]},
                distortion_coefficients={'data': [0, 0, 0, 0, 0]})
    assert validate_calibration(data) is data
    data['camera_matrix']['data'][0] = 0
    with pytest.raises(ValueError):
        validate_calibration(data)
    data['camera_matrix']['data'][0] = float('nan')
    with pytest.raises(ValueError):
        validate_calibration(data)
