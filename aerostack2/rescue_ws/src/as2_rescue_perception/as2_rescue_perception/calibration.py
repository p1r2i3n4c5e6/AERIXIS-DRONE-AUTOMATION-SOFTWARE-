"""Validate calibration structure without inventing or rescaling intrinsics."""
import math


def validate_calibration(data):
    if not isinstance(data, dict):
        raise ValueError('Calibration must be a mapping')
    for key in ('image_width', 'image_height'):
        if type(data.get(key)) is not int or data[key] <= 0:
            raise ValueError(f'Invalid calibration {key}')
    for key, size in [('camera_matrix', 9), ('rectification_matrix', 9),
                      ('projection_matrix', 12), ('distortion_coefficients', None)]:
        values = data[key]['data']
        if size is not None and len(values) != size:
            raise ValueError(f'Invalid calibration {key} length')
        if not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f'Non-finite calibration {key}')
    if float(data['camera_matrix']['data'][0]) <= 0 or float(data['camera_matrix']['data'][4]) <= 0:
        raise ValueError('Camera focal lengths must be positive')
    if not isinstance(data.get('distortion_model'), str) or not data['distortion_model']:
        raise ValueError('Missing distortion model')
    return data
