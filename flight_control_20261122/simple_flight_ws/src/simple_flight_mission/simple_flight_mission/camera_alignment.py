"""Offline camera/body mapping. No ROS, aircraft motion or hardware commands."""
import argparse
import json
import math
from pathlib import Path
from .visual_task import estimate_jacobian, inverse, finite


def default_alignment_path():
    return Path.home() / 'simple_flight_ws/config/camera_alignment.json'


def validate_alignment(value):
    if not isinstance(value, dict) or value.get('schema') != 1:
        raise ValueError('Camera alignment schema must be 1')
    if value.get('enabled') is not True:
        raise ValueError('Camera alignment must explicitly be enabled')
    height = value.get('reference_height_m')
    size = value.get('image_size')
    matrix = value.get('body_jacobian_px_per_m')
    if not finite(height) or not .2 <= height <= 2.:
        raise ValueError('Invalid camera alignment reference height')
    if (not isinstance(size, list) or len(size) != 2
            or any(type(v) is not int or v <= 0 for v in size)):
        raise ValueError('Invalid camera alignment image size')
    if (not isinstance(matrix, (list, tuple)) or len(matrix) != 2
            or any(not isinstance(row, (list, tuple)) or len(row) != 2 for row in matrix)
            or any(not finite(v) for row in matrix for v in row)):
        raise ValueError('Invalid camera/body Jacobian')
    inv = inverse(matrix)
    norm = math.sqrt(sum(v*v for row in matrix for v in row))
    inv_norm = math.sqrt(sum(v*v for row in inv for v in row))
    if not 30 <= norm <= 10000 or norm*inv_norm > 12:
        raise ValueError('Camera/body Jacobian scale or conditioning invalid')
    return value


def load_alignment(path=None):
    path = Path(path) if path is not None else default_alignment_path()
    if not path.exists():
        return None
    value = json.loads(path.read_text(encoding='utf-8'))
    if isinstance(value, dict) and value.get('schema') == 1 and value.get('enabled') is False:
        return None
    return validate_alignment(value)


def world_jacobian(value, yaw, height, image_size):
    if value is None:
        return None
    validate_alignment(value)
    # Do not infer altitude scale or camera geometry from an unmeasured height.
    if (not finite(yaw) or not finite(height)
            or abs(height-value['reference_height_m']) > .02
            or image_size is None or list(image_size) != value['image_size']):
        return None
    c, s = math.cos(yaw), math.sin(yaw)
    # World XY -> aircraft forward/left -> image UV.
    return tuple((a*c-b*s, a*s+b*c) for a, b in value['body_jacobian_px_per_m'])


def fit_marker_samples(samples, reference_height_m, image_size):
    """Stationary disarmed aircraft; move marker, not aircraft, along body axes."""
    if not isinstance(samples, list) or len(samples) != 3:
        raise ValueError('Exactly three independent marker samples required')
    converted = []
    for sample in samples:
        if not isinstance(sample, dict):
            raise ValueError('Invalid marker sample')
        xy, uv = sample.get('marker_body_xy_m'), sample.get('pixel_uv')
        if (not isinstance(xy, list) or not isinstance(uv, list)
                or len(xy) != 2 or len(uv) != 2
                or any(not finite(v) for v in xy+uv)):
            raise ValueError('Marker samples require finite XY and UV pairs')
        if (not isinstance(image_size, list) or len(image_size) != 2
                or any(type(v) is not int or v <= 0 for v in image_size)
                or not all(0 <= uv[i] <= image_size[i] for i in range(2))):
            raise ValueError('Marker sample outside configured image')
        # Moving a fixed-plane marker is opposite to translating the camera.
        converted.append((tuple(-v for v in xy), tuple(uv)))
    matrix = estimate_jacobian(converted)
    value = dict(schema=1, enabled=True, reference_height_m=reference_height_m,
                 image_size=image_size, body_jacobian_px_per_m=[list(row) for row in matrix],
                 method='stationary_aircraft_marker_translation', samples=samples)
    return validate_alignment(value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    data = json.loads(args.samples.read_text(encoding='utf-8'))
    value = fit_marker_samples(data['samples'], data['reference_height_m'], data['image_size'])
    args.output.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Saved offline camera alignment:', args.output)


if __name__ == '__main__':
    main()
