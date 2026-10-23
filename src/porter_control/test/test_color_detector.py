"""Check target selection, bearing convention and detector startup settings."""

import math

import numpy as np
from porter_control.color_detector import detect_orange, DetectorSettings
import pytest


def scene(width=640):
    """Return a blank BGR test image."""
    return np.zeros((480, width, 3), dtype=np.uint8)


@pytest.mark.parametrize('width', [320, 640, 1280])
@pytest.mark.parametrize('fraction', [0.25, 0.5, 0.75])
def test_bearing_is_resolution_independent(width, fraction):
    """Use the pinhole formula and ROS left-positive convention at each size."""
    image = scene(width)
    u = round(width * fraction)
    image[180:261, u - 20:u + 21] = (0, 115, 255)
    detection, _ = detect_orange(image, DetectorSettings())
    expected = math.atan((1 - 2 * fraction) * math.tan(math.radians(40)))
    assert detection is not None
    assert detection.u == pytest.approx(u)
    assert detection.bearing == pytest.approx(expected)
    assert detection.area == pytest.approx(3200.0)


def test_largest_orange_region_wins_over_noise_and_other_colours():
    """Ignore a larger blue object and isolated orange pixels."""
    image = scene()
    image[10:180, 400:600] = (255, 0, 0)
    image[200:241, 80:121] = (0, 115, 255)
    image[200:301, 250:351] = (0, 115, 255)
    image[40, 40] = (0, 115, 255)
    detection, mask = detect_orange(image, DetectorSettings())
    assert detection is not None
    assert (detection.u, detection.v) == pytest.approx((300, 250))
    assert detection.area == pytest.approx(10000)
    assert mask[40, 40] == 0


@pytest.mark.parametrize('colour', [(0, 0, 0), (255, 0, 0), (50, 90, 130)])
def test_background_and_brown_crates_do_not_become_targets(colour):
    """Reject empty frames, blue objects and low-saturation brown crates."""
    image = scene()
    image[100:400, 100:500] = colour
    detection, _ = detect_orange(image, DetectorSettings())
    assert detection is None


def test_dark_orange_vest_is_retained():
    """Keep a highly saturated vest even in the warehouse's shaded view."""
    image = scene()
    image[100:201, 200:301] = (0, 40, 90)
    assert detect_orange(image, DetectorSettings())[0] is not None


def test_small_region_is_ignored_and_exact_area_threshold_is_accepted():
    """No target below the area threshold; equality remains a valid target."""
    image = scene()
    image[100:121, 100:121] = (0, 115, 255)
    assert detect_orange(image, DetectorSettings(min_area=401))[0] is None
    assert detect_orange(image, DetectorSettings(min_area=400))[0] is not None


@pytest.mark.parametrize('settings', [
    {'hsv_lower': (5, 180)},
    {'hsv_upper': (180, 255, 255)},
    {'hsv_lower': (5, -1, 50)},
    {'hsv_lower': (26, 180, 50)},
    {'min_area': -1},
    {'min_area': math.nan},
    {'hfov_deg': 0},
    {'hfov_deg': 180},
    {'hfov_deg': math.inf},
    {'morphology_size': 2},
    {'morphology_size': 23},
])
def test_invalid_settings_fail_at_startup(settings):
    """Reject undefined projections, invalid HSV ranges and kernel sizes."""
    with pytest.raises(ValueError):
        DetectorSettings(**settings)


@pytest.mark.parametrize('image', [
    np.zeros((0, 640, 3), np.uint8),
    np.zeros((480, 640), np.uint8),
    np.zeros((480, 640, 4), np.uint8),
    np.zeros((480, 640, 3), np.float32),
])
def test_invalid_image_layouts_are_rejected(image):
    """Avoid accepting malformed data as a missing or valid target."""
    with pytest.raises(ValueError):
        detect_orange(image, DetectorSettings())
