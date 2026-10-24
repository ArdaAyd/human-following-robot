"""
Check every branch of the following rule, its boundaries and its scan helpers.

decide() takes no ROS state, so each case below is a plain function call: the
simulator is not needed to prove which command the rule produces.
"""

from dataclasses import fields, replace
import math
from pathlib import Path

from porter_control.follow_controller import (
    decide, FollowSettings, Observation, ScanView,
)
import pytest
from sensor_msgs.msg import LaserScan
import yaml  # type: ignore[import-untyped]

SETTINGS = FollowSettings()
CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'follow_controller.yaml'
RAYS = 181


def following(**changes):
    """Return an observation of a freshly seen target, overridden by keyword."""
    defaults = {
        'age': 0.1, 'bearing': 0.0, 'target_range': 2.0, 'closest_ahead': 2.0,
        'pitch': 0.0, 'yaw_rate': 0.0, 'last_seen_side': 1.0,
    }
    return Observation(**{**defaults, **changes})


def scan(ranges, increment=math.radians(1.0)):
    """Return a ScanView spanning +-90 degrees over explicit returns."""
    return ScanView(tuple(ranges), -math.pi / 2, increment, 0.1, 10.0)


# --------------------------------------------------------------------- turning

@pytest.mark.parametrize('bearing, expected', [
    (0.0, 0.0), (0.1, 0.15), (-0.1, -0.15), (0.2, 0.3), (-0.2, -0.3),
    (0.9, 1.0), (-0.9, -1.0),          # kp_angle * bearing exceeds omega_max
])
def test_turn_rate_is_proportional_to_bearing_and_clamped(bearing, expected):
    """Steer towards the person, left positive, never above omega_max."""
    _, omega = decide(following(bearing=bearing), SETTINGS)
    assert omega == pytest.approx(expected)


@pytest.mark.parametrize('yaw_rate, expected', [
    (0.0, 0.45), (1.0, 0.35), (-1.0, 0.55), (3.0, 0.15),
])
def test_measured_yaw_rate_damps_the_turn(yaw_rate, expected):
    """Subtract kd_angle times the gyroscope reading from the turn command."""
    _, omega = decide(following(bearing=0.3, yaw_rate=yaw_rate), SETTINGS)
    assert omega == pytest.approx(expected)


def test_turning_continues_while_forward_motion_is_gated():
    """Keep steering at full rate even when the bearing gate stops driving."""
    linear, omega = decide(following(bearing=0.9), SETTINGS)
    assert linear == 0.0
    assert omega == pytest.approx(SETTINGS.omega_max)


# --------------------------------------------------------------- forward speed

@pytest.mark.parametrize('target_range, expected', [
    (0.7, 0.0), (0.95, 0.0), (1.0, 0.0),        # at or inside the set point
    (1.05, 0.04), (1.2, 0.16), (1.5, 0.4),      # proportional band
    (1.625, 0.5), (2.0, 0.5), (5.0, 0.5),       # clamped to v_max
])
def test_forward_speed_closes_the_range_error_without_reversing(target_range, expected):
    """Drive at kp_distance times the range error, clipped to [0, v_max]."""
    linear, _ = decide(following(target_range=target_range), SETTINGS)
    assert linear == pytest.approx(expected)


def test_target_without_a_range_measurement_only_turns():
    """Turn towards a target the camera sees but the LiDAR cannot measure."""
    linear, omega = decide(following(bearing=0.2, target_range=None), SETTINGS)
    assert linear == 0.0
    assert omega == pytest.approx(0.3)


@pytest.mark.parametrize('pitch', [0.2, -0.2, math.radians(8.0), -math.radians(8.0)])
def test_ramp_halves_the_speed_ceiling_whatever_the_slope_sign(pitch):
    """Cap the speed at half v_max on a slope, up or down."""
    linear, _ = decide(following(target_range=5.0, pitch=pitch), SETTINGS)
    assert linear == pytest.approx(SETTINGS.v_max / 2)


@pytest.mark.parametrize('pitch, expected', [
    (0.0, 0.5), (0.086, 0.5), (0.087, 0.5), (0.0871, 0.25),
])
def test_pitch_threshold_is_exclusive(pitch, expected):
    """Halve the ceiling strictly above pitch_threshold, not at it."""
    linear, _ = decide(following(target_range=5.0, pitch=pitch), SETTINGS)
    assert linear == pytest.approx(expected)


@pytest.mark.parametrize('bearing, drives', [
    (0.0, True), (0.34, True), (0.35, True), (0.36, False), (0.9, False),
])
def test_robot_turns_in_place_above_the_angle_threshold(bearing, drives):
    """Hold the speed at zero until the bearing is inside angle_threshold."""
    linear, _ = decide(following(bearing=bearing), SETTINGS)
    assert (linear > 0.0) is drives


@pytest.mark.parametrize('closest_ahead, drives', [
    (None, True), (2.0, True), (0.51, True), (0.5, True), (0.49, False), (0.2, False),
])
def test_safety_stop_engages_strictly_below_the_safety_distance(closest_ahead, drives):
    """Stop driving only when something ahead is closer than safety_distance."""
    linear, omega = decide(following(bearing=0.2, closest_ahead=closest_ahead), SETTINGS)
    assert (linear > 0.0) is drives
    assert omega == pytest.approx(0.3)


# -------------------------------------------------------- staleness and search

@pytest.mark.parametrize('age', [0.0, 0.1, 0.5])
def test_sighting_counts_as_fresh_up_to_and_including_the_timeout(age):
    """Keep following while the sighting is no older than target_timeout."""
    linear, _ = decide(following(age=age), SETTINGS)
    assert linear == pytest.approx(SETTINGS.v_max)


@pytest.mark.parametrize('side', [1.0, -1.0])
@pytest.mark.parametrize('age', [0.51, 1.0, 4.99])
def test_stale_target_sweeps_towards_the_side_it_was_last_seen(age, side):
    """Turn in place towards the last sighting instead of driving blind."""
    linear, omega = decide(following(age=age, last_seen_side=side), SETTINGS)
    assert linear == 0.0
    assert omega == pytest.approx(side * SETTINGS.search_omega)


@pytest.mark.parametrize('age', [5.0, 7.0, 1e6])
def test_search_gives_up_once_search_duration_has_passed(age):
    """Come to a complete stop rather than sweeping for ever."""
    assert decide(following(age=age), SETTINGS) == (0.0, 0.0)


def test_target_never_seen_keeps_the_robot_still():
    """Publish a zero command before the first detection ever arrives."""
    assert decide(following(age=None), SETTINGS) == (0.0, 0.0)


# ----------------------------------------------------------------- scan helper

def test_median_bridges_a_gap_between_the_legs():
    """Use a neighbour's range when the centre rays fall into a gap."""
    ranges = [2.0] * RAYS
    for index in (89, 90, 91):
        ranges[index] = math.inf
    assert scan(ranges).median_at(0.0) == pytest.approx(2.0)


def test_median_rejects_a_single_noisy_ray():
    """Prefer the median of the neighbourhood over one spurious return."""
    ranges = [2.0] * RAYS
    ranges[90] = 0.4
    assert scan(ranges).median_at(0.0) == pytest.approx(2.0)


@pytest.mark.parametrize('ranges', [
    [math.inf] * RAYS,
    [math.nan] * RAYS,
    [0.05] * RAYS,          # below range_min
    [12.0] * RAYS,          # above range_max
])
def test_median_is_none_when_nothing_valid_is_measured(ranges):
    """Report no measurement rather than passing on an invalid return."""
    assert scan(ranges).median_at(0.0) is None


@pytest.mark.parametrize('bearing', [-math.pi, -1.66, 1.66, math.pi])
def test_median_is_none_for_a_bearing_outside_the_scan(bearing):
    """Refuse bearings off the end of the arc instead of wrapping the window."""
    assert scan([2.0] * RAYS).median_at(bearing) is None


def test_median_is_none_for_a_degenerate_scan():
    """Avoid dividing by a zero angle increment."""
    assert scan([2.0] * RAYS, increment=0.0).median_at(0.0) is None


def test_median_reads_the_ray_nearest_the_bearing():
    """Round the requested bearing to the closest ray index."""
    ranges = [2.0] * RAYS
    ranges[135] = 7.0          # +45 degrees
    view = scan(ranges)
    assert view.median_at(math.radians(45.0), neighbours=0) == pytest.approx(7.0)
    assert view.median_at(0.0, neighbours=0) == pytest.approx(2.0)


def test_closest_in_cone_ignores_obstacles_outside_the_cone():
    """Watch only the forward safety cone, not the whole scan."""
    ranges = [2.0] * RAYS
    ranges[0] = 0.3            # -90 degrees, well outside the cone
    assert scan(ranges).closest_in_cone() == pytest.approx(2.0)
    ranges[90] = 0.3           # straight ahead
    assert scan(ranges).closest_in_cone() == pytest.approx(0.3)


def test_closest_in_cone_is_none_when_no_return_is_valid():
    """Report an empty cone rather than a false clearance."""
    assert scan([math.inf] * RAYS).closest_in_cone() is None


def test_from_message_copies_only_the_fields_the_rule_needs():
    """Convert a LaserScan into a plain immutable view."""
    message = LaserScan()
    message.angle_min = -1.0
    message.angle_increment = 0.01
    message.range_min = 0.2
    message.range_max = 8.0
    message.ranges = [1.0, 2.0, 3.0]
    view = ScanView.from_message(message)
    assert view.ranges == (1.0, 2.0, 3.0)
    assert view.angle_min == pytest.approx(-1.0)
    assert view.angle_increment == pytest.approx(0.01)
    assert view.range_min == pytest.approx(0.2)
    assert view.range_max == pytest.approx(8.0)


# --------------------------------------------------------------------- settings

@pytest.mark.parametrize('changes', [
    {'target_distance': 0.0}, {'target_distance': -1.0},
    {'v_max': 0.0}, {'omega_max': -0.1},
    {'angle_threshold': 0.0}, {'angle_threshold': 4.0},
    {'pitch_threshold': 0.0}, {'pitch_threshold': math.pi / 2},
    {'target_timeout': 0.0},
    {'kp_angle': -0.1}, {'kd_angle': -0.1}, {'kp_distance': -0.1},
    {'safety_distance': -0.1}, {'search_omega': -0.1},
    {'search_duration': 0.5},      # equal to target_timeout: sweep unreachable
    {'search_duration': 0.2},
    {'v_max': math.nan}, {'kp_angle': math.inf},
    {'target_distance': True}, {'v_max': 'fast'},
])
def test_invalid_settings_fail_at_startup(changes):
    """Reject gains that would disable, freeze or undefine the control law."""
    with pytest.raises(ValueError):
        replace(SETTINGS, **changes)


def test_shipped_configuration_is_complete_and_valid():
    """Catch a gain that exists in the code but was never written to the yaml."""
    document = yaml.safe_load(CONFIG.read_text())
    parameters = dict(document['follow_controller']['ros__parameters'])
    # Every node in this package runs on the Gazebo clock.
    assert parameters.pop('use_sim_time') is True
    assert set(parameters) == {field.name for field in fields(FollowSettings)}
    FollowSettings(**parameters)
