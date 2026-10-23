"""Check the demo's timing contract and invalid startup settings."""

import math

from porter_control.person_walker import build_phases, DEFAULTS, phase_at
import pytest


@pytest.mark.parametrize('seconds, command', [
    (0.0, (0.0, 0.0, 0.0)),
    (1.999999999, (0.0, 0.0, 0.0)),
    (2.0, (0.3, 0.0, 0.0)),
    (13.999999999, (0.3, 0.0, 0.0)),
    (14.0, (0.0, 0.0, math.pi / 6.0)),
    (17.0, (0.3, 0.0, 0.0)),
    (27.0, (0.0, 0.0, math.pi / 6.0)),
    (30.0, (0.3, 0.0, 0.0)),
    (52.0, (0.0, 0.0, math.pi / 6.0)),
    (55.0, (0.3, 0.0, 0.0)),
    (69.0, (0.0, 0.3, 0.0)),
    (72.0, (0.0, 0.0, 0.0)),
    (1000.0, (0.0, 0.0, 0.0)),
])
def test_default_timing(seconds, command):
    """Switch exactly at boundaries and never repeat the finished sequence."""
    phase = phase_at(round(seconds * 1_000_000_000), build_phases(DEFAULTS))
    assert (phase.linear_x, phase.linear_y, phase.angular_z) == command


def test_zero_duration_phases_are_skipped():
    """Allow a shortened scenario without briefly issuing skipped commands."""
    settings = dict(DEFAULTS, start_delay=0.0, straight_duration=0.0,
                    turn_duration=0.0, approach_duration=0.0,
                    cross_duration=0.0, ramp_duration=0.0)
    phases = build_phases(settings)
    assert phase_at(0, phases).linear_y == 0.3
    assert phase_at(3_000_000_000, phases).name == 'stopped'


@pytest.mark.parametrize('name, value', [
    ('publish_rate', 0.0),
    ('publish_rate', -1.0),
    ('straight_duration', -0.1),
    ('start_delay', math.nan),
    ('turn_speed', math.inf),
    ('sidestep_speed', -0.3),
])
def test_invalid_settings(name, value):
    """Reject settings that would cause undefined timing or commands."""
    with pytest.raises(ValueError, match=name):
        build_phases(dict(DEFAULTS, **{name: value}))
