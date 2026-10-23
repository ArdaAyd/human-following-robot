"""
Exercise the controller's topics, clock handling and shutdown over ROS.

decide() is covered without ROS in test_follow_controller.py; what is left to
prove here is the adapter around it: that a stamp from the detector decides
staleness, that an IMU quaternion becomes a speed limit, and that an
interrupted run leaves the robot stopped rather than driving on.
"""

import math
import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PointStamped, Twist
from porter_control.follow_controller import FollowController, FollowSettings
import pytest
import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, LaserScan

SETTINGS = FollowSettings()
RAYS = 181


def _remappings(prefix):
    """Return ros-args that move the node's topics under a private prefix."""
    return [
        '--ros-args', '-p', 'use_sim_time:=false',
        '-r', f'/target:={prefix}/target',
        '-r', f'/scan:={prefix}/scan',
        '-r', f'/imu:={prefix}/imu',
        '-r', f'/cmd_vel:={prefix}/cmd_vel',
    ]


class Harness:
    """A controller, a probe that feeds it, and the commands it produced."""

    def __init__(self):
        """Create the controller and the probe on one executor."""
        self.node = FollowController()
        self.probe = rclpy.create_node('follow_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.probe)
        self.commands = []
        self.probe.create_subscription(
            Twist, '/cmd_vel',
            lambda m: self.commands.append((m.linear.x, m.angular.z)), 10)
        self._target = self.probe.create_publisher(PointStamped, '/target', 10)
        self._scan = self.probe.create_publisher(LaserScan, '/scan', 10)
        self._imu = self.probe.create_publisher(Imu, '/imu', 20)

    def send_target(self, bearing=0.0, age=0.0, stamp_seconds=None):
        """Publish a sighting, aged against this clock or stamped absolutely."""
        message = PointStamped()
        if stamp_seconds is None:
            message.header.stamp = (
                self.probe.get_clock().now() - Duration(seconds=age)).to_msg()
        else:
            message.header.stamp.sec = int(stamp_seconds)
        message.header.frame_id = 'camera_link'
        message.point.x = bearing
        message.point.y = 5000.0
        self._target.publish(message)

    def send_scan(self, distance=3.0):
        """Publish a flat scan at one distance across the whole arc."""
        message = LaserScan()
        message.angle_min, message.angle_max = -math.pi / 2, math.pi / 2
        message.angle_increment = math.radians(1.0)
        message.range_min, message.range_max = 0.1, 10.0
        message.ranges = [distance] * RAYS
        self._scan.publish(message)

    def send_imu(self, pitch=0.0, yaw_rate=0.0):
        """Publish an orientation tilted by `pitch` about the Y axis."""
        message = Imu()
        message.orientation.w = math.cos(pitch / 2.0)
        message.orientation.y = math.sin(pitch / 2.0)
        message.angular_velocity.z = yaw_rate
        self._imu.publish(message)

    def drive(self, seconds, target=True, **inputs):
        """Feed the inputs for a while, optionally withholding the target."""
        target_keys = {'bearing', 'age', 'stamp_seconds'}
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if target:
                self.send_target(**{k: v for k, v in inputs.items()
                                    if k in target_keys})
            self.send_scan(**{k: v for k, v in inputs.items()
                              if k == 'distance'})
            self.send_imu(**{k: v for k, v in inputs.items()
                             if k in ('pitch', 'yaw_rate')})
            self.executor.spin_once(timeout_sec=0.01)

    def last(self):
        """Return the most recent command, failing if none was published."""
        assert self.commands, 'the controller published no command at all'
        return self.commands[-1]

    def close(self):
        """Release the executor and both nodes."""
        self.executor.shutdown()
        self.node.destroy_node()
        self.probe.destroy_node()


@pytest.fixture
def harness():
    """Provide a controller wired to a probe over private topics."""
    prefix = f'/follow_test_{os.getpid()}'
    rclpy.init(args=_remappings(prefix))
    created = Harness()
    try:
        yield created
    finally:
        created.close()
        rclpy.try_shutdown()


def test_follows_a_target_seen_straight_ahead(harness):
    """Drive forward at the speed the range error calls for."""
    harness.drive(1.0, bearing=0.0, distance=3.0)
    linear, angular = harness.last()
    assert linear == pytest.approx(SETTINGS.v_max)
    assert angular == pytest.approx(0.0)


def test_turns_towards_a_target_off_to_one_side(harness):
    """Steer by kp_angle times the bearing while still closing the distance."""
    harness.drive(1.0, bearing=0.2, distance=3.0)
    linear, angular = harness.last()
    assert linear == pytest.approx(SETTINGS.v_max)
    assert angular == pytest.approx(SETTINGS.kp_angle * 0.2)


def test_an_old_camera_stamp_counts_as_stale_on_arrival(harness):
    """
    Measure staleness from the camera frame, not from the time of delivery.

    The detector forwards the image stamp, so a sighting that took two seconds
    to reach the controller must not be treated as a fresh one.
    """
    harness.drive(1.0, bearing=0.0, age=2.0, distance=3.0)
    linear, angular = harness.last()
    assert linear == 0.0
    assert angular == pytest.approx(SETTINGS.search_omega)


def test_a_clock_mismatch_leaves_the_robot_stopped(harness):
    """
    Refuse to act on sightings stamped by a clock this node does not share.

    This harness runs on wall time, so a simulated-clock stamp looks billions
    of seconds old. The robot must stop rather than steer on the bearing.
    """
    harness.drive(1.0, bearing=0.3, stamp_seconds=30, distance=3.0)
    assert harness.last() == (0.0, 0.0)


def test_imu_pitch_halves_the_speed_on_a_slope(harness):
    """Convert the orientation quaternion into the ramp speed limit."""
    harness.drive(1.0, bearing=0.0, distance=3.0, pitch=0.2)
    linear, _ = harness.last()
    assert linear == pytest.approx(SETTINGS.v_max / 2)


@pytest.mark.parametrize('bearing, side', [(0.5, 1.0), (-0.5, -1.0)])
def test_sweep_turns_towards_the_side_the_target_was_last_seen(harness, bearing, side):
    """Remember which way the person went, then sweep that way once lost."""
    harness.drive(0.4, bearing=bearing, distance=3.0)
    turning, steering = harness.last()
    assert turning == 0.0          # bearing is outside angle_threshold
    assert steering == pytest.approx(SETTINGS.kp_angle * bearing)
    harness.drive(1.2, target=False, distance=3.0)
    linear, angular = harness.last()
    assert linear == 0.0
    assert angular == pytest.approx(side * SETTINGS.search_omega)


def test_gains_are_read_only_at_runtime(harness):
    """Reject a live change, so a run cannot be retuned halfway through."""
    assert harness.node.describe_parameter('v_max').read_only
    result, = harness.node.set_parameters(
        [Parameter('v_max', Parameter.Type.DOUBLE, 0.1)])
    assert not result.successful
    assert 'read-only' in result.reason
    assert harness.node.get_parameter('v_max').value == pytest.approx(SETTINGS.v_max)


def test_an_invalid_gain_stops_the_node_from_starting():
    """Fail at startup rather than driving with a broken control law."""
    prefix = f'/follow_bad_{os.getpid()}'
    rclpy.init(args=_remappings(prefix) + ['-p', 'target_distance:=-1.0'])
    try:
        with pytest.raises(ValueError, match='target_distance'):
            FollowController()
    finally:
        rclpy.try_shutdown()


def test_interrupt_leaves_a_zero_command_on_the_wire():
    """
    Stop the robot when the run is interrupted.

    rclpy's default handlers tear the context down before cleanup runs, which
    would leave the drive plugin holding the last non-zero command.
    """
    prefix = f'/follow_sigint_{os.getpid()}'
    rclpy.init(args=_remappings(prefix))
    commands = []
    try:
        probe = rclpy.create_node('sigint_probe')
        probe.create_subscription(
            Twist, '/cmd_vel',
            lambda m: commands.append((m.linear.x, m.angular.z)), 10)
        target = probe.create_publisher(PointStamped, '/target', 10)
        scan = probe.create_publisher(LaserScan, '/scan', 10)
        imu = probe.create_publisher(Imu, '/imu', 20)

        child = subprocess.Popen(
            [sys.executable, '-c',
             'from porter_control.follow_controller import main; main()']
            + _remappings(prefix),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            flat = LaserScan()
            flat.angle_min, flat.angle_max = -math.pi / 2, math.pi / 2
            flat.angle_increment = math.radians(1.0)
            flat.range_min, flat.range_max = 0.1, 10.0
            flat.ranges = [3.0] * RAYS
            level = Imu()
            level.orientation.w = 1.0

            deadline = time.monotonic() + 15.0
            while not any(c[0] != 0.0 for c in commands):
                assert child.poll() is None, 'the controller exited early'
                assert time.monotonic() < deadline, 'no motion command arrived'
                point = PointStamped()
                point.header.stamp = probe.get_clock().now().to_msg()
                point.point.y = 5000.0
                target.publish(point)
                scan.publish(flat)
                imu.publish(level)
                rclpy.spin_once(probe, timeout_sec=0.02)

            child.send_signal(signal.SIGINT)
            assert child.wait(timeout=10.0) == 0
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.02)
            assert commands[-1] == (0.0, 0.0)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait(timeout=5.0)
            probe.destroy_node()
    finally:
        rclpy.try_shutdown()
