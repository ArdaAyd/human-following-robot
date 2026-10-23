"""Exercise clock handling and stop delivery over actual ROS topics."""

import math
import os
import time

from geometry_msgs.msg import Twist
from porter_control.person_walker import PersonWalker
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rosgraph_msgs.msg import Clock


def test_clock_readiness_pause_rewind_and_stop():
    """Wait for readiness, follow simulated time, and stop before teardown."""
    test_namespace = f'/person_walker_test_{os.getpid()}'
    rclpy.init(args=[
        '--ros-args', '-p', 'use_sim_time:=true',
        '-r', f'/clock:={test_namespace}/clock',
        '-r', f'/person/cmd_vel:={test_namespace}/cmd_vel',
    ])
    walker = PersonWalker()
    probe = rclpy.create_node('walker_clock_test')
    clock_publisher = probe.create_publisher(Clock, '/clock', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(walker)
    executor.add_node(probe)
    received = []

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    def set_clock(seconds):
        nanoseconds = round(seconds * 1_000_000_000)
        message = Clock()
        message.clock.sec = nanoseconds // 1_000_000_000
        message.clock.nanosec = nanoseconds % 1_000_000_000
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            clock_publisher.publish(message)
            pump(0.05)
            if walker.get_clock().now().nanoseconds == nanoseconds:
                return
        raise AssertionError('The node did not receive the test clock')

    def expect(seconds, expected):
        set_clock(seconds)
        received.clear()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            pump(0.05)
            if received and all(abs(a - b) < 1e-6
                                for a, b in zip(received[-1], expected)):
                return
        raise AssertionError(f'Expected {expected}, received {received[-3:]}')

    def collect(message):
        received.append((message.linear.x, message.linear.y, message.angular.z))

    try:
        # Forty simulated seconds pass with no command receiver. The scenario
        # must still start with its full delay once a receiver appears.
        set_clock(10.0)
        pump(0.15)
        set_clock(50.0)
        pump(0.15)
        probe.create_subscription(Twist, '/person/cmd_vel', collect, 10)
        expect(50.0, (0.0, 0.0, 0.0))
        expect(52.0, (0.3, 0.0, 0.0))
        expect(59.95, (0.3, 0.0, 0.0))
        received.clear()
        pump(0.3)
        assert received and all(command == (0.3, 0.0, 0.0) for command in received)
        expect(60.0, (0.0, 0.0, math.pi / 6.0))
        expect(63.0, (0.3, 0.0, 0.0))
        expect(73.0, (0.0, 0.3, 0.0))
        expect(76.0, (0.0, 0.0, 0.0))
        expect(90.0, (0.0, 0.0, 0.0))
        expect(10.0, (0.0, 0.0, 0.0))
        expect(12.0, (0.3, 0.0, 0.0))
        received.clear()
        walker.stop()
        pump(0.2)
        assert received and received[-1] == (0.0, 0.0, 0.0)
    finally:
        walker.stop()
        executor.shutdown()
        walker.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()
