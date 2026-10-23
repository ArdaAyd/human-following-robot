"""Exercise the detector's timestamps, RGB input and loss behaviour over ROS."""

import os
import signal
import subprocess
import sys
import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from porter_control.color_detector import ColorDetector
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def test_target_debug_loss_and_recovery():
    """Publish detections only for valid targets and recover after bad input."""
    prefix = f'/detector_test_{os.getpid()}'
    rclpy.init(args=[
        '--ros-args', '-p', 'use_sim_time:=false',
        '-r', f'/camera/image_raw:={prefix}/image',
        '-r', f'/target:={prefix}/target',
        '-r', f'/target_debug_image:={prefix}/debug',
    ])
    detector = ColorDetector()
    probe = rclpy.create_node('detector_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(probe)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    publisher = probe.create_publisher(Image, '/camera/image_raw', qos)
    targets = []
    debug = []
    probe.create_subscription(PointStamped, '/target', targets.append, 10)
    probe.create_subscription(Image, '/target_debug_image', debug.append, qos)
    bridge = CvBridge()

    def pump_until(predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError('Expected ROS output was not received')

    def send(rgb, seconds):
        message = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        message.header.stamp.sec = seconds
        message.header.stamp.nanosec = 123456789
        message.header.frame_id = 'camera_optical_link'
        publisher.publish(message)
        pump_until(lambda: debug and debug[-1].header.stamp.sec == seconds)
        return message

    try:
        pump_until(lambda: publisher.get_subscription_count() > 0
                   and detector.count_subscribers(f'{prefix}/target') > 0
                   and detector.count_subscribers(f'{prefix}/debug') > 0)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[100:201, 100:201] = (255, 115, 0)
        source = send(image, 123)
        pump_until(lambda: targets and targets[-1].header.stamp.sec == 123)
        assert targets[-1].header.stamp == source.header.stamp
        assert targets[-1].header.frame_id == 'camera_link'
        assert targets[-1].point.x > 0.2
        assert targets[-1].point.y == 10000.0
        assert targets[-1].point.z == 0.0
        assert debug[-1].header == source.header
        assert (debug[-1].width, debug[-1].height, debug[-1].encoding) == (640, 480, 'rgb8')
        assert np.any(bridge.imgmsg_to_cv2(debug[-1], 'rgb8'))

        send(np.zeros_like(image), 124)
        tiny = np.zeros_like(image)
        tiny[100:111, 100:111] = (255, 115, 0)
        send(tiny, 125)
        assert not any(t.header.stamp.sec in (124, 125) for t in targets)

        malformed = Image()
        malformed.header.stamp.sec = 126
        malformed.encoding = 'invalid_encoding'
        publisher.publish(malformed)
        # Keep spinning past delivery, then prove that a good frame recovers.
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        truncated = Image()
        truncated.header.stamp.sec = 126
        truncated.header.stamp.nanosec = 1
        truncated.encoding = 'rgb8'
        truncated.width = 640
        truncated.height = 480
        truncated.step = 1920
        publisher.publish(truncated)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        centred = np.zeros_like(image)
        centred[100:201, 300:341] = (255, 115, 0)
        source = send(centred, 127)
        pump_until(lambda: targets[-1].header.stamp.sec == 127)
        assert targets[-1].point.x == 0.0
        assert targets[-1].header.stamp == source.header.stamp
        assert not any(t.header.stamp.sec == 126 for t in targets)
    finally:
        executor.shutdown()
        detector.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()


def test_idle_detector_exits_on_repeated_interrupt(tmp_path):
    """Allow Ctrl+C to close the node even before any camera data arrives."""
    log_path = tmp_path / 'shutdown.log'
    environment = dict(os.environ, ROS_LOG_DIR=str(tmp_path / 'ros-logs'))
    command = [
        sys.executable, '-c',
        'from porter_control.color_detector import main; main()',
        '--ros-args', '-r', f'/camera/image_raw:=/idle_detector_{os.getpid()}/image',
    ]
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=environment)
        try:
            deadline = time.monotonic() + 8.0
            while 'Detecting orange vest' not in log_path.read_text():
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                time.sleep(0.02)
            time.sleep(0.1)
            process.send_signal(signal.SIGINT)
            time.sleep(0.002)
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            assert process.wait(timeout=5.0) == 0, log_path.read_text()
            assert 'Traceback' not in log_path.read_text()
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=3.0)
