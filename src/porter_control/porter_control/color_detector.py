"""
Detect the orange vest and publish its horizontal bearing, positive to the left.

The largest sufficiently large HSV contour becomes /target (PointStamped):
point.x is bearing in radians, point.y is contour area in pixels, point.z is
unused. This is a bearing/area protocol, not a Cartesian position. The header
keeps the source image timestamp and names camera_link, whose +X is forward.
No target is published on a valid image without a detection. A masked RGB image
is published on /target_debug_image for every valid input frame.

Owner: Ahmet Melih ÜSTÜNER
"""

from dataclasses import dataclass
import math
import signal

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class DetectorSettings:
    """Immutable HSV, contour and camera settings, validated at startup."""

    hsv_lower: tuple[int, int, int] = (5, 180, 50)
    hsv_upper: tuple[int, int, int] = (25, 255, 255)
    min_area: float = 500.0
    hfov_deg: float = 80.0
    morphology_size: int = 3

    def __post_init__(self):
        """Reject invalid OpenCV ranges and undefined projection settings."""
        for name in ('hsv_lower', 'hsv_upper'):
            values = getattr(self, name)
            if len(values) != 3 or any(type(v) is not int for v in values):
                raise ValueError(f'{name} must contain three integers')
            if any(not 0 <= v <= limit for v, limit in zip(values, (179, 255, 255))):
                raise ValueError(f'{name} must use H=0..179 and S/V=0..255')
        if any(lo > hi for lo, hi in zip(self.hsv_lower, self.hsv_upper)):
            raise ValueError('hsv_lower must not exceed hsv_upper')
        if not math.isfinite(self.min_area) or self.min_area < 0:
            raise ValueError('min_area must be finite and non-negative')
        if not math.isfinite(self.hfov_deg) or not 0 < self.hfov_deg < 180:
            raise ValueError('hfov_deg must be finite and between 0 and 180')
        if (type(self.morphology_size) is not int or self.morphology_size < 0
                or self.morphology_size > 21
                or (self.morphology_size > 0 and self.morphology_size % 2 == 0)):
            raise ValueError('morphology_size must be 0 or an odd integer up to 21')


@dataclass(frozen=True)
class Detection:
    """Largest accepted contour's centroid, area and horizontal bearing."""

    u: float
    v: float
    area: float
    bearing: float


def detect_orange(image_bgr: np.ndarray, settings: DetectorSettings
                  ) -> tuple[Detection | None, np.ndarray]:
    """Threshold an 8-bit BGR image and project its largest orange contour."""
    if (image_bgr.dtype != np.uint8 or image_bgr.ndim != 3
            or image_bgr.shape[2] != 3 or min(image_bgr.shape[:2]) == 0):
        raise ValueError('Expected a non-empty uint8 BGR image with three channels')
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(settings.hsv_lower, dtype=np.uint8),
                       np.array(settings.hsv_upper, dtype=np.uint8))
    if settings.morphology_size > 1:
        kernel = np.ones((settings.morphology_size, settings.morphology_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    if area < settings.min_area or moments['m00'] <= 0:
        return None, mask
    u = moments['m10'] / moments['m00']
    v = moments['m01'] / moments['m00']
    centre = image_bgr.shape[1] / 2.0
    focal = centre / math.tan(math.radians(settings.hfov_deg) / 2.0)
    return Detection(u, v, area, math.atan((centre - u) / focal)), mask


def debug_image(mask: np.ndarray, detection: Detection | None) -> np.ndarray:
    """Render the mask, image centre and accepted centroid as an RGB image."""
    image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    cv2.line(image, (mask.shape[1] // 2, 0),
             (mask.shape[1] // 2, mask.shape[0] - 1), (255, 80, 80), 1)
    if detection is None:
        label = 'NO TARGET'
    else:
        cv2.drawMarker(image, (round(detection.u), round(detection.v)),
                       (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        label = f'{detection.bearing:+.3f} rad  {detection.area:.0f} px'
    cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 0), 1, cv2.LINE_AA)
    return image


class ColorDetector(Node):
    """Convert camera images into the agreed bearing/area target protocol."""

    def __init__(self):
        """Read startup settings and create camera and output topics."""
        super().__init__('color_detector')
        defaults = DetectorSettings()

        def parameter(name, value):
            return self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value

        self._settings = DetectorSettings(
            hsv_lower=tuple(parameter('hsv_lower', list(defaults.hsv_lower))),
            hsv_upper=tuple(parameter('hsv_upper', list(defaults.hsv_upper))),
            min_area=parameter('min_area', defaults.min_area),
            hfov_deg=parameter('hfov_deg', defaults.hfov_deg),
            morphology_size=parameter('morphology_size', defaults.morphology_size),
        )
        self._bearing_frame = parameter('bearing_frame', 'camera_link')
        if not self._bearing_frame:
            raise ValueError('bearing_frame must not be empty')
        self._bridge = CvBridge()
        # Keep the newest frame, with reliable delivery of the large RGB image.
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._target = self.create_publisher(PointStamped, '/target', 10)
        self._debug = self.create_publisher(Image, '/target_debug_image', image_qos)
        self._camera = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, image_qos)
        self.get_logger().info('Detecting orange vest on /camera/image_raw')

    def _on_image(self, message: Image) -> None:
        """Publish a fresh detection, or only a debug mask when none is found."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            detection, mask = detect_orange(bgr, self._settings)
        except (TypeError, ValueError, cv2.error) as error:
            self.get_logger().warning(f'Skipping invalid image: {error}',
                                      throttle_duration_sec=5.0)
            return
        if detection is not None:
            target = PointStamped()
            target.header.stamp = message.header.stamp
            target.header.frame_id = self._bearing_frame
            target.point.x = detection.bearing
            target.point.y = detection.area
            self._target.publish(target)
        debug = self._bridge.cv2_to_imgmsg(debug_image(mask, detection), encoding='rgb8')
        debug.header = message.header
        self._debug.publish(debug)


def _interrupt(signum, frame):
    """Prevent repeated wrapper signals from interrupting ROS cleanup."""
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None):
    """Run the detector until interrupted and release its ROS resources."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_handlers = {
        sig: signal.signal(sig, _interrupt)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = ColorDetector()
        while rclpy.ok():
            # Let Python handle shutdown even when no camera frames arrive.
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
