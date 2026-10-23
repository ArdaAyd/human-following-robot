"""
Combine target bearing, LiDAR range and IMU tilt into a drive command.

Subscribes
----------
/target : geometry_msgs/PointStamped
    point.x is the bearing to the person in radians, positive to the left;
    point.y is the pixel area of the detected blob. Published by
    color_detector only while it actually sees the vest.
/scan : sensor_msgs/LaserScan
    The range is read along the target bearing, not at the global minimum, so
    that obstacles beside the robot are never mistaken for the person.
/imu : sensor_msgs/Imu
    Angular velocity feeds the derivative term; pitch caps the forward speed
    on the ramp.

Publishes
---------
/cmd_vel : geometry_msgs/Twist

The three inputs arrive at different rates (roughly 30, 10 and 100 Hz), so the
callbacks only store the latest message and a fixed-rate timer does the
deciding. Driving straight from a sensor callback would tie the control rate
to that sensor and leave the robot running on a stale command whenever a
message is late.

The control law itself is the free function decide(), which reads an
Observation and returns a command. It touches no ROS state, so every branch of
the rule can be tested without a simulator; FollowController is only the
adapter that fills an Observation from the topics.

Owner: Arda AYDIN
"""

from collections.abc import Sequence
from dataclasses import dataclass, fields
import math
import signal

from geometry_msgs.msg import PointStamped, Twist
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan

CONTROL_RATE = 20.0          # Hz
SAFETY_HALF_ANGLE = math.radians(30.0)
RANGE_NEIGHBOURS = 3         # rays either side of the target bearing
# A sighting should reach this node within a fraction of a second of
# being taken. A wildly different delay means this node's clock and the
# detector's disagree, which staleness alone would silently read as a
# target that is never fresh.
STAMP_SANITY_DELAY = 5.0     # s

# Gains that must stay positive, and gains that may be switched off with zero.
_POSITIVE = ('target_distance', 'v_max', 'omega_max', 'angle_threshold',
             'pitch_threshold', 'target_timeout')
_NON_NEGATIVE = ('kp_angle', 'kd_angle', 'kp_distance', 'safety_distance',
                 'search_duration', 'search_omega')


@dataclass(frozen=True)
class FollowSettings:
    """Immutable control gains and limits, validated at startup."""

    target_distance: float = 1.0
    kp_angle: float = 1.5
    kd_angle: float = 0.1
    kp_distance: float = 0.8
    v_max: float = 0.5
    omega_max: float = 1.0
    angle_threshold: float = 0.35
    safety_distance: float = 0.5
    pitch_threshold: float = 0.087
    target_timeout: float = 0.5
    search_duration: float = 5.0
    search_omega: float = 0.4

    def __post_init__(self) -> None:
        """Reject values that would disable or undefine the control law."""
        for name in _POSITIVE + _NON_NEGATIVE:
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f'{name} must be a number')
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
        for name in _POSITIVE:
            if getattr(self, name) <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        for name in _NON_NEGATIVE:
            if getattr(self, name) < 0.0:
                raise ValueError(f'{name} must not be negative')
        if self.angle_threshold > math.pi:
            raise ValueError('angle_threshold must not exceed pi')
        if self.pitch_threshold >= math.pi / 2:
            raise ValueError('pitch_threshold must stay below pi/2')
        # Searching only runs while target_timeout < age < search_duration, so
        # the sweep would be unreachable if the window were empty.
        if self.search_duration <= self.target_timeout:
            raise ValueError('search_duration must exceed target_timeout')


@dataclass(frozen=True)
class Observation:
    """Everything the control law knows at one tick."""

    age: float | None            # s since the last target, None if never seen
    bearing: float = 0.0         # rad, positive to the left
    target_range: float | None = None    # m along the bearing
    closest_ahead: float | None = None   # m inside the safety cone
    pitch: float = 0.0           # rad
    yaw_rate: float = 0.0        # rad/s
    last_seen_side: float = 1.0  # +1 left, -1 right


@dataclass(frozen=True)
class ScanView:
    """Read-only view over one LaserScan's numbers, free of ROS types."""

    ranges: Sequence[float]
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float

    @classmethod
    def from_message(cls, message: LaserScan) -> 'ScanView':
        """Keep the scan fields the control law needs and drop the rest."""
        return cls(
            ranges=tuple(message.ranges),
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
        )

    def _valid(self, value: float) -> bool:
        """Report whether one return lies inside the sensor's usable band."""
        return math.isfinite(value) and self.range_min <= value <= self.range_max

    def median_at(self, bearing: float,
                  neighbours: int = RANGE_NEIGHBOURS) -> float | None:
        """
        Median range around one bearing, or None when nothing is measured.

        A single ray can fall into a gap between the person's legs or land on
        a noisy edge, so the median of a few neighbours is used instead.
        """
        if self.angle_increment == 0.0:
            return None
        index = round((bearing - self.angle_min) / self.angle_increment)
        # A bearing outside the scan is unmeasured. Without this guard a
        # negative index would slice from the end of the list and return the
        # median of an unrelated arc.
        if not 0 <= index < len(self.ranges):
            return None
        lo = max(0, index - neighbours)
        hi = min(len(self.ranges), index + neighbours + 1)
        window = sorted(r for r in self.ranges[lo:hi] if self._valid(r))
        if not window:
            return None
        return window[len(window) // 2]

    def closest_in_cone(self,
                        half_angle: float = SAFETY_HALF_ANGLE) -> float | None:
        """Nearest valid return inside the forward cone, or None if empty."""
        nearest = None
        for i, value in enumerate(self.ranges):
            if not self._valid(value):
                continue
            angle = self.angle_min + i * self.angle_increment
            if abs(angle) <= half_angle and (nearest is None or value < nearest):
                nearest = value
        return nearest


def _clamp(value: float, limit: float) -> float:
    """Confine a signed command to +-limit."""
    return max(-limit, min(limit, value))


def decide(observation: Observation,
           settings: FollowSettings) -> tuple[float, float]:
    """
    Return the (linear, angular) command for one tick.

    The rule, in order: a stale target sweeps towards its last side and then
    gives up; otherwise the robot always turns towards the bearing, and only
    drives forward once it is roughly aligned, has a range measurement, and
    sees nothing closer than safety_distance ahead.
    """
    age = observation.age
    if age is None or age > settings.target_timeout:
        if age is not None and age < settings.search_duration:
            return 0.0, observation.last_seen_side * settings.search_omega
        return 0.0, 0.0

    # Turn towards the target. The derivative term uses the measured yaw rate
    # rather than a differenced bearing, which would amplify the detector's
    # pixel noise.
    omega = _clamp(settings.kp_angle * observation.bearing
                   - settings.kd_angle * observation.yaw_rate,
                   settings.omega_max)

    if observation.target_range is None:
        # Seen by the camera but not measured by the LiDAR: turn only.
        return 0.0, omega

    # Forward speed closes the gap to target_distance. On the ramp the ceiling
    # is halved; the sign of the pitch does not matter.
    v_limit = settings.v_max
    if abs(observation.pitch) > settings.pitch_threshold:
        v_limit *= 0.5

    linear = settings.kp_distance * (observation.target_range
                                     - settings.target_distance)
    linear = max(0.0, min(linear, v_limit))

    # Turn before driving, so the robot does not arc into obstacles.
    if abs(observation.bearing) > settings.angle_threshold:
        linear = 0.0

    # Safety stop overrides everything except turning in place.
    if (observation.closest_ahead is not None
            and observation.closest_ahead < settings.safety_distance):
        linear = 0.0

    return linear, omega


class FollowController(Node):
    """Fill an Observation from the sensor topics and publish decide()'s command."""

    def __init__(self) -> None:
        """Read startup gains and create the sensor and command topics."""
        super().__init__('follow_controller')

        defaults = FollowSettings()
        self._settings = FollowSettings(**{
            field.name: self.declare_parameter(
                field.name, getattr(defaults, field.name),
                ParameterDescriptor(read_only=True)).value
            for field in fields(FollowSettings)
        })

        self.create_subscription(PointStamped, '/target', self._on_target, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Latest input, written by the callbacks and read by the timer.
        self._bearing: float | None = None
        self._target_time: Time | None = None
        self._last_seen_side = 1.0     # +1 left, -1 right
        self._scan: ScanView | None = None
        self._pitch = 0.0
        self._yaw_rate = 0.0

        self.create_timer(1.0 / CONTROL_RATE, self._tick)
        self.get_logger().info('follow_controller ready')

    # ------------------------------------------------------------------ inputs

    def _on_target(self, message: PointStamped) -> None:
        """Store the bearing and the stamp of the frame it was measured in."""
        self._bearing = message.point.x
        # The detector forwards the camera frame's stamp, so staleness counts
        # from when the person was actually seen, not from when this arrived.
        self._target_time = Time.from_msg(message.header.stamp)
        delay = (self.get_clock().now() - self._target_time).nanoseconds * 1e-9
        if abs(delay) > STAMP_SANITY_DELAY:
            self.get_logger().warning(
                f'/target is stamped {delay:.1f} s away from this clock; check '
                'that use_sim_time matches color_detector',
                throttle_duration_sec=5.0)
        if abs(message.point.x) > 1e-3:
            self._last_seen_side = math.copysign(1.0, message.point.x)

    def _on_scan(self, message: LaserScan) -> None:
        """Keep the latest scan in the form the control law reads."""
        self._scan = ScanView.from_message(message)

    def _on_imu(self, message: Imu) -> None:
        """Track pitch and yaw rate from the orientation and gyroscope."""
        q = message.orientation
        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        self._yaw_rate = message.angular_velocity.z

    # ------------------------------------------------------------------ helpers

    def _age(self) -> float | None:
        """Seconds since the last /target message, or None if there was none."""
        if self._target_time is None:
            return None
        return (self.get_clock().now() - self._target_time).nanoseconds * 1e-9

    def _observe(self) -> Observation:
        """Assemble the current inputs into one Observation."""
        bearing = 0.0 if self._bearing is None else self._bearing
        scan = self._scan
        return Observation(
            age=self._age(),
            bearing=bearing,
            target_range=None if scan is None else scan.median_at(bearing),
            closest_ahead=None if scan is None else scan.closest_in_cone(),
            pitch=self._pitch,
            yaw_rate=self._yaw_rate,
            last_seen_side=self._last_seen_side,
        )

    def _publish(self, linear: float, angular: float) -> None:
        """Send one velocity command."""
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._cmd_pub.publish(command)

    def stop(self) -> None:
        """Command zero velocity, so shutdown does not leave the robot moving."""
        self._publish(0.0, 0.0)

    # ------------------------------------------------------------------ control

    def _tick(self) -> None:
        """Decide and publish once, at CONTROL_RATE."""
        self._publish(*decide(self._observe(), self._settings))


def _interrupt(signum, frame):
    """Prevent repeated wrapper signals from interrupting ROS cleanup."""
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, signal.SIG_IGN)
    raise KeyboardInterrupt


def main(args=None) -> None:
    """Run the controller until interrupted, leaving the robot stopped."""
    # rclpy's own handlers shut the context down before cleanup runs, which
    # would leave the stop command below unsent and the robot still driving.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_handlers = {
        sig: signal.signal(sig, _interrupt)
        for sig in (signal.SIGINT, signal.SIGTERM)
    }
    node = None
    try:
        node = FollowController()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                # The drive plugin holds the last command until it is replaced.
                if rclpy.ok():
                    node.stop()
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
