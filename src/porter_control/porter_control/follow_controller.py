"""Combine target bearing, LiDAR range and IMU tilt into a drive command.

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

Owner: Arda AYDIN
"""

import math

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

CONTROL_RATE = 20.0          # Hz
SAFETY_HALF_ANGLE = math.radians(30.0)
RANGE_NEIGHBOURS = 3         # rays either side of the target bearing


class FollowController(Node):

    def __init__(self):
        super().__init__('follow_controller')

        self.declare_parameters('', [
            ('target_distance', 1.0),
            ('kp_angle', 1.5),
            ('kd_angle', 0.1),
            ('kp_distance', 0.8),
            ('v_max', 0.5),
            ('omega_max', 1.0),
            ('angle_threshold', 0.35),
            ('safety_distance', 0.5),
            ('pitch_threshold', 0.087),
            ('target_timeout', 0.5),
            ('search_duration', 5.0),
            ('search_omega', 0.4),
        ])

        self.create_subscription(PointStamped, '/target', self._on_target, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Imu, '/imu', self._on_imu, 20)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Latest input, written by the callbacks and read by the timer.
        self._bearing = None
        self._target_time = None
        self._last_seen_side = 1.0     # +1 left, -1 right
        self._scan = None
        self._pitch = 0.0
        self._yaw_rate = 0.0

        self.create_timer(1.0 / CONTROL_RATE, self._tick)
        self.get_logger().info('follow_controller ready')

    # ------------------------------------------------------------------ inputs

    def _on_target(self, msg):
        self._bearing = msg.point.x
        self._target_time = self.get_clock().now()
        if abs(msg.point.x) > 1e-3:
            self._last_seen_side = math.copysign(1.0, msg.point.x)

    def _on_scan(self, msg):
        self._scan = msg

    def _on_imu(self, msg):
        q = msg.orientation
        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        self._pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        self._yaw_rate = msg.angular_velocity.z

    # ------------------------------------------------------------------ helpers

    def _p(self, name):
        return self.get_parameter(name).value

    def _age(self):
        """Seconds since the last /target message, or None if there was none."""
        if self._target_time is None:
            return None
        return (self.get_clock().now() - self._target_time).nanoseconds * 1e-9

    def _range_at(self, bearing):
        """Median range around one bearing, or None when nothing is measured.

        A single ray can fall into a gap between the person's legs or land on
        a noisy edge, so the median of a few neighbours is used instead.
        """
        scan = self._scan
        if scan is None:
            return None
        index = round((bearing - scan.angle_min) / scan.angle_increment)
        lo = max(0, index - RANGE_NEIGHBOURS)
        hi = min(len(scan.ranges), index + RANGE_NEIGHBOURS + 1)
        window = [r for r in scan.ranges[lo:hi]
                  if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        if not window:
            return None
        window.sort()
        return window[len(window) // 2]

    def _closest_ahead(self):
        """Nearest valid return inside the forward safety cone."""
        scan = self._scan
        if scan is None:
            return None
        nearest = None
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or not (scan.range_min <= r <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) <= SAFETY_HALF_ANGLE and (nearest is None or r < nearest):
                nearest = r
        return nearest

    def _publish(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ control

    def _tick(self):
        age = self._age()

        # 1. No target, or the last sighting has gone stale.
        if age is None or age > self._p('target_timeout'):
            self._search(age)
            return

        bearing = self._bearing

        # 2. Turn towards the target. The derivative term uses the measured
        #    yaw rate rather than a differenced bearing, which would amplify
        #    the detector's pixel noise.
        omega = (self._p('kp_angle') * bearing
                 - self._p('kd_angle') * self._yaw_rate)
        omega = self._clamp(omega, self._p('omega_max'))

        # 3. Forward speed closes the gap to target_distance. On the ramp the
        #    ceiling is halved; the sign of the pitch does not matter.
        v_limit = self._p('v_max')
        if abs(self._pitch) > self._p('pitch_threshold'):
            v_limit *= 0.5

        measured = self._range_at(bearing)
        if measured is None:
            # Seen by the camera but not measured by the LiDAR: turn only.
            self._publish(0.0, omega)
            return

        linear = self._p('kp_distance') * (measured - self._p('target_distance'))
        linear = max(0.0, min(linear, v_limit))

        # 4. Turn before driving, so the robot does not arc into obstacles.
        if abs(bearing) > self._p('angle_threshold'):
            linear = 0.0

        # 5. Safety stop overrides everything except turning in place.
        closest = self._closest_ahead()
        if closest is not None and closest < self._p('safety_distance'):
            linear = 0.0

        self._publish(linear, omega)

    def _search(self, age):
        """Sweep towards the side the target was last seen, then give up."""
        if age is not None and age < self._p('search_duration'):
            self._publish(0.0, self._last_seen_side * self._p('search_omega'))
        else:
            self._publish(0.0, 0.0)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = FollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the robot stopped: the drive plugin holds the last command.
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
