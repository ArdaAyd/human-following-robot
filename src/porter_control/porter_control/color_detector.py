"""Detect the orange safety vest in the camera image and publish its bearing.

Subscribes
----------
/camera/image_raw : sensor_msgs/Image
    Raw RGB frame from the mast camera.

Publishes
---------
/target : geometry_msgs/PointStamped
    point.x -- bearing of the target in radians, positive to the left
    point.y -- area of the detected blob in pixels
    Nothing is published when no blob passes the area threshold.
/target_debug_image : sensor_msgs/Image
    Colour mask with the detected centroid marked, for visual inspection.

The bearing is derived from the horizontal centroid u of the largest contour:

    f     = (W / 2) / tan(HFOV / 2)
    theta = atan((W / 2 - u) / f)

Owner: TEAMMATE
"""

import rclpy
from rclpy.node import Node


class ColorDetector(Node):

    def __init__(self):
        super().__init__('color_detector')
        self.get_logger().warning('color_detector is not implemented yet')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
