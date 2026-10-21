"""Drive the followed person model through a fixed demo scenario.

Publishes
---------
/person/cmd_vel : geometry_msgs/Twist
    Consumed by the VelocityControl plugin attached to the person model.

Scenario: walk straight -> turn -> climb the ramp -> sudden sidestep (to test
recovery when the target leaves the camera frame) -> stop.

The scenario is identical on every run so that the live demo, the backup video
and the plots in the report all come from the same motion.

Owner: Ahmet Melih ÜSTÜNER
"""

import rclpy
from rclpy.node import Node


class PersonWalker(Node):

    def __init__(self):
        super().__init__('person_walker')
        self.get_logger().warning('person_walker is not implemented yet')


def main(args=None):
    rclpy.init(args=args)
    node = PersonWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
