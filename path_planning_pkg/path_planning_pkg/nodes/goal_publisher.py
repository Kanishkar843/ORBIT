#!/usr/bin/env python3
"""
goal_publisher node
Publishes a single PoseStamped to /goal_pose and optionally to
/navigate_to_pose (as an action). Useful for quick CLI based goal injection.

Usage (after sourcing):
  ros2 run path_planning_pkg goal_publisher --ros-args \
      -p x:=1.5 -p y:=0.5 -p yaw:=90.0
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        self.declare_parameter('x', 1.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)  

        x   = self.get_parameter('x').value
        y   = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value

        pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        yaw_rad = math.radians(yaw)
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)


        def _publish():
            pub.publish(msg)
            self.get_logger().info(
                f'Published goal: x={x:.2f} y={y:.2f} yaw={yaw:.1f}°'
            )

        self.create_timer(0.5, _publish)
        self.create_timer(3.0, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()