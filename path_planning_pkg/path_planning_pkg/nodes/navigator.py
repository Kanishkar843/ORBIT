#!/usr/bin/env python3
"""
Navigator Node
Sends a sequence of Nav2 NavigateToPose goals and prints the outcome.
Goals can be supplied via the 'nav_goals' parameter or defined in the
DEFAULT_GOALS list below.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


DEFAULT_GOALS: list[tuple[float, float, float]] = [
    (1.0,  0.0,   0.0),
    (1.0,  1.5,  90.0),
    (0.0,  1.5, 180.0),
    (0.0,  0.0, -90.0),
]


def yaw_to_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """Convert yaw angle (degrees) → (x, y, z, w) quaternion."""
    yaw = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Navigator(Node):
    """Sends NavigateToPose goals one at a time."""

    def __init__(self):
        super().__init__('navigator')

        self.declare_parameter('goals', '')  

        cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=cb_group,
        )

        goals_param = self.get_parameter('goals').get_parameter_value().string_value
        self._goals = self._parse_goals(goals_param) or DEFAULT_GOALS
        self._goal_index = 0

        
        self.create_timer(2.0, self._start)

    # ── Helpers ────────────────────────────────────────────────────────────

    def _parse_goals(self, raw: str) -> list[tuple[float, float, float]]:
        if not raw:
            return []
        goals = []
        for entry in raw.split(';'):
            parts = entry.strip().split(',')
            if len(parts) == 3:
                try:
                    goals.append((float(parts[0]), float(parts[1]), float(parts[2])))
                except ValueError:
                    pass
        return goals

    def _build_goal_msg(self, x: float, y: float, yaw_deg: float) -> NavigateToPose.Goal:
        qx, qy, qz, qw = yaw_to_quaternion(yaw_deg)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    # ── Goal sequence ──────────────────────────────────────────────────────

    def _start(self):
        """Called once after startup delay; triggers the first goal."""
        self.destroy_timer(self._timer_list[0])  # cancel startup timer

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('NavigateToPose action server not available — exiting.')
            return

        self.get_logger().info(f'Navigator ready. {len(self._goals)} goal(s) queued.')
        self._send_next_goal()

    def _send_next_goal(self):
        if self._goal_index >= len(self._goals):
            self.get_logger().info('All goals completed!')
            return

        x, y, yaw = self._goals[self._goal_index]
        self.get_logger().info(
            f'Sending goal {self._goal_index + 1}/{len(self._goals)}: '
            f'({x:.2f}, {y:.2f}, {yaw:.1f}°)'
        )

        goal_msg = self._build_goal_msg(x, y, yaw)
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            return
        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        status = result.status
        from action_msgs.msg import GoalStatus
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Goal {self._goal_index + 1} reached successfully.'
            )
        else:
            self.get_logger().warn(
                f'Goal {self._goal_index + 1} finished with status {status}.'
            )
        self._goal_index += 1
        self._send_next_goal()

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.get_logger().debug(f'Distance remaining: {dist:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()