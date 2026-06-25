#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute, FollowPath
from std_msgs.msg import Float32

class RouteBridge(Node):
    def __init__(self):
        super().__init__('route_bridge')
        self._route_client = ActionClient(self, ComputeAndTrackRoute, 'compute_and_track_route')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')
        self._following = False
        self._current_edge = None  # track by edge, not a set

        # Subscribe to speed limit from route operations
        self._speed_sub = self.create_subscription(
            Float32, '/speed_limit', self._speed_callback, 10)
        self._current_speed_limit = 0.0  # 0.0 means no limit

    def _speed_callback(self, msg):
        self._current_speed_limit = msg.data
        self.get_logger().info(f'Speed limit updated: {msg.data}')

    def run(self, goal_id):
        self.get_logger().info(f'Starting route to node {goal_id}')
        self._route_client.wait_for_server()

        goal = ComputeAndTrackRoute.Goal()
        goal.goal_id = goal_id
        goal.use_start = False
        goal.use_poses = False

        send_future = self._route_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Route goal rejected')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Route complete')

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        edge_id = fb.current_edge_id

        # Only send new path when we move to a NEW edge
        if (fb.path and len(fb.path.poses) > 0
                and not self._following
                and edge_id != self._current_edge):  # ← fixed: compare to current, not a set

            self._current_edge = edge_id
            self.get_logger().info(
                f'New edge {edge_id}, {len(fb.path.poses)} poses, '
                f'speed_limit={self._current_speed_limit}')
            self._following = True
            self._send_to_controller(fb.path)

    def _send_to_controller(self, path):
        self._follow_client.wait_for_server()
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'

        future = self._follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('follow_path rejected')
            self._following = False
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Edge followed successfully')
        self._following = False

def main():
    rclpy.init()
    node = RouteBridge()
    node.run(goal_id=6)
    rclpy.shutdown()

if __name__ == '__main__':
    main()