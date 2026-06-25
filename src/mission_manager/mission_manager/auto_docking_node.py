import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from opennav_docking_msgs.action import DockRobot   # type: ignore
from opennav_docking_msgs.action import UndockRobot # type: ignore

class AutoDocking(Node):

    def __init__(self):
        super().__init__('auto_docking')
        self.is_currently_docking = False
        self.is_currently_undocking = False
        self.is_docked = False
        self.is_charging = False  # track locally to guard undock trigger

        self._action_client = ActionClient(self, DockRobot, 'dock_robot')
        self._action_client_undock = ActionClient(self, UndockRobot, 'undock_robot')
        self._charging_pub = self.create_publisher(Bool, '/set_charging', 10)

        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_callback,
            10)

    def battery_callback(self, msg):
        # Dock when battery low
        if msg.percentage <= 0.20 and not self.is_currently_docking and not self.is_docked:
            self.get_logger().info(f'Battery at {msg.percentage*100:.1f}%, initiating docking...')
            self.is_currently_docking = True
            self.send_dock_goal()

        # Undock only after charging has started AND battery is full
        elif msg.percentage >= 0.96 and self.is_docked and self.is_charging and not self.is_currently_undocking:
            self.get_logger().info('Battery full, initiating undocking...')
            self.is_currently_undocking = True
            self.send_undock_goal()

    # ── DOCKING ──────────────────────────────────────────────

    def send_dock_goal(self):
        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = 'my_dock'
        goal_msg.navigate_to_staging_pose = True

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.dock_goal_response_callback)

    def dock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Docking goal REJECTED')
            self.is_currently_docking = False
            return
        self.get_logger().info('Docking goal ACCEPTED, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.dock_result_callback)

    def dock_result_callback(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Docking SUCCEEDED — robot is docked!')
            self.is_docked = True
            self.set_charging(True)
        else:
            self.get_logger().error(f'❌ Docking FAILED with status: {status}')
            self.is_docked = False

        self.is_currently_docking = False

    # ── UNDOCKING ─────────────────────────────────────────────

    def send_undock_goal(self):
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = 'aruco_dock'

        self._action_client_undock.wait_for_server()
        future = self._action_client_undock.send_goal_async(goal_msg)
        future.add_done_callback(self.undock_goal_response_callback)

    def undock_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undocking goal REJECTED')
            self.is_currently_undocking = False
            return
        self.get_logger().info('Undocking goal ACCEPTED, waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Undocking SUCCEEDED — robot is free!')
            self.is_docked = False
            self.is_charging = False  # reset local flag
            self.set_charging(False)  # tell battery node to stop charging
        else:
            self.get_logger().error(f'❌ Undocking FAILED with status: {status}')

        self.is_currently_undocking = False

    # ── HELPERS ───────────────────────────────────────────────

    def set_charging(self, value: bool):
        self.is_charging = value  # track locally
        msg = Bool()
        msg.data = value
        self._charging_pub.publish(msg)
        self.get_logger().info(f'Charging set to: {value}')

def main(args=None):
    rclpy.init()
    my_node = AutoDocking()
    rclpy.spin(my_node)
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()