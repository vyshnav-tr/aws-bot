#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

class BatteryStatePublisher(Node):

  def __init__(self):
    super().__init__('battery_state_pub')

    self.publisher_battery_state = self.create_publisher(BatteryState, '/battery_status', 10)
    self.create_subscription(Bool, '/set_charging', self.charging_callback, 10)

    self.timer = self.create_timer(1.0, self.get_battery_state)

    self.is_charging = False
    self.battery_voltage = 9.0
    self.percent_charge_level = 0.10  # start at 10%
    self.decrement_factor = 0.91     # discharge multiplier
    self.increment_factor = 0.05      # 1% per second when charging

  def charging_callback(self, msg):
    self.is_charging = msg.data
    self.get_logger().info(f'is_charging set to: {self.is_charging}')

  def get_battery_state(self):
    msg = BatteryState()
    msg.voltage = self.battery_voltage
    msg.percentage = self.percent_charge_level
    self.publisher_battery_state.publish(msg)

    if self.is_charging:
      self.percent_charge_level = min(1.0, self.percent_charge_level + self.increment_factor)
      self.battery_voltage = min(12.6, self.battery_voltage + self.increment_factor * 10)
    else:
      self.percent_charge_level = max(0.0, self.percent_charge_level * self.decrement_factor)
      self.battery_voltage = max(0.0, self.battery_voltage * self.decrement_factor)

    self.get_logger().info(f'Battery: {self.percent_charge_level*100:.1f}% | {self.battery_voltage:.2f}V | charging: {self.is_charging}')

def main(args=None):
  rclpy.init(args=args)
  battery_state_pub = BatteryStatePublisher()
  rclpy.spin(battery_state_pub)
  battery_state_pub.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()