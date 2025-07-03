#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import random

from led_panel_interfaces.srv import SetLED

class BatteryNode(Node): 
    def __init__(self):
        super().__init__("battery_node") 
        self.battery_level_ = float(100)
        self.client_ = self.create_client(SetLED, "set_led")
        self.get_logger().info("Battery Node has been started, battery level is 100%")

    def battery_low_callback(self):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service Set LED not available, waiting again...")
        request = SetLED.Request()
        if self.battery_level_ < 10 and request.set_led_state == [False, False, False]:
            self.get_logger().info("Battery level is low, turning on LED 1")
            request.set_led_state = [True, False, False]
        if self.battery_level_ == 100:
            self.get_logger().info("Battery level is sufficient, turning off LED 1")
            request.set_led_state = [False, False, False]

        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    up=True
    while rclpy.ok():
        node.battery_level_ += random.uniform(0, 5) * (1 if up else -1)
        node.battery_level_ = min(max(node.battery_level_, 0.0), 100.0)
        if node.battery_level_ in (0.0, 100.0): up = not up
        print(f"{node.battery_level_:.1f}%")
        time.sleep(0.5)
        node.battery_low_callback()
    
if __name__ == "__main__":
    main()
