#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus 


class HardwareStatusPublisherNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("hardware_status_publisher") # MODIFY NAME 
        self.publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10) # MODIFY TOPIC NAME
        self.timer = self.create_timer(1.0, self.publish_hardware_status)
        self.get_logger().info("Hardware Status Publisher Node has been started.")
        
    def publish_hardware_status(self):
        msg = HardwareStatus()
        msg.are_motors_ready = True
        msg.temperature = 75.0
        msg.debug_message = "All systems operational"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
