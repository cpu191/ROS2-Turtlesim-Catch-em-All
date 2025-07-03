#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from example_interfaces.msg import Int64

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher") 
        self.declare_parameter("counter", 5)
        self.declare_parameter("timer_period", 0.7)
        self.counter_ = self.get_parameter("counter").value
        self.timer_period_ = self.get_parameter("timer_period").value
        
        self.add_post_set_parameters_callback(self.parameter_callback)

        self.publisher_ = self.create_publisher(Int64, "number", 10)
        self.timers_ = self.create_timer(self.timer_period_, self.publish_number)
        self.get_logger().info("PY Number Publisher has been started.")

    def parameter_callback(self, params:list[Parameter]):
        for param in params:
            if param.name =="counter":
                self.counter_ = param.value
                self.get_logger().info(f"Counter parameter updated to {self.counter_}") 

    def publish_number(self):
        msg = Int64()
        msg.data = self.counter_
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
