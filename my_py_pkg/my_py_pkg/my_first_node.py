#interpreter: python3
#!/usr/bin/env python3

#import ros2 library for python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("Hello, ROS2 Py")
        self.create_timer(1.0,self.timer_callback)

      
    # Create a timer that calls the timer_callback function every second
    def timer_callback(self):
        self.get_logger().info("Hello  "+ str(self.counter_))
        self.counter_  += 1

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    # keep the node alive
    rclpy.spin(node)

    node.timer_callback
    # Shutdown the node
    rclpy.shutdown()

if __name__ == "__main__":
    main()