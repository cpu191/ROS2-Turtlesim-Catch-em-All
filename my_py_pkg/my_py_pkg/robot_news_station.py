#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__('robot_news_station')
        self.declare_parameter("robot_name", "C3PO-FAKE")
        self.robot_name_ = self.get_parameter("robot_name").value
        self.publishers__ = self.create_publisher(String,"robot_news", 10)
        self.timer__ = self.create_timer(1,self.publish_news)
        self.get_logger().info(f"Robot News Station {self.robot_name_} has been started.")
        self.add_post_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == "robot_name":
                self.robot_name_ = param.value
                self.get_logger().info(f"Robot name changed to: {self.robot_name_}")

    def publish_news(self):
        msg = String()
        msg.data = f"Hello, this is {self.robot_name_} from the Robot News Station!"
        self.publishers__.publish(msg)


def main (args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()