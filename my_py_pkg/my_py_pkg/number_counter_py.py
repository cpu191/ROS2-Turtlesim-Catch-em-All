#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class Number_Counter(Node): 
    def __init__(self):
        super().__init__("Number_Counter") 
        self.counter_ = 0
        self.subscription_ = self.create_subscription(Int64, "number",self.listener_callback, 10)
        self.publishers_ = self.create_publisher(Int64, "number_count", 10)
        self.services_= self.create_service(SetBool, "reset_counter", self.reset_counter_callback,)
       #self.timers_ = self.create_timer(0.5, self.publish_number)

    def listener_callback(self,msg:Int64):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.publishers_.publish(new_msg)
        self.get_logger().info(f"Received: {msg.data}, Counter: {self.counter_}")

    def reset_counter_callback(self, request:SetBool.Request, response:SetBool.Response):
        if request.data== 1:
            self.counter_ = 0
            response.success = True
            response.message = "Counter reset to 0"
            self.get_logger().info(f"Reset counter: {self.counter_}")
        else:
            response.success = False
            response.message = "Counter not reset"
            self.get_logger().info(f"Counter not reset: {self.counter_}")
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = Number_Counter() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
