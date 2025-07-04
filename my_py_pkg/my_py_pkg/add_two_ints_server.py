#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  

class AddTwoIntsServerNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("add_two_ints_server") # MODIFY NAME 
        self.server_=self.create_service(AddTwoInts,"add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("AddTwoInts server has been started.")
    
    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Incoming request: {request.a} + {request.b} = {response.sum}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
