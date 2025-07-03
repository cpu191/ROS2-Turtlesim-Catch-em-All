#!/usr/bin/env python3
import rclpy
import random
from dataclasses import dataclass
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_msgs.msg import String 

@dataclass
class TurtleInfo:
    name: String
    pose: Pose

class TurtleManager(Node): 
    def __init__(self):
        super().__init__("turtle_manager")
        self.get_logger().info("Turtle Manager Node has been started.")
        self.target_turtles_ = []  # List to keep track of target turtles
        self.spawn_random_turtle_ = self.create_client(Spawn, "spawn")
  
        self.declare_parameter("spawn_interval", 3.0)  # Default spawn interval
        spawn_interval = self.get_parameter("spawn_interval").value
        self.spawn_timer = self.create_timer(spawn_interval, self.spawn_turtle)  # Timer to spawn a turtle every 3 seconds

        self.target_name_publisher = self.create_publisher(String, "spawned_target_name", 10)
        self.target_pose_publisher = self.create_publisher(Pose, "spawned_target_pose", 10)


    def spawn_turtle(self):
        while not self.spawn_random_turtle_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Spawn service not available. Waiting Again...")
        request = Spawn.Request()
        request.x = random.uniform(0.0,11.0)  # Random x position within the turtlesim window
        request.y = random.uniform(0.0, 11.0)  # Random y position within the turtlesim window
        request.theta = random.uniform(0.0, 3.14)  # Random theta (orientation) 
        spawn_pose = Pose()
        spawn_pose.x = request.x
        spawn_pose.y = request.y
        spawn_pose.theta = request.theta
        future = self.spawn_random_turtle_.call_async(request)
        future.add_done_callback(lambda fut: self.handle_spawn_response(fut, spawn_pose))

    def handle_spawn_response(self, future, spawn_pose):
        if future.result() is not None:
            turtle_name = String()
            turtle_name.data = future.result().name
            # self.target_turtles_.append(TurtleInfo(name=turtle_name, pose=spawn_pose))
            self.get_logger().info(f"Spawned turtle: {turtle_name.data}")
            # Publish the pose and name of the last spawned turtle
            self.target_pose_publisher.publish(spawn_pose)
            self.target_name_publisher.publish(turtle_name)
            

def main(args=None):
    rclpy.init(args=args)
    node = TurtleManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
