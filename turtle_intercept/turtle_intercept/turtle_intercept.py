#!/usr/bin/env python3
import math
import rclpy
from typing import List
from dataclasses import dataclass
from std_msgs.msg import String
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from geometry_msgs.msg import Twist


@dataclass
class TurtleInfo:
    name: String
    pose: Pose

class TurtleIntercept(Node): 
    def __init__(self):
        super().__init__("turtle_intercept") 
        self.get_logger().info("Turtle Intercept Node has been started.")
        self.declare_parameter("kill_distance", 0.1)  # Default kill distance
        self.kill_distance_ = self.get_parameter("kill_distance").value
        self.target_name_ = None
        self.target_pose_ = None
        self.current_target_ = None
        self.subscriber_ = self.create_subscription(String,"spawned_target_name",self.target_name_callback,10)
        #Make a list to keep track of target turtles and their poses
        self.targets_ = []
        self.subscriber_pose_ = self.create_subscription(Pose, "spawned_target_pose", self.target_pose_callback, 10)
        self.turtle_pose_ = Pose() 
        self.turtle_pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.turtle_pose_callback, 10)
        self.publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.timer_ = self.create_timer(0.1, self.move_turtle)  # Timer to call move_turtle every 0.1 seconds
        self.kill_turtle_ = self.create_client(Kill, "kill")

    def turtle_pose_callback(self, msg):
        self.turtle_pose_ = msg

    def target_name_callback(self, msg):
        self.target_name_ = msg.data
        self.combine_target_info()

    def target_pose_callback(self, msg):
        self.target_pose_ = msg
        self.combine_target_info()

    def combine_target_info(self):
        for target in self.targets_:
            if target.name.data == self.target_name_ or target.pose == self.target_pose_:
                return  # If the target already exists, do not add it again
            
        if self.target_name_ is not None and self.target_pose_ is not None:
            target_info = TurtleInfo(name=String(data=self.target_name_), pose=self.target_pose_)
            self.targets_.append(target_info)
            # self.get_logger().info(f"Target turtle info updated: {target_info.name.data} at pose ({target_info.pose.x}, {target_info.pose.y})")
    
    def calculate_relative_pose(self, current_pose:Pose, goal_pose:Pose):
        dx = goal_pose.x - current_pose.x
        dy = goal_pose.y - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx) - current_pose.theta
        angle = (angle + math.pi) % (2 * math.pi) - math.pi  # Normalize angle to [-pi, pi]
        return Pose(x=dx, y=dy, theta=angle), distance

    def get_minimum_distance_target(self):
        closest_distance = 15.56  # Initialize with the max possible distance in turtlesim
        current_target = None
        for target in self.targets_:
            relative_pose, distance = self.calculate_relative_pose(self.turtle_pose_, target.pose)
            if distance < closest_distance:
                closest_distance = distance
                current_target = target
        return current_target, closest_distance

    def move_turtle(self):

        if not self.targets_:
            self.get_logger().info("No target turtles available to move towards.")
            return
        # self.get_logger().info("Looping through targets...")
        # for target in self.targets_:
        #     self.get_logger().info(f"Target: {target.name.data}, Pose: ({target.pose.x}, {target.pose.y})")
        if self.target_name_ is not None:
            k_angular = 2.5  # Proportional gain for angular speed
            k_linear = 1.5  # Proportional gain for linear speed
            relative_pose = Pose(theta=0.0) # Initialize relative pose
            current_target, distance = self.get_minimum_distance_target()
            Twist_msg = Twist()
            Twist_msg.linear.x = k_linear * distance

            if current_target is not None:
                relative_pose, _ = self.calculate_relative_pose(self.turtle_pose_, current_target.pose)
           
            if abs(relative_pose.theta) < 0.1:  # Small threshold to prevent jitter
                Twist_msg.angular.z = 0.0               
                
            else:
                Twist_msg.angular.z = relative_pose.theta*k_angular  # Adjust angular speed

            
            self.publisher_.publish(Twist_msg)
            # self.get_logger().info(f"twist angular.z={Twist_msg.angular.z},theta={relative_pose.theta}, distance={distance}")
            if distance < self.kill_distance_:
                self.kill_turtle(current_target.name.data)
                # self.targets_.remove(current_target)

        else:
            # self.get_logger().info("No target turtle to move towards.")
            delay = 0.5
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=delay))  # Sleep


    def kill_turtle(self, turtle_name):
        self.get_logger().info(f"Killing turtle: {turtle_name}")
        
        if not self.kill_turtle_.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Kill service not available.")
            return
        
        request = Kill.Request()
        request.name = turtle_name
        _,distance = self.calculate_relative_pose(self.turtle_pose_, self.target_pose_)
        self.get_logger().info(f"Distance to target turtle: {distance}")
        dx = self.target_pose_.x - self.turtle_pose_.x
        dy = self.target_pose_.y - self.turtle_pose_.y
        self.get_logger().info(f"Relative position to target turtle: dx={dx}, dy={dy}")
        future = self.kill_turtle_.call_async(request)
        future.add_done_callback(lambda fut: self.handle_kill_response(fut, turtle_name))
        
    def handle_kill_response(self, future, turtle_name):
        if future.result() is not None:

            for target in self.targets_:
                self.get_logger().info(f"looping through {target.name.data}")
                if target.name.data == turtle_name:
                    self.get_logger().info(f"removing {target.name.data}")
                    self.targets_.remove(target)
                    break
            
            self.get_logger().info(f"Turtle {turtle_name} killed successfully.")
            return
        else:
            self.get_logger().error("Failed to kill turtle.")
            return
            
            
def main(args=None):
    rclpy.init(args=args)
    node = TurtleIntercept()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
