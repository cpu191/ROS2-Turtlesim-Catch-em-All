# ROS2-Turtlesim-Catch-em-All
This is the final project of the ROS 2 course. It showcases a complete mini-system built with ROS 2 (Python), where multiple turtles are dynamically spawned, tracked, and removed in the Turtlesim simulation environment.
It demonstrates core ROS 2 features such as nodes, topics, services, publishers, and subscribers using the Turtlesim simulator.

The system contains two nodes:

*turtle_manager*:
  - Spawns turtles at random locations.
  - Publishes each turtle’s name and pose for other nodes to use.

*turtle_intercept*:
  - Subscribes to the published target information.
  - Selects the closest active turtle as the target.
  - Publishes velocity commands to /turtle1/cmd_vel to navigate turtle1 toward the target.
  - Requests the kill service once the target is reached.

*The motion control currently uses a proportional (P) controller.
There are plans to upgrade to PD or PID control for smoother movement.*

**This project showcases:**
* Creation and coordination of multiple ROS 2 nodes.*
*  Real-time communication via publishers and subscribers.*
*  Use of ROS 2 services for spawning and killing entities.*
*  Basic feedback control in a simulation environment.*
