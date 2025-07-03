#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HardwareStatusPublisherNode : public rclcpp::Node {
public:
  HardwareStatusPublisherNode() : Node("hardware_status_publisher") {
    pub_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
        "hardware_status", 10);
    timer_ = this->create_wall_timer(
        1s, std::bind(&HardwareStatusPublisherNode::timer_callback, this));
  }

private:
  rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback() {
    auto message = my_robot_interfaces::msg::HardwareStatus();
    message.temperature = 25.0;      // Example temperature value
    message.are_motors_ready = true; // Example motor status
    message.debug_message = "All systems operational"; // Example debug message
    pub_->publish(message);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareStatusPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
