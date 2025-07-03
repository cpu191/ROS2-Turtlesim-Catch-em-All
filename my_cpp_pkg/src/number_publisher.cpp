// The number_publisher node publishes a number (always the same) on the
// “/number” topic,
//  with the existing type example_interfaces/msg/Int64.

#include "example_interfaces//msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class number_publisher : public rclcpp::Node // MODIFY NAME
{
public:
  number_publisher() : Node("number_publisher_cpp"), number_(2) {
    this->declare_parameter("number", 2);
    this->declare_parameter("timer_period", 1.0);
    publisher_ =
        this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    RCLCPP_INFO(this->get_logger(), "Number Publisher Node has been created.");
    double timer_period_ = this->get_parameter("timer_period").as_double();
    number_ = this->get_parameter("number").as_int();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&number_publisher::publish_number, this));
  }

private:
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t number_; // The number to be published

  void publish_number() {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", msg.data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<number_publisher>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}