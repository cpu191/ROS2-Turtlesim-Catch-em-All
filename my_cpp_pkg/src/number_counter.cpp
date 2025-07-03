// The number_counter node subscribes to the “/number” topic. It keeps a counter
// variable. Every time a new number is received, it’s added to the counter.
// The node also has a publisher on the “/number_count” topic.
// When the counter is updated, the publisher directly publishes the new value
// on the topic. Create a subscription to the "number" topic The callback
// function is called when a new message is received The message type is
// example_interfaces::msg::Int64 The queue size is set to 10
#include "example_interfaces//msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class NumberCounter : public rclcpp::Node // MODIFY NAME
{
public:
  NumberCounter() : Node("number_counter"), count_(0) {
    subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10,
        std::bind(&NumberCounter::number_callback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<example_interfaces::msg::Int64>(
        "number_count", 10);
    service_ = this->create_service<example_interfaces::srv::SetBool>(
        "reset_counter",
        std::bind(&NumberCounter::reset_counter_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Number Counter Node has been created.");
  }

private:
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t count_; // The counter variable

  void number_callback(const example_interfaces::msg::Int64::SharedPtr msg) {
    count_ += msg->data;
    auto updated_msg = example_interfaces::msg::Int64();
    updated_msg.data = count_;
    publisher_->publish(updated_msg);
    RCLCPP_INFO(this->get_logger(), "Updated counter: '%ld'", count_);
  }

  void reset_counter_callback(
      const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
      std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    if (request->data) {
      count_ = 0;
      response->success = true;
      response->message = "Counter reset to 0.";
      RCLCPP_INFO(this->get_logger(), "Counter reset to 0.");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounter>(); // MODIFY NAME
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}