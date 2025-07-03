#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp::Node {
public:
  AddTwoIntsServerNode() : Node("add_two_ints_server") {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsServerNode::callback_AddTwoInts, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Ready to add two ints.");
  }

private:
  void callback_AddTwoInts(
      const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
      example_interfaces::srv::AddTwoInts::Response::SharedPtr response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(),
                "Incoming request: a=%ld, b=%ld, result =%ld", request->a,
                request->b, response->sum);
  }
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
