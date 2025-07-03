#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClient : public rclcpp::Node // MODIFY NAME
{
public:
  AddTwoIntsClient()
      : Node("add_two_ints_client") // MODIFY NAME
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
        "add_two_ints");
  }

  void callAddTwoInts(int a, int b) {
    while (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
                  "Service not available, waiting again...");
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
    }

    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    auto future = client_->async_send_request(
        request, std::bind(&AddTwoIntsClient::callback_AddTwoInts, this, _1));
  }

private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  void callback_AddTwoInts(
      rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture
          future) {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Result of add_two_ints: %ld",
                response->sum);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClient>(); // MODIFY NAME
  node->callAddTwoInts(98, 12);                     // Example call
  node->callAddTwoInts(9, 3);                       // Example call
  node->callAddTwoInts(2, 7);                       // Example call
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
