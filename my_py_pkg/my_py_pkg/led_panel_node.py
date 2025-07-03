import rclpy
from rclpy.node import Node

from led_panel_interfaces.msg import _led_panel_state
from led_panel_interfaces.srv import _set_led

class LedPanelNode(Node):
    def __init__(self):
        super().__init__("led_panel_node")
        self.declare_parameter("led_panel_state", [True, False, True])
        self.current_led_panel_state_ = _led_panel_state.LEDPanelState()
        self.current_led_panel_state_.led_panel_state=self.get_parameter("led_panel_state").value
        self.publishers_ = self.create_publisher(_led_panel_state.LEDPanelState, "led_panel_state", 10)
        self.timer = self.create_timer(1.0, self.publish_led_panel_state)
        self.service_ = self.create_service(_set_led.SetLED, "set_led", self.set_led_callback)
        
        self.get_logger().info("LED Panel Node has been started.")

    def publish_led_panel_state(self):
        current_state = _led_panel_state.LEDPanelState()
        current_state = self.current_led_panel_state_
        self.publishers_.publish(current_state)

    def set_led_callback(self, request: _set_led.SetLED.Request, response: _set_led.SetLED.Response):
        #self.get_logger().info(f"Set LED {request._set_led_state[0]} , {request._set_led_state[1]} , {request._set_led_state[2]}")
        self.current_led_panel_state_.led_panel_state= request._set_led_state
        response.success = True
        response.debug_message = "LED panel state updated successfully."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
