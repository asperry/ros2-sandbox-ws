#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLedState

class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel")
        self.panel_state_ = [False, False, False]
        self.panel_state_publisher_ = self.create_publisher(
            LedPanelState, "led_panel_state", 10)
        self.panel_state_publishing_timer_ = self.create_timer(1.0, self.publish_panel_state)
        self.set_led_service_ = self.create_service(
            SetLedState, "set_led", self.callback_set_led)
        self.get_logger().info("Led Panel has been started.")

    def publish_panel_state(self):
        msg = LedPanelState()
        msg.state = self.panel_state_
        self.panel_state_publisher_.publish(msg)

    def callback_set_led(self, request, response):
        led_number = request.led_number
        state = request.state

        if led_number > len(self.panel_state_) or led_number <= 0:
            response.success = False
            return response

        self.panel_state_[led_number - 1] = state
        response.success = True
        self.publish_panel_state()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
