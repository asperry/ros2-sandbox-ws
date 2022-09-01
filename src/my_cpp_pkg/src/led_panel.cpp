#include <memory>
#include <functional>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"
#include "my_robot_interfaces/srv/set_led_state.hpp"

using namespace std::placeholders;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        panel_state_publisher_ = create_publisher<my_robot_interfaces::msg::LedPanelState>(
            "led_panel_state", 10);
        panel_state_publishing_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&LedPanelNode::publishPanelState, this));
        set_led_service_ = create_service<my_robot_interfaces::srv::SetLedState>(
            "set_led",
            std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Led Panel has been started.");
    }

private:
    void publishPanelState()
    {
        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.state = panel_state_;
        panel_state_publisher_->publish(msg);
    }

    void callbackSetLed(const my_robot_interfaces::srv::SetLedState::Request::SharedPtr request,
                          my_robot_interfaces::srv::SetLedState::Response::SharedPtr response)
    {
        if (request->led_number > 0 && request->led_number < 4)
        {
            panel_state_[request->led_number - 1] = request->state;
            response->success = true;
            publishPanelState();
            RCLCPP_INFO(get_logger(), "Updated LED %d to %s", request->led_number, request->state ? "true" : "false");
        }
        else
        {
            response->success = false;
            RCLCPP_WARN(get_logger(), "Requested LED number must be 1, 2, or 3.");
        }
    }

    std::array<bool, 3> panel_state_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr panel_state_publisher_;
    rclcpp::Service<my_robot_interfaces::srv::SetLedState>::SharedPtr set_led_service_;
    rclcpp::TimerBase::SharedPtr panel_state_publishing_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LedPanelNode>());
    rclcpp::shutdown();
    return 0;
}