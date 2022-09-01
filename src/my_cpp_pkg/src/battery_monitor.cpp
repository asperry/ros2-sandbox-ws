#include <memory>
#include <functional>
#include <chrono>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led_state.hpp"

class BatteryMonitorNode : public rclcpp::Node
{
public:
    BatteryMonitorNode() : Node("battery_monitor")
    {
        set_led_client_ = create_client<my_robot_interfaces::srv::SetLedState>("set_led");
        while (!set_led_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Waiting for the server to be up...");
        }
        battery_state_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&BatteryMonitorNode::callbackBatteryStateTimer, this));
        RCLCPP_INFO(get_logger(), "Battery monitor has been started.");
    }

private:
    void callbackBatteryStateTimer()
    {
        RCLCPP_INFO(get_logger(), "Timer ticked off.");
        seconds_until_battery_state_change_--;
        if (seconds_until_battery_state_change_ == 0)
        {
            battery_state_ = !battery_state_;
            battery_state_ ? seconds_until_battery_state_change_ = 4 : seconds_until_battery_state_change_ = 6;
            RCLCPP_INFO(get_logger(), battery_state_ ? "Battery is charged" : "Battery is depleted");
            
            auto request = std::make_shared<my_robot_interfaces::srv::SetLedState::Request>();
            request->led_number = 3;
            request->state = !battery_state_;

            auto future = set_led_client_->async_send_request(request);
            future_async_ = std::async(std::launch::async, &BatteryMonitorNode::handleSetLedResponse, this, future);
        }
    }

    void handleSetLedResponse(std::shared_future<my_robot_interfaces::srv::SetLedState::Response::SharedPtr> future)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(get_logger(), "Service request %s", response->success ? "succeeded" : "failed");
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Service call failed");
        }
    }

    bool battery_state_ = true;
    int seconds_until_battery_state_change_ = 4;
    std::shared_future<void> future_async_;
    rclcpp::Client<my_robot_interfaces::srv::SetLedState>::SharedPtr set_led_client_;
    rclcpp::TimerBase::SharedPtr battery_state_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatteryMonitorNode>());
    rclcpp::shutdown();
    return 0;
}