#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        subscription_ = create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callback_number, this, std::placeholders::_1));
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        reset_counter_service_ = create_service<example_interfaces::srv::SetBool>(
            "reset_counter", std::bind(&NumberCounterNode::callback_reset_counter, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Number counter has been started.");
    }

private:
    void callback_number(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = counter_;
        publisher_->publish(new_msg);
    }

    void callback_reset_counter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                                example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            response->message = "Counter successfully reset";
            RCLCPP_INFO(get_logger(), "Counter has been reset");
        }
        else
        {
            response->success = false;
            response->message = "Counter has not been reset";
            RCLCPP_INFO(get_logger(), "Counter has not been reset");
        }
    }

    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_service_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounterNode>());
    rclcpp::shutdown();
    return 0;
}