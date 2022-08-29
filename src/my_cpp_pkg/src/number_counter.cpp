#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter"), counter_(0)
    {
        subscription_ = create_subscription<example_interfaces::msg::Int64>(
            "number", 10, std::bind(&NumberCounterNode::callback_number, this, std::placeholders::_1));
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number_count", 10);
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

    int counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberCounterNode>());
    rclcpp::shutdown();
    return 0;
}