#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::publish_number, this));
        RCLCPP_INFO(get_logger(), "Number publisher has been started.");
    }

private:
    void publish_number()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = 2;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NumberPublisherNode>());
    rclcpp::shutdown();
    return 0;
}