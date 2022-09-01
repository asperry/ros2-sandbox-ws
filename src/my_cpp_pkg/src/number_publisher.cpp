#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"


class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        declare_parameter("number_to_publish", 2);
        declare_parameter("publish_frequency", 1.0);

        publisher_ = create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(int(1000.0 / get_parameter("publish_frequency").as_double())),
                                   std::bind(&NumberPublisherNode::publish_number, this));
        RCLCPP_INFO(get_logger(), "Number publisher has been started.");
    }

private:
    void publish_number()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = get_parameter("number_to_publish").as_int();
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