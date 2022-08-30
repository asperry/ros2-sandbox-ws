#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&AddTwoIntsServerNode::callback_add_two_ints, this, _1, _2));
        RCLCPP_INFO(get_logger(), "Add two ints server has been started");
    }

private:
    void callback_add_two_ints(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                               const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(get_logger(), "%d + %d = %d", request->a, request->b, response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddTwoIntsServerNode>());
    rclcpp::shutdown();
    return 0;
}