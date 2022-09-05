#include <memory>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;


class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller")
    {
        turtle1_pose_subscription_ = create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackTurtle1Pose, this, _1));
        target_turtle_subscription_ = create_subscription<turtlesim::msg::Pose>(
            "target_turtle", 10, std::bind(&TurtleControllerNode::callbackTargetTurtle, this, _1));
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    }

private:
    void callbackTurtle1Pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        if (received_first_target_)
        {
            // Compute a new command velocity based on the current and target turtle positions
            // and publish it on the turtle1/cmd_vel
            auto new_cmd_vel = geometry_msgs::msg::Twist();

            float dist = std::sqrt(std::pow(current_target_turtle_.x - pose->x, 2) + 
                                   std::pow(current_target_turtle_.y - pose->y, 2));
            new_cmd_vel.linear.x = 2.0 * dist;

            // Cross product approach for controlling orientation
            // new_cmd_vel.angular.z =  6.0 * (std::cos(pose->theta) * (current_target_turtle_.y - pose->y) - 
            //                                 std::sin(pose->theta) * (current_target_turtle_.x - pose->x));

            // Difference approach for controlling orientation
            new_cmd_vel.angular.z = std::atan2(current_target_turtle_.y - pose->y, current_target_turtle_.x - pose->x) - pose->theta;
            if (new_cmd_vel.angular.z > M_PI)
                new_cmd_vel.angular.z -= 2*M_PI;
            else if (new_cmd_vel.angular.z < -M_PI)
                new_cmd_vel.angular.z += 2*M_PI;
            new_cmd_vel.angular.z *= 6.0;

            cmd_vel_publisher_->publish(new_cmd_vel);
        }
    }

    void callbackTargetTurtle(const turtlesim::msg::Pose::SharedPtr target_turtle_pose)
    {
        current_target_turtle_.x = target_turtle_pose->x;
        current_target_turtle_.y = target_turtle_pose->y;
        received_first_target_ = true;
    }

    bool received_first_target_ = false;
    turtlesim::msg::Pose current_target_turtle_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_subscription_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr target_turtle_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleControllerNode>());
    rclcpp::shutdown();
    return 0;
}