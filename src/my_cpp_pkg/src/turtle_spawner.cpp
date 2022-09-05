#include <memory>
#include <functional>
#include <chrono>
#include <future>
#include <random>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;


struct Turtle
{
    float x, y;
    std::string name;
};


class TurtleSpawnerNode : public rclcpp::Node
{
public:
    TurtleSpawnerNode() : Node("turtle_spawner"), gen(rd()), pos_distrib(0.0, 11.0), theta_distrib(0, 2 * M_PI)
    {
        spawn_client_ = create_client<turtlesim::srv::Spawn>("spawn");
        kill_client_ = create_client<turtlesim::srv::Kill>("kill");
        pose_subscription_ = create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleSpawnerNode::callbackTurtle1Pose, this, _1));
        target_turtle_publisher_ = create_publisher<turtlesim::msg::Pose>("target_turtle", 10);
        spawn_timer_ = create_wall_timer(
            std::chrono::seconds(1), std::bind(&TurtleSpawnerNode::spawnRandomTurtle, this));
    }

private:
    void callbackTurtle1Pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        // Save turtle1 pose
        current_turtle1_pose_.x = pose->x;
        current_turtle1_pose_.y = pose->y;

        // If turtle1 has arrived near enough to a target turtle, kill it and send a new target turtle
        float threshold = 0.5;
        if (target_turtles_.size() > 0 && computeDistanceToTurtle1(current_target_turtle_) < threshold)
        {
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();

            target_update_mutex_.lock();
            kill_request->name = current_target_turtle_.name;
            target_turtles_.erase(target_turtles_.begin() + current_target_turtle_index_);
            if (target_turtles_.size() > 0)
                publishNewTargetTurtle();
            target_update_mutex_.unlock();

            kill_client_->async_send_request(kill_request);
        }
    }

    void spawnRandomTurtle()
    {
        // Generate a random turtle spawn request
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = pos_distrib(gen);
        request->y = pos_distrib(gen);
        request->theta = theta_distrib(gen);

        auto future = spawn_client_->async_send_request(request);
        spawn_future_async_ = std::async(std::launch::async, &TurtleSpawnerNode::handleSpawnResponse, this, future, request);
    }

    void handleSpawnResponse(std::shared_future<turtlesim::srv::Spawn::Response::SharedPtr> future,
                             const turtlesim::srv::Spawn::Request::SharedPtr request)
    {
        try
        {
            auto response = future.get();

            // Save the new turtle to the set of target turtles
            target_update_mutex_.lock();
            target_turtles_.push_back({request->x, request->y, response->name});
            publishNewTargetTurtle();
            target_update_mutex_.unlock();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Service call failed");
        }
    }

    float computeDistanceToTurtle1(Turtle other_turtle)
    {
        return std::sqrt(std::pow(current_turtle1_pose_.x - other_turtle.x, 2) + 
                         std::pow(current_turtle1_pose_.y - other_turtle.y, 2));
    }

    void publishNewTargetTurtle()
    {
        // Find the nearest turtle to turtle1
        float smallest_dist = 1000;
        for (unsigned long i = 0; i < target_turtles_.size(); i++)
        {
            float dist = computeDistanceToTurtle1(target_turtles_.at(i));
            if (dist < smallest_dist)
            {
                smallest_dist = dist;
                current_target_turtle_index_ = i;
            }
        }
        current_target_turtle_ = target_turtles_.at(current_target_turtle_index_);

        // Publish the nearest turtle as the new target turtle
        auto target_turtle_pose = turtlesim::msg::Pose();
        target_turtle_pose.x = current_target_turtle_.x;
        target_turtle_pose.y = current_target_turtle_.y;
        target_turtle_publisher_->publish(target_turtle_pose);
    }

    std::random_device rd;  // Used to obtain a seed for the random number engine
    std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> pos_distrib;
    std::uniform_real_distribution<> theta_distrib;
    std::shared_future<void> spawn_future_async_;
    std::vector<Turtle> target_turtles_;
    std::mutex target_update_mutex_;
    Turtle current_target_turtle_;
    unsigned long current_target_turtle_index_ = 0;
    turtlesim::msg::Pose current_turtle1_pose_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<turtlesim::msg::Pose>::SharedPtr target_turtle_publisher_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSpawnerNode>());
    rclcpp::shutdown();
    return 0;
}