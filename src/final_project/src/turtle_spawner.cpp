#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include <random>

using namespace std::chrono_literals;

class TurtleSpawnerClientNode : public rclcpp::Node
{
public:
    TurtleSpawnerClientNode() : Node("turtle_spawner")
    {
        client_= this->create_client<turtlesim::srv::Spawn>("spawn");
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                    std::bind(&TurtleSpawnerClientNode::callSpawnTurtle, this));
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner Node started");
    }

private:
    void callSpawnTurtle()
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the turtlesim spawn service...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = getRandomFloat(1.0, 10.0);
        request->y = getRandomFloat(1.0, 10.0);
        request->theta = getRandomFloat(0.0, 2 * M_PI);

        RCLCPP_INFO(this->get_logger(), "Spawning turtle at (%.2f, %.2f, %.2f)",
                    request->x, request->y, request->theta);

        client_->async_send_request(
            request, std::bind(&TurtleSpawnerClientNode::callbackCallSpawnTurtle, this, std::placeholders::_1));
    }

    void callbackCallSpawnTurtle(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", response->name.c_str());
    }

    float getRandomFloat(float min, float max)
    {
        std::uniform_real_distribution<float> dist(min, max);
        return dist(rng_);
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::default_random_engine rng_{std::random_device{}()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
