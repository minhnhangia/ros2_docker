#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), is_battery_full_{true}
    {
        client_= this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        timer_ = this->create_wall_timer(std::chrono::seconds(10),
                                    std::bind(&BatteryNode::simulateBatteryLife, this));

        RCLCPP_INFO(this->get_logger(), "Battery Node started");
    }

private:
    void callSetLED(const int led_number, bool state)
    {
        while (!client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        client_->async_send_request(
            request, std::bind(&BatteryNode::callbackcallSetLED, this, std::placeholders::_1));
    }

    void callbackcallSetLED(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "%s", response->debug_message.c_str());
    }
    
    void simulateBatteryLife()
    {
        is_battery_full_ = !is_battery_full_;
        if (is_battery_full_)
        {
            RCLCPP_INFO(this->get_logger(), "Battery is full!");
            callSetLED(BATTERY_CHARGING_LED_, false);
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging...");
            callSetLED(BATTERY_CHARGING_LED_, true);
        }
    }

    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;

    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr int BATTERY_CHARGING_LED_{3};
    bool is_battery_full_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}