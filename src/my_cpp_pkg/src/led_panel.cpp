#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_status.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class LEDPanelNode : public rclcpp::Node 
{
public:
    LEDPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_states", std::vector<bool>{false,false,false});
        led_states_ = this->get_parameter("led_states").as_bool_array();

        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedStatus>("led_status", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&LEDPanelNode::publishLEDStatus, this));

        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
        "set_led", 
        std::bind(&LEDPanelNode::callbackSetLED, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "LED Panel Node started");
    }

private:
    void publishLEDStatus()
    {
        auto msg = my_robot_interfaces::msg::LedStatus();
        msg.is_led1_on = led_states_[0];
        msg.is_led2_on = led_states_[1];
        msg.is_led3_on = led_states_[2];
        publisher_->publish(msg);
    }

    void  callbackSetLED(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                             const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        const int led_index = request->led_number - 1;

        if (led_index < 0 || led_index >= static_cast<int>(led_states_.size()))
        {
            response->is_success = false;
            response->debug_message = "LED " + std::to_string(request->led_number) + " is invalid";
            return;
        }
        
        led_states_[led_index] = request->state;
        response->is_success = true;
        response->debug_message = "LED " + std::to_string(request->led_number) + " has been set";
        RCLCPP_INFO(this->get_logger(), "LED %d set to %s", 
            (int)request->led_number, request->state ? "ON" : "OFF");
    }

    rclcpp::Publisher<my_robot_interfaces::msg::LedStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    
    std::vector<bool> led_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEDPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
