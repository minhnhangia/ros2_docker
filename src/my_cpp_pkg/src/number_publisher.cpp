#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPubNode : public rclcpp::Node
{
public:
    NumberPubNode() 
    : Node("number_publisher")
    {
        this->declare_parameter("number", 1);
        this->declare_parameter("timer_period", 1.0);
        number_ = this->get_parameter("number").as_int();
        double timer_period = this->get_parameter("timer_period").as_double();

        param_callback_handle_ = this->add_post_set_parameters_callback(
            std::bind(&NumberPubNode::parametersCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(timer_period),
                                         std::bind(&NumberPubNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher started");
    }

private:
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }

    void parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param: parameters)
        {
            if (param.get_name() == "number")
                number_ = param.as_int();
        }
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    int number_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPubNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
