#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
using std::placeholders::_1;
class SimpleParameterNode : public rclcpp::Node
{
public:
    SimpleParameterNode(): Node("simple_parameter_node")
    {
        // Declare a parameter with a default value
        this->declare_parameter<int>("simple_int_param", 28);
        this->declare_parameter<std::string>("simple_string_param", "Hello, ROS2!");

        param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameterNode::paramChangeCallback, this, _1));
        // Log the parameter values
        RCLCPP_INFO(this->get_logger(), "Integer parameter: %d", this->get_parameter("simple_int_param").as_int());
        RCLCPP_INFO(this->get_logger(), "String parameter: %s", this->get_parameter("simple_string_param").as_string().c_str());
    }

private:
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult paramChangeCallback(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "simple_int_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                int new_value = param.as_int();
                RCLCPP_INFO(this->get_logger(), "simple_int_param changed to: %d", new_value);
                result.successful = true;
            }
            else if (param.get_name() == "simple_string_param" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                std::string new_value = param.as_string();
                RCLCPP_INFO(this->get_logger(), "simple_string_param changed to: %s", new_value.c_str());
                result.successful = true;
            }
            else
            {
                result.successful = false;
                result.reason = "Invalid parameter or type";
                RCLCPP_WARN(this->get_logger(), "Failed to change parameter: %s", param.get_name().c_str());
                return result;
            }
        }
        return result;
    }
};

int main (int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleParameterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

