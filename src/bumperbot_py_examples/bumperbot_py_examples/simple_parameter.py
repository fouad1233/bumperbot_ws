import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter_node")
        # Declare a parameter with a default value
        self.declare_parameter("simple_init_param", 28)
        self.declare_parameter("simple_string_param", "Hello,Fouad")
        
        self.add_on_set_parameters_callback(self.param_change_callback)

    def param_change_callback(self, params):
        result = SetParametersResult()
        for param in params:
            if param.name == "simple_init_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Parameter 'simple_int_param' changed to: {param.value}")
                result.successful = True
            elif param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Parameter 'simple_string_param' changed to: {param.value}")
                result.successful = True
            else:
                result.successful = False
                result.reason = f"Parameter '{param.name}' change rejected."
        return result

def main():
    rclpy.init()
    simple_param_node = SimpleParameter()
    rclpy.spin(simple_param_node)
    simple_param_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()