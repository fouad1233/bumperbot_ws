#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>

using namespace std::placeholders;
using namespace std::chrono_literals;


class SimpleServiceClient: public rclcpp::Node
{
public:
    SimpleServiceClient(int a, int b): Node("simple_service_client")
    {
        client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");
        auto request_ = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
        request_->a = a;
        request_->b = b;

        while (!client_->wait_for_service(1s)) {
            if(!rclcpp::ok()) {
                RCLCPP_ERROR_STREAM(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO_STREAM(get_logger(), "Service not available, waiting again...");
        }

        auto result_future = client_->async_send_request(request_,
            std::bind(&SimpleServiceClient::responseCallback, this, _1));
    }


private:
    rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;

    void responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid()) {
        RCLCPP_INFO_STREAM(get_logger(), "Response received: " << future.get()->sum);
        } else {
            RCLCPP_ERROR_STREAM(get_logger(), "Service call failed");
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);

    if (argc != 3) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Usage: simple_service_client <int a> <int b>");
        return 1;
    }

    int a = std::stoi(argv[1]);
    int b = std::stoi(argv[2]);

    auto node = std::make_shared<SimpleServiceClient>(a,b);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}