#include "bumperbot_cpp_examples/simple_tf_kinematics.hpp"
using namespace std::chrono_literals;
SimpleTfKinematics::SimpleTfKinematics(const std::string & name) 
    : Node(name) // Call the base class constructor because Node has no default constructor
    , x_increment_(0.05) // 5 cm per timer callback
    , last_x_(0.0) // Start from x = 0.0
    , rotations_counter_(0)
{
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    // Define a static transform from "world" to "odom"
    static_transform_stamped_.header.stamp = get_clock()->now();
    static_transform_stamped_.header.frame_id = "bumperbot_base";
    static_transform_stamped_.child_frame_id = "bumperbot_top";
    static_transform_stamped_.transform.translation.x = 0.0;
    static_transform_stamped_.transform.translation.y = 0.0;
    static_transform_stamped_.transform.translation.z = 0.3;
    static_transform_stamped_.transform.rotation.x = 0.0;
    static_transform_stamped_.transform.rotation.y = 0.0;
    static_transform_stamped_.transform.rotation.z = 0.0;
    static_transform_stamped_.transform.rotation.w = 1.0; // No rotation

    // Send the static transform
    static_tf_broadcaster_->sendTransform(static_transform_stamped_);

    RCLCPP_INFO_STREAM(get_logger(), "Publishing static transform between "
        << static_transform_stamped_.header.frame_id << " and "
        << static_transform_stamped_.child_frame_id);

    // add timer callback to publish dynamic transform periodically at 10 Hz
    timer_ = this->create_wall_timer(
        0.1s,
        std::bind(&SimpleTfKinematics::timerCallback, this));

    get_transform_srv_ = this->create_service<bumperbot_msgs::srv::GetTransform>(
        "get_transform",
        std::bind(&SimpleTfKinematics::getTransformCallback, this,
            std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO_STREAM(get_logger(), "Service 'get_transform' is ready.");

    last_orientation_.setRPY(0.0, 0.0, 0.0);
    orientation_increment_.setRPY(0.0, 0.0, 0.05); // 0.05 radians per callback
    
}
void SimpleTfKinematics::timerCallback()
{
    // Define a dynamic transform from "odom" to "bumperbot_base"
    dynamic_transform_stamped_.header.stamp = get_clock()->now();
    dynamic_transform_stamped_.header.frame_id = "odom";
    dynamic_transform_stamped_.child_frame_id = "bumperbot_base";  
    // Movement of the bumperbot_base frame relative to the odom frame
    dynamic_transform_stamped_.transform.translation.x = last_x_ + x_increment_;  // Move forward at 1 m/s
    dynamic_transform_stamped_.transform.translation.y = 0.0;
    dynamic_transform_stamped_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q = last_orientation_ * orientation_increment_;
    q.normalize();
    dynamic_transform_stamped_.transform.rotation.x = q.x();
    dynamic_transform_stamped_.transform.rotation.y = q.y();
    dynamic_transform_stamped_.transform.rotation.z = q.z();
    dynamic_transform_stamped_.transform.rotation.w = q.w();
    dynamic_tf_broadcaster_->sendTransform(dynamic_transform_stamped_);
    last_x_  = dynamic_transform_stamped_.transform.translation.x;
    rotations_counter_++;
    last_orientation_ = q;

    if (rotations_counter_>= 100)
    {
        orientation_increment_ = orientation_increment_.inverse();
        rotations_counter_ = 0;
    }

}

bool SimpleTfKinematics::getTransformCallback(
    const std::shared_ptr<bumperbot_msgs::srv::GetTransform::Request> req,
    std::shared_ptr<bumperbot_msgs::srv::GetTransform::Response> res)
{
    RCLCPP_INFO_STREAM(get_logger(), "Received request for transform from '"
        << req->frame_id << "' to '" << req->child_frame_id << "'.");
    geometry_msgs::msg::TransformStamped requested_transform;
    try {
        requested_transform = 
            tf_buffer_->lookupTransform(
                req->frame_id,
                req->child_frame_id,
                tf2::TimePointZero);

        RCLCPP_INFO_STREAM(get_logger(), "Providing transform from '"
            << req->frame_id << "' to '" << req->child_frame_id << "'.");
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_STREAM(get_logger(), "Could not transform from '"
            << req->frame_id << "' to '" << req->child_frame_id
            << "': " << ex.what());

        res->transform = requested_transform;
        res->success = false;
        return true;
        // Leave res->transform default initialized
    }
    res->success = true;
    return true;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTfKinematics>("simple_tf_kinematics");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}