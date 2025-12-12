#include "bumperbot_cpp_examples/simple_turtlesim_kinematics.hpp"

using std::placeholders::_1; // one variable as input

SimpleTurtlesimKinematics::SimpleTurtlesimKinematics(const std::string &name ): Node(name)
{
    turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&SimpleTurtlesimKinematics::turtle1PoseCallback, this, _1)
    );

    turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle2/pose", 10,
        std::bind(&SimpleTurtlesimKinematics::turtle2PoseCallback, this, _1)
    );

}


void SimpleTurtlesimKinematics::turtle1PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle1_pose_ = pose;
    //RCLCPP_INFO(this->get_logger(), "Turtle1 Pose - x: %.2f, y: %.2f, theta: %.2f", 
    //            pose.x, pose.y, pose.theta);
}

void SimpleTurtlesimKinematics::turtle2PoseCallback(const turtlesim::msg::Pose &pose)
{
    last_turtle2_pose_ = pose;
    //RCLCPP_INFO(this->get_logger(), "Turtle2 Pose - x: %.2f, y: %.2f, theta: %.2f", 
    //            pose.x, pose.y, pose.theta);
    float Tx, Ty, distance;
    Tx = last_turtle2_pose_.x - last_turtle1_pose_.x;
    Ty = last_turtle2_pose_.y - last_turtle1_pose_.y;
    distance = sqrt(Tx*Tx + Ty*Ty);
    float theta_rad = last_turtle2_pose_.theta - last_turtle1_pose_.theta;
    float theta_deg = 180.0 * theta_rad / 3.141592653589793;
    RCLCPP_INFO_STREAM(this->get_logger(), 
        "Relative Orientation (Turtle2 wrt Turtle1): " << theta_deg << " degrees\n"
        << "Rotation Matrix: |R11    R12| :|" << std::cos(theta_rad) << "   " << -std::sin(theta_rad) << "| :|cos(theta)  -sin(theta)|\n"
        << "                 |R21    R22| :|" << std::sin(theta_rad) << "   " << std::cos(theta_rad)  << "| :|sin(theta)   cos(theta)|\n"
        << "Translation vector Tx: " << Tx << ", Ty: " << Ty << "\n"
        << "Distance between Turtle1 and Turtle2: " << distance);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto simple_turtlesim_kinematics = std::make_shared<SimpleTurtlesimKinematics>("simple_turtlesim_kinematics");
    rclcpp::spin(simple_turtlesim_kinematics);
    rclcpp::shutdown();
    return 0;
}
