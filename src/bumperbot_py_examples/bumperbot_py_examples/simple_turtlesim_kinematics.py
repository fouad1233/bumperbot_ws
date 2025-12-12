import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class SimpleTurtlesimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtlesim_kinematics')

        self.turtle1_pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.turtle1_pose_callback, 10)
        self.turtle2_pose_sub_ = self.create_subscription(Pose, "/turtle2/pose", self.turtle2_pose_callback, 10)

        self.last_turtle1_pose_ = Pose()
        self.last_turtle2_pose_ = Pose()

    def turtle1_pose_callback(self, msg):
        self.get_logger().info(f"Turtle1 Pose - x: {msg.x}, y: {msg.y}, theta: {msg.theta}")
        self.last_turtle1_pose_ = msg
    def turtle2_pose_callback(self, msg):
        self.get_logger().info(f"Turtle2 Pose - x: {msg.x}, y: {msg.y}, theta: {msg.theta}")
        self.last_turtle2_pose_ = msg
        Tx = self.last_turtle2_pose_.x - self.last_turtle1_pose_.x
        Ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y
        distance = (Tx**2 + Ty**2)**0.5
        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = 180.0 * theta_rad / 3.141592653589793
        self.get_logger().info("""\n
        Translation Vector from Turtle1 to Turtle2: \n
        Tx: {:.5f}, Ty: {:.5f} \n
        Distance: {:.5f} \n
        Rotation Matrix 
        Theta radians: {:.5f},
        theta deg: {:.5f}
        |R11    R12| :|{:.5f}   {:.5f}| :|cos(theta)  -sin(theta)|
        |R21    R22| :|{:.5f}   {:.5f}| :|sin(theta)   cos(theta)|
        """.format(Tx, Ty, distance, theta_rad, theta_deg,
                    math.cos(theta_rad), -math.sin(theta_rad),
                    math.sin(theta_rad), math.cos(theta_rad)))


def main(args=None):
    rclpy.init(args=args)
    simple_turtlesim_kinematics = SimpleTurtlesimKinematics()
    try:
        rclpy.spin(simple_turtlesim_kinematics)
    except KeyboardInterrupt:
        pass
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()