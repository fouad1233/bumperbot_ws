import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

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
        self.get_logger().info("""\n
        Translation Vector from Turtle1 to Turtle2: \n
        Tx: {:.5f}, Ty: {:.5f} \n
        Distance: {:.5f} \n
        """.format(Tx, Ty, distance))

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