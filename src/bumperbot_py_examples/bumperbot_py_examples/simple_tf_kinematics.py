import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__('simple_tf_kinematics')

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)

        self.static_transform_stamped_ = TransformStamped()

        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"

        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3

        # These are quaternion values not Euler angles because tf2 uses quaternions
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0 # No rotation

        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)
        self.get_logger().info("Static transform from 'bumperbot_base' to 'bumperbot_top' broadcasted.") 

def main(args=None):
    rclpy.init(args=args)

    simple_tf_kinematics_node = SimpleTfKinematics()

    rclpy.spin(simple_tf_kinematics_node)

    simple_tf_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()