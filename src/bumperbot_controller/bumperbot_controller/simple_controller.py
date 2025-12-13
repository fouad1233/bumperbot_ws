#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_seperation', 0.17)

        self.wheel_raduis_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_seperation_ = self.get_parameter('wheel_seperation').get_parameter_value().double_value

        self.get_logger().info(f'Using Wheel Radius: {self.wheel_raduis_},\n \
                                Wheel Separation: {self.wheel_seperation_}')
        # Publisher for wheel veolocity commands
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        # Subscriber for commands coming from the joystick
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10)

        self.speed_conversion_ = np.array([[self.wheel_raduis_/2, self.wheel_raduis_/2],
                                           [self.wheel_raduis_/self.wheel_seperation_, -self.wheel_raduis_/self.wheel_seperation_]])
        
        """
        The equation is as follows:
        [V]   :   [ r/2        r/2         ]    [theta_r_dot]
        [W]   :   [ r/W_s       -r/W_s     ]    [theta_l_dot]
    |_______|  |_____________________________| |___________]
Robot speed R_s Speed Conversion Matrix M_s    Wheel Angular Velocities theta_dot
        Where:
        V = Linear Velocity
        W = Angular Velocity
        r = Wheel Radius
        W_s = Wheel Separation
        theta_r_dot = Right Wheel Angular Velocity
        theta_l_dot = Left Wheel Angular Velocity
        """

        self.get_logger().info('the conversion matrix is: %s' % str(self.speed_conversion_))
        
    def velCallback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        # theta_dot = M_s_inv * [V; W]
        wheel_speed = np.linalg.inv(self.speed_conversion_).dot(robot_speed)
        wheel_speed_msg = Float64MultiArray()
        # The first element is the left wheel, the second is the right wheel
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        pass
 
def main(args=None):
    rclpy.init(args=args)
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown() 
if __name__ == '__main__':
    main()        