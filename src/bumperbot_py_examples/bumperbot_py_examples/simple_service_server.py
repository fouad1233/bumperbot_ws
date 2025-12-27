import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
"""
How to use this server from command line

colcon build
In different terminal run the server
. install/setup.bash 
ros2 run bumperbot_py_examples simple_service_server 
In different terminal call the server
. install/setup.bash 

ros2 service call /add_two_ints bumperbot_msgs/srv/AddTwoInts 'a: 2
b: 3' 


"""

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.service = self.create_service(AddTwoInts, "add_two_ints", 
                                           self.serviceCallback)
        
        self.get_logger().info("Service add_two_ints ready")

    def serviceCallback(self, req, res):
        self.get_logger().info(f"New request received: a={req.a}, b={req.b}")
        res.sum = req.a + req.b
        self.get_logger().info("Returning sum: %d" %res.sum)
        return res

def main():
    rclpy.init()
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()