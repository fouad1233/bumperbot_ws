import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self, a , b):
        super().__init__('simple_service_client')
        self.client_ = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AddTwoInts.Request()
        self.request.a = a
        self.request.b = b

        self.future_ = self.client_.call_async(self.request)

        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info(f"Serive response add_two_ints: {future.result().sum}")



def main(args=None):
    rclpy.init(args=args)
    # The first arf is the name of the node
    if len(sys.argv) != 3:
        print("Wrong number of args!" \
        "Usage: ros2 run bumperbot_py_examples simple_service_client a b")
        return -1
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    simple_service_client = SimpleServiceClient(a, b)
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
