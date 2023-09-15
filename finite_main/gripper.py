import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Gripper(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'add_two_int', self.this_callback)
    
    def this_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def main():
    rclpy.init()
    Gripper = Gripper()
    rclpy.spin(Gripper)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()