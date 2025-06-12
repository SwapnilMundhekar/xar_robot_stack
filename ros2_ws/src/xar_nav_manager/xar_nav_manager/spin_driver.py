import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
import time

class SpinDriver(Node):
    def __init__(self):
        super().__init__('spin_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/status_msgs', 10)
        self.create_subscription(Empty, '/trigger_left_spin', self.left_spin, 10)
        self.create_subscription(Empty, '/trigger_right_spin', self.right_spin, 10)

    def left_spin(self, msg):
        self.status_pub.publish(String(data='[You clicked Left Spin - So I will Spin 360° left]'))
        spin = Twist()
        spin.angular.z = 1.57
        self.cmd_pub.publish(spin)
        time.sleep(4)
        self.cmd_pub.publish(Twist())

    def right_spin(self, msg):
        self.status_pub.publish(String(data='[You clicked Right Spin - So I will do a Right 360° Spin]'))
        spin = Twist()
        spin.angular.z = -1.57
        self.cmd_pub.publish(spin)
        time.sleep(4)
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = SpinDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()