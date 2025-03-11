#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LEDPWMNode(Node):
    def __init__(self):
        super().__init__('LEDPWMNode')
        self.get_logger().info('Hello ROS 2 world!')

def main(args=None):
    rclpy.init(args=args)
    node = LEDPWMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
