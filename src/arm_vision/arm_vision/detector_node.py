#!/usr/bin/env python3
"""方块检测节点：订阅图像，识别俄罗斯方块，发布抓取位姿"""
import rclpy
from rclpy.node import Node

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector')
        self.get_logger().info('Vision detector node ready (dummy)')

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
