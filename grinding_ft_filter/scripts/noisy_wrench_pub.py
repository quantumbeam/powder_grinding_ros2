#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import numpy as np
import random

class NoisyWrenchPublisher(Node):
    def __init__(self):
        super().__init__('noisy_wrench_publisher')
        self.publisher_ = self.create_publisher(WrenchStamped, '/wrench_raw', 10)
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Noisy wrench publisher started.')

    def timer_callback(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # ノイジーな力・トルク（±10N, ±1Nmくらいランダム）
        msg.wrench.force.x = 5.0 + random.uniform(-2, 2)
        msg.wrench.force.y = 0.0 + random.uniform(-0.5, 0.5)
        msg.wrench.force.z = 0.0 + random.uniform(-0.5, 0.5)

        msg.wrench.torque.x = 0.0 + random.uniform(-0.1, 0.1)
        msg.wrench.torque.y = 0.0 + random.uniform(-0.1, 0.1)
        msg.wrench.torque.z = 0.0 + random.uniform(-0.1, 0.1)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NoisyWrenchPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
