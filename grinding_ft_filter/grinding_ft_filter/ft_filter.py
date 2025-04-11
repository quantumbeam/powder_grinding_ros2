import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty

import numpy as np
from collections import deque
from grinding_motion_routines import conversions, filters


class FTFilterNode(Node):
    def __init__(self):
        super().__init__('ft_filter')

        # Declare parameters
        self.declare_parameter('input_topic', '/wrench_raw')
        self.declare_parameter('output_topic', '/wrench_filtered')
        self.declare_parameter('sampling_frequency', 500.0)
        self.declare_parameter('cutoff_frequency', 2.5)
        self.declare_parameter('filter_order', 3)
        self.declare_parameter('data_window', 100)

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.cutoff_frequency = self.get_parameter('cutoff_frequency').get_parameter_value().double_value
        self.filter_order = self.get_parameter('filter_order').get_parameter_value().integer_value
        self.data_window = self.get_parameter('data_window').get_parameter_value().integer_value

        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")

        # Data queue for filtering
        self.data_queue = deque(maxlen=self.data_window)

        # Filtering utility
        self.filter = filters.ButterLowPass(self.cutoff_frequency, self.sampling_frequency, self.filter_order)

        # Offset for calibration
        self.wrench_offset = np.zeros(6)

        # Subscribers and publishers
        self.subscription = self.create_subscription(WrenchStamped, self.input_topic, self.wrench_callback, 10)
        self.publisher = self.create_publisher(WrenchStamped, self.output_topic, 10)

        # Zeroing service
        self.zero_service = self.create_service(Empty, self.output_topic + '/zero_ftsensor', self.handle_zero)

    def wrench_callback(self, msg):
        wrench_np = conversions.from_wrench(msg.wrench)
        self.data_queue.append(wrench_np)

        if len(self.data_queue) < self.data_window:
            return

        filtered_data = self.filter(np.array(self.data_queue))[-1] - self.wrench_offset
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        filtered_msg.wrench = conversions.to_wrench(filtered_data)
        self.publisher.publish(filtered_msg)

    def handle_zero(self, request, response):
        if len(self.data_queue) < self.data_window:
            self.get_logger().warn("Not enough data to compute offset.")
            return response
        filtered_data = self.filter(np.array(self.data_queue))[-1]
        self.wrench_offset = filtered_data
        self.get_logger().info("FT sensor zeroed (offset updated).")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FTFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()