#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import WrenchStamped, Wrench, Vector3
from std_srvs.srv import Empty

import numpy as np
from collections import deque
import scipy.signal

# ========== Conversions (ベタ書き) ==========
def from_wrench(msg):
    array = np.zeros(6)
    array[:3] = [msg.force.x, msg.force.y, msg.force.z]
    array[3:] = [msg.torque.x, msg.torque.y, msg.torque.z]
    return array

def to_wrench(array):
    msg = Wrench()
    msg.force = Vector3(x=array[0], y=array[1], z=array[2])
    msg.torque = Vector3(x=array[3], y=array[4], z=array[5])
    return msg

# ========== Filters (ベタ書き: ButterLowPass class のみ) ==========
class ButterLowPass:
    def __init__(self, cutoff, fs, order=5):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = scipy.signal.butter(order, normal_cutoff, btype="low", analog=False)

    def __call__(self, x):
        if not hasattr(self, "zi"):
            cols = x.shape[1]
            zi = scipy.signal.lfiltic(self.b, self.a, []).tolist() * cols
            self.zi = np.array(scipy.signal.lfiltic(self.b, self.a, []).tolist() * cols)
            self.zi.shape = (-1, cols)
        (filtered, self.zi) = scipy.signal.lfilter(self.b, self.a, x, zi=self.zi, axis=0)
        return filtered

# ========== FTFilterNode ==========
class FTFilterNode(Node):
    def __init__(self):
        super().__init__('ft_filter')

        self.declare_parameter('input_topic', '/wrench_raw')
        self.declare_parameter('output_topic', '/wrench_filtered')
        self.declare_parameter('sampling_frequency', 500.0)
        self.declare_parameter('cutoff_frequency', 2.5)
        self.declare_parameter('filter_order', 3)
        self.declare_parameter('data_window', 100)
        self.declare_parameter('initial_zero', True)
        self.declare_parameter('disable_filtering', False)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.cutoff_frequency = self.get_parameter('cutoff_frequency').get_parameter_value().double_value
        self.filter_order = self.get_parameter('filter_order').get_parameter_value().integer_value
        self.data_window = self.get_parameter('data_window').get_parameter_value().integer_value
        self.initial_zero = self.get_parameter('initial_zero').get_parameter_value().bool_value
        self.disable_filtering = self.get_parameter('disable_filtering').get_parameter_value().bool_value


        self.get_logger().info(f"Subscribing to: {self.input_topic}")
        self.get_logger().info(f"Publishing to: {self.output_topic}")

        self.data_queue = deque(maxlen=self.data_window)
        self.filter = ButterLowPass(self.cutoff_frequency, self.sampling_frequency, self.filter_order)
        self.wrench_offset = np.zeros(6)

        self.subscription = self.create_subscription(WrenchStamped, self.input_topic, self.wrench_callback, 10)
        self.publisher = self.create_publisher(WrenchStamped, self.output_topic, 10)
        self.zero_service = self.create_service(Empty, self.output_topic + '/zero_ftsensor', self.handle_zero)

        # 自動ゼロ初期化
        if self.initial_zero:
            self.zero_initialized = False
            self.timer = self.create_timer(1.0, self._check_and_zero_init)
        else:
            self.zero_initialized = True  # 無効化するなら初期化済みにしておく

    def wrench_callback(self, msg):
        wrench_np = from_wrench(msg.wrench)
        self.data_queue.append(wrench_np)

        if len(self.data_queue) < self.data_window:
            return
        
        if self.disable_filtering:
            filtered_data = wrench_np - self.wrench_offset
        else:
            filtered_data = self.filter(np.array(self.data_queue))[-1] - self.wrench_offset

        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        filtered_msg.wrench = to_wrench(filtered_data)
        self.publisher.publish(filtered_msg)

    def handle_zero(self, request, response):
        if len(self.data_queue) < self.data_window:
            self.get_logger().warn("Not enough data to compute offset.")
            return response
        filtered_data = self.filter(np.array(self.data_queue))[-1]
        self.wrench_offset = filtered_data
        self.get_logger().info("FT sensor zeroed (offset updated).")
        return response
    
    def _check_and_zero_init(self):
        if not self.zero_initialized and len(self.data_queue) >= self.data_window:
            filtered_data = self.filter(np.array(self.data_queue))[-1]
            self.wrench_offset = filtered_data
            self.get_logger().info("Auto zeroing complete.")
            self.zero_initialized = True
            self.timer.cancel()  # 以降このタイマーは不要なので停止
   


def main(args=None):
    rclpy.init(args=args)
    node = FTFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
