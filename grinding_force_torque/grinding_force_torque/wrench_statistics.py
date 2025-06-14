#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from  grinding_force_torque.srv import WrenchStatistics
import numpy as np


class WrenchStatisticsNode(Node):
    def __init__(self):
        super().__init__("wrench_statistics")

        self.is_recording = False
        self.forces_x = []
        self.forces_y = []
        self.forces_z = []
        self.torques_x = []
        self.torques_y = []
        self.torques_z = []

        # Declare parameters
        self.declare_parameter("wrench_topic", "/wrench_raw")
        self.declare_parameter("wrench_statistics_service", "/wrench_statistics")

        wrench_topic = self.get_parameter("wrench_topic").get_parameter_value().string_value
        service_name = self.get_parameter("wrench_statistics_service").get_parameter_value().string_value

        self.create_subscription(WrenchStamped, wrench_topic, self.wrench_callback, 10)
        self.create_service(WrenchStatistics, service_name, self.handle_start_stop)

        self.get_logger().info("WrenchStatisticsNode started.")

    def wrench_callback(self, msg):
        if self.is_recording:
            self.get_logger().info("Recording incoming wrench message.")  # ← 追加！
            self.forces_x.append(msg.wrench.force.x)
            self.forces_y.append(msg.wrench.force.y)
            self.forces_z.append(msg.wrench.force.z)
            self.torques_x.append(msg.wrench.torque.x)
            self.torques_y.append(msg.wrench.torque.y)
            self.torques_z.append(msg.wrench.torque.z)

    def handle_start_stop(self, request, response):
        self.get_logger().info(f"Request: {request.command}")

        if request.command == "start":
            self.is_recording = True
            self.clear_data()
            response.success = True
            response.average_x = response.average_y = response.average_z = 0.0
            response.variance_x = response.variance_y = response.variance_z = 0.0
            response.average_tx = response.average_ty = response.average_tz = 0.0
            response.variance_tx = response.variance_ty = response.variance_tz = 0.0
        elif request.command == "stop":
            self.is_recording = False
            (ax, ay, az, vx, vy, vz,
             tx, ty, tz, vtx, vty, vtz) = self.compute_statistics()
            response.success = True
            response.average_x = ax
            response.average_y = ay
            response.average_z = az
            response.variance_x = vx
            response.variance_y = vy
            response.variance_z = vz
            response.average_tx = tx
            response.average_ty = ty
            response.average_tz = tz
            response.variance_tx = vtx
            response.variance_ty = vty
            response.variance_tz = vtz
        else:
            self.get_logger().warn("Unknown command received.")
            response.success = False

        return response

    def compute_statistics(self):
        # Force
        avg_fx = float(np.average(self.forces_x)) if self.forces_x else 0.0
        avg_fy = float(np.average(self.forces_y)) if self.forces_y else 0.0
        avg_fz = float(np.average(self.forces_z)) if self.forces_z else 0.0
        var_fx = float(np.var(self.forces_x)) if self.forces_x else 0.0
        var_fy = float(np.var(self.forces_y)) if self.forces_y else 0.0
        var_fz = float(np.var(self.forces_z)) if self.forces_z else 0.0

        # Torque
        avg_tx = float(np.average(self.torques_x)) if self.torques_x else 0.0
        avg_ty = float(np.average(self.torques_y)) if self.torques_y else 0.0
        avg_tz = float(np.average(self.torques_z)) if self.torques_z else 0.0
        var_tx = float(np.var(self.torques_x)) if self.torques_x else 0.0
        var_ty = float(np.var(self.torques_y)) if self.torques_y else 0.0
        var_tz = float(np.var(self.torques_z)) if self.torques_z else 0.0

        self.get_logger().info(f"Force Averages:  x={avg_fx:.3f}, y={avg_fy:.3f}, z={avg_fz:.3f}")
        self.get_logger().info(f"Force Variances: x={var_fx:.3f}, y={var_fy:.3f}, z={var_fz:.3f}")
        self.get_logger().info(f"Torque Averages:  x={avg_tx:.3f}, y={avg_ty:.3f}, z={avg_tz:.3f}")
        self.get_logger().info(f"Torque Variances: x={var_tx:.3f}, y={var_ty:.3f}, z={var_tz:.3f}")

        return (avg_fx, avg_fy, avg_fz, var_fx, var_fy, var_fz,
                avg_tx, avg_ty, avg_tz, var_tx, var_ty, var_tz)

    def clear_data(self):
        self.forces_x.clear()
        self.forces_y.clear()
        self.forces_z.clear()
        self.torques_x.clear()
        self.torques_y.clear()
        self.torques_z.clear()


def main(args=None):
    rclpy.init(args=args)
    node = WrenchStatisticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
