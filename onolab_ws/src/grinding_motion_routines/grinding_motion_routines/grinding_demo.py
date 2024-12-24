import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose, Point
from tf_transformations import quaternion_from_euler, quaternion_slerp


from numpy import pi
from math import tau, dist, fabs, cos, sin, sqrt
from cmath import nan
import sys
import copy
import numpy as np

# from ur_pykdl import ur_kinematics
from grinding_motion_routines.helpers import *
from grinding_motion_routines.marker_display import MarkerDisplay


class GrindingDemo(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Declare parameters
        self.declare_parameter("experiment_time", 300)
        self.declare_parameter("pouse_time_list", [30, 60, 90, 120, 150, 180, 210, 240, 270, 300])
        self.declare_parameter("grinding_ee_link", "pestle_tip")
        self.declare_parameter("gathering_ee_link", "spatula_tip")
        self.declare_parameter("joint_trajectory_controller_name", "scaled_pos_joint_traj_controller")
        self.declare_parameter("urdf_name", "ur5e")
        self.declare_parameter("ik_solver", "trac_ik")
        self.declare_parameter("motion_planner_id", "RRTConnect")
        self.declare_parameter("planning_time", 20)
        self.declare_parameter("max_attempts", 100)
        self.declare_parameter("grinding_pos_begining", [-8, 0])
        self.declare_parameter("grinding_pos_end", [-8, 0.0001])
        self.declare_parameter("grinding_center_pos", [0, 0])
        self.declare_parameter("grinding_number_of_rotation", 20)
        self.declare_parameter("grinding_sec_per_rotation", 0.5)
        self.declare_parameter("grinding_number_of_waypoints_per_circle", 50)
        self.declare_parameter("grinding_angle_scale", 0.3)
        self.declare_parameter("grinding_rz_begining", 36)
        self.declare_parameter("grinding_rz_end", 36)
        self.declare_parameter("grinding_vel_scale", 1)
        self.declare_parameter("grinding_acc_scale", 1)
        self.declare_parameter("grinding_yaw_bias", 3.14159)  # rad(pi)
        self.declare_parameter("grinding_joint_difference_limit_for_motion_planning", 0.03)
        self.declare_parameter("gathering_pos_begining", [30, 0])
        self.declare_parameter("gathering_pos_end", [-22, 0.0001])
        self.declare_parameter("gathering_number_of_rotation", 5)
        self.declare_parameter("gathering_sec_per_rotation", 2)
        self.declare_parameter("gathering_angle_scale", 0)
        self.declare_parameter("gathering_rz_begining", 35)
        self.declare_parameter("gathering_rz_end", 35)
        self.declare_parameter("gathering_vel_scale", 0.1)
        self.declare_parameter("gathering_acc_scale", 0.1)
        self.declare_parameter("gathering_number_of_waypoints_per_circle", 100)
        self.declare_parameter("gathering_yaw_bias", 3.14159)  # rad(pi)
        self.declare_parameter("gathering_joint_difference_limit_for_motion_planning", 0.03)

        # Get parameters
        experiment_time = self.get_parameter("experiment_time").get_parameter_value().integer_value
        pouse_time_list = self.get_parameter("pouse_time_list").get_parameter_value().double_array_value
        grinding_ee_link = self.get_parameter("grinding_ee_link").get_parameter_value().string_value
        gathering_ee_link = self.get_parameter("gathering_ee_link").get_parameter_value().string_value
        joint_trajectory_controller_name = self.get_parameter("joint_trajectory_controller_name").get_parameter_value().string_value
        urdf_name = self.get_parameter("urdf_name").get_parameter_value().string_value
        ik_solver = self.get_parameter("ik_solver").get_parameter_value().string_value
        motion_planner_id = self.get_parameter("motion_planner_id").get_parameter_value().string_value
        planning_time = self.get_parameter("planning_time").get_parameter_value().integer_value
        max_attempts = self.get_parameter("max_attempts").get_parameter_value().integer_value
        grinding_pos_begining = self.get_parameter("grinding_pos_begining").get_parameter_value().double_array_value
        grinding_pos_end = self.get_parameter("grinding_pos_end").get_parameter_value().double_array_value
        grinding_center_pos = self.get_parameter("grinding_center_pos").get_parameter_value().double_array_value
        grinding_number_of_rotation = self.get_parameter("grinding_number_of_rotation").get_parameter_value().integer_value
        grinding_sec_per_rotation = self.get_parameter("grinding_sec_per_rotation").get_parameter_value().double_value
        grinding_number_of_waypoints_per_circle = self.get_parameter("grinding_number_of_waypoints_per_circle").get_parameter_value().integer_value
        grinding_angle_scale = self.get_parameter("grinding_angle_scale").get_parameter_value().double_value
        grinding_rz_begining = self.get_parameter("grinding_rz_begining").get_parameter_value().double_value
        grinding_rz_end = self.get_parameter("grinding_rz_end").get_parameter_value().double_value
        grinding_vel_scale = self.get_parameter("grinding_vel_scale").get_parameter_value().double_value
        grinding_acc_scale = self.get_parameter("grinding_acc_scale").get_parameter_value().double_value
        grinding_yaw_bias = self.get_parameter("grinding_yaw_bias").get_parameter_value().double_value
        grinding_joint_difference_limit_for_motion_planning = self.get_parameter("grinding_joint_difference_limit_for_motion_planning").get_parameter_value().double_value
        gathering_pos_begining = self.get_parameter("gathering_pos_begining").get_parameter_value().double_array_value
        gathering_pos_end = self.get_parameter("gathering_pos_end").get_parameter_value().double_array_value
        gathering_number_of_rotation = self.get_parameter("gathering_number_of_rotation").get_parameter_value().integer_value
        gathering_sec_per_rotation = self.get_parameter("gathering_sec_per_rotation").get_parameter_value().double_value
        gathering_angle_scale = self.get_parameter("gathering_angle_scale").get_parameter_value().double_value
        gathering_rz_begining = self.get_parameter("gathering_rz_begining").get_parameter_value().double_value
        gathering_rz_end = self.get_parameter("gathering_rz_end").get_parameter_value().double_value
        gathering_vel_scale = self.get_parameter("gathering_vel_scale").get_parameter_value().double_value
        gathering_acc_scale = self.get_parameter("gathering_acc_scale").get_parameter_value().double_value
        gathering_number_of_waypoints_per_circle = self.get_parameter("gathering_number_of_waypoints_per_circle").get_parameter_value().integer_value
        gathering_yaw_bias = self.get_parameter("gathering_yaw_bias").get_parameter_value().double_value
        gathering_joint_difference_limit_for_motion_planning = self.get_parameter("gathering_joint_difference_limit_for_motion_planning").get_parameter_value().double_value

        # Output parameters
        self.get_logger().info(f"experiment_time: {experiment_time}")
        self.get_logger().info(f"pouse_time_list: {pouse_time_list}")
        self.get_logger().info(f"grinding_ee_link: {grinding_ee_link}")
        self.get_logger().info(f"gathering_ee_link: {gathering_ee_link}")
        self.get_logger().info(f"joint_trajectory_controller_name: {joint_trajectory_controller_name}")
        self.get_logger().info(f"urdf_name: {urdf_name}")
        self.get_logger().info(f"ik_solver: {ik_solver}")
        self.get_logger().info(f"motion_planner_id: {motion_planner_id}")
        self.get_logger().info(f"planning_time: {planning_time}")
        self.get_logger().info(f"max_attempts: {max_attempts}")
        self.get_logger().info(f"grinding_pos_begining: {grinding_pos_begining}")
        self.get_logger().info(f"grinding_pos_end: {grinding_pos_end}")
        self.get_logger().info(f"grinding_center_pos: {grinding_center_pos}")
        self.get_logger().info(f"grinding_number_of_rotation: {grinding_number_of_rotation}")
        self.get_logger().info(f"grinding_sec_per_rotation: {grinding_sec_per_rotation}")
        self.get_logger().info(f"grinding_number_of_waypoints_per_circle: {grinding_number_of_waypoints_per_circle}")
        self.get_logger().info(f"grinding_angle_scale: {grinding_angle_scale}")
        self.get_logger().info(f"grinding_rz_begining: {grinding_rz_begining}")
        self.get_logger().info(f"grinding_rz_end: {grinding_rz_end}")
        self.get_logger().info(f"grinding_vel_scale: {grinding_vel_scale}")
        self.get_logger().info(f"grinding_acc_scale: {grinding_acc_scale}")
        self.get_logger().info(f"grinding_yaw_bias: {grinding_yaw_bias}")
        self.get_logger().info(f"grinding_joint_difference_limit_for_motion_planning: {grinding_joint_difference_limit_for_motion_planning}")
        self.get_logger().info(f"gathering_pos_begining: {gathering_pos_begining}")
        self.get_logger().info(f"gathering_pos_end: {gathering_pos_end}")
        self.get_logger().info(f"gathering_number_of_rotation: {gathering_number_of_rotation}")
        self.get_logger().info(f"gathering_sec_per_rotation: {gathering_sec_per_rotation}")
        self.get_logger().info(f"gathering_angle_scale: {gathering_angle_scale}")
        self.get_logger().info(f"gathering_rz_begining: {gathering_rz_begining}")
        self.get_logger().info(f"gathering_rz_end: {gathering_rz_end}")
        self.get_logger().info(f"gathering_vel_scale: {gathering_vel_scale}")
        self.get_logger().info(f"gathering_acc_scale: {gathering_acc_scale}")
        self.get_logger().info(f"gathering_number_of_waypoints_per_circle: {gathering_number_of_waypoints_per_circle}")
        self.get_logger().info(f"gathering_yaw_bias: {gathering_yaw_bias}")
        self.get_logger().info(f"gathering_joint_difference_limit_for_motion_planning: {gathering_joint_difference_limit_for_motion_planning}")

def main():
    # if len(sys.argv) == 1:
    #     print("You need to set xml_file file path to arg1")
    #     exit()
    # else:
    #     URDF_path = sys.argv[1]

    rclpy.init()
    rclpy.create_node("create_waypoints_node")
    # kin = ur_kinematics(URDF_path, base_link="base_link", ee_link="tool0")

    grinding_waypoints = GrindingWaypoints("create_waypoints_node")
    waypoints = grinding_waypoints.create_circular_waypoints(debug=False)
    print(len(waypoints))
    pose_list = [waypoints[i].position for i in range(len(waypoints))]
    for p in pose_list:
        print(p.x, p.y, p.z)

    marker_display = MarkerDisplay("marker_publisher")
    marker_display.display_waypoints(waypoints, scale=0.01)

    rclpy.spin(marker_display)
    grinding_waypoints.destroy_node()
    marker_display.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
