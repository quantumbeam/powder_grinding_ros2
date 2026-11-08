#!/usr/bin/env python3
"""
COBOTTA grinding demo with a simple Tkinter GUI.

ROS2 port of the ROS1 `grinding_motion_routines/scripts/cobotta_grinding_demo.py`.
MoveIt is replaced by JointTrajectoryControllerHelper (Trac-IK + joint_trajectory_controller).
"""

import sys
import threading
import tkinter as tk
from math import pi

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory

from grinding_motion_routines.grinding_motion_generator import MotionGenerator
from grinding_motion_routines.grinding_motion_primitive import GrindingMotionPrimitive
from grinding_robot_control.JTC_helper import JointTrajectoryControllerHelper, IKType


def declare_parameters(node):
    node.declare_parameter("controller_name", "arm_controller")
    node.declare_parameter("joint_names", [f"joint_{i}" for i in range(1, 7)])
    node.declare_parameter("base_link", "base_link")
    node.declare_parameter("grinding_ee_link", "pestle_tip")
    node.declare_parameter("gathering_ee_link", "spatula_tip")
    node.declare_parameter("robot_description_package", "grinding_robot_description")
    node.declare_parameter("robot_description_file_path", "/urdf/cobotta/cobotta_with_pestle.urdf")
    node.declare_parameter("ik_solver", "trac_ik")

    node.declare_parameter("mortar_top_position", [0.25, 0.0, 0.022])
    node.declare_parameter("mortar_inner_scale", [0.04, 0.04, 0.036])
    node.declare_parameter("init_pose_height", 0.05)

    for motion in ("grinding", "gathering"):
        node.declare_parameter(f"{motion}_pos_beginning_mm", [-8.0, 0.0])
        node.declare_parameter(f"{motion}_pos_end_mm", [-8.0, 0.0001])
        node.declare_parameter(f"{motion}_center_pos_mm", [0.0, 0.0])
        node.declare_parameter(f"{motion}_rz_beginning_mm", 36.0)
        node.declare_parameter(f"{motion}_rz_end_mm", 36.0)
        node.declare_parameter(f"{motion}_number_of_rotation", 5.0)
        node.declare_parameter(f"{motion}_angle_scale", 0.0)
        node.declare_parameter(f"{motion}_yaw_bias", 0.0)
        node.declare_parameter(f"{motion}_number_of_waypoints_per_circle", 50)
        node.declare_parameter(f"{motion}_sec_per_rotation", 5.0)
        node.declare_parameter(f"{motion}_joint_difference_limit_for_motion_planning", 0.03)

    node.declare_parameter("ee_change_to_gathering_sec", 8.0)
    node.declare_parameter("ee_change_to_grinding_sec", 8.0)
    node.declare_parameter("max_attempts", 100)


def compute_waypoints(node, motion_generator, motion):
    p = lambda name: node.get_parameter(f"{motion}_{name}").value
    return motion_generator.create_circular_waypoints(
        beginning_position=p("pos_beginning_mm"),
        end_position=p("pos_end_mm"),
        beginning_radius_z=p("rz_beginning_mm"),
        end_radius_z=p("rz_end_mm"),
        angle_scale=p("angle_scale"),
        yaw_bias=p("yaw_bias"),
        number_of_rotations=int(p("number_of_rotation")),
        number_of_waypoints_per_circle=int(p("number_of_waypoints_per_circle")),
        center_position=p("center_pos_mm"),
    )


def make_init_pose(position, yaw):
    quat = Rotation.from_euler("xyz", [pi, 0, yaw], degrees=False).as_quat()
    return list(position) + list(quat)


def main():
    rclpy.init(args=sys.argv)
    node = Node("cobotta_grinding_demo")
    declare_parameters(node)
    param = lambda name: node.get_parameter(name).value

    ################### motion generator ###################
    mortar_top = param("mortar_top_position")
    mortar_inner = param("mortar_inner_scale")
    mortar_top_pos = {"x": mortar_top[0], "y": mortar_top[1], "z": mortar_top[2]}
    mortar_inner_size = {"x": mortar_inner[0], "y": mortar_inner[1], "z": mortar_inner[2]}
    motion_gen = MotionGenerator(mortar_top_pos, mortar_inner_size)

    ################### motion executor ###################
    grinding_ee_link = param("grinding_ee_link")
    gathering_ee_link = param("gathering_ee_link")
    jtc_helper = JointTrajectoryControllerHelper(
        controller_name=param("controller_name"),
        joints_name=param("joint_names"),
        tool_link=grinding_ee_link,
        base_link=param("base_link"),
        robot_urdf_file_path=get_package_share_directory(param("robot_description_package"))
        + param("robot_description_file_path"),
        ik_solver=IKType.TRACK_IK if param("ik_solver") == "trac_ik" else IKType.BIO_IK,
        node_name="cobotta_jtc_helper",
    )

    ################### init pose ###################
    init_pos = [mortar_top[0], mortar_top[1], mortar_top[2] + param("init_pose_height")]
    grinding_init_pose = make_init_pose(init_pos, param("grinding_yaw_bias"))
    gathering_init_pose = make_init_pose(init_pos, param("gathering_yaw_bias"))

    primitive = GrindingMotionPrimitive(
        node=node,
        jtc_helper=jtc_helper,
        init_pose=grinding_init_pose,
        grinding_ee_link=grinding_ee_link,
        gathering_ee_link=gathering_ee_link,
        gathering_init_pose=gathering_init_pose,
        ee_change_to_gathering_sec=param("ee_change_to_gathering_sec"),
        ee_change_to_grinding_sec=param("ee_change_to_grinding_sec"),
    )

    # NOTE: JTC helper spins itself while waiting for joint_states / action results,
    # so it must not be added to another executor here.
    node.get_logger().info("Goto init pose")
    primitive.go_to_init_pose()

    max_attempts = int(param("max_attempts"))

    def is_at_grinding_init_pose(pos_tol=0.001, rot_tol=0.01):
        # Skip the pre-motion to the init pose when the pestle is already there
        current_joints = jtc_helper.get_current_joint_positions()
        if current_joints is None:
            return False
        current_pose = jtc_helper.solve_fk(current_joints)
        if current_pose is None:
            return False
        pos_err = np.linalg.norm(np.array(current_pose[:3]) - np.array(grinding_init_pose[:3]))
        rot_err = (Rotation.from_quat(current_pose[3:7]).inv() * Rotation.from_quat(grinding_init_pose[3:])).magnitude()
        return pos_err < pos_tol and rot_err < rot_tol

    def send_grinding_command():
        primitive.execute_grinding(
            pre_motion=not is_at_grinding_init_pose(),
            waypoints=compute_waypoints(node, motion_gen, "grinding"),
            grinding_sec=param("grinding_sec_per_rotation") * param("grinding_number_of_rotation"),
            joint_difference_limit=param("grinding_joint_difference_limit_for_motion_planning"),
            max_ik_retries_on_jump=max_attempts,
            wait_for_completion=True,
        )

    def send_gathering_command():
        primitive.execute_gathering(
            waypoints=compute_waypoints(node, motion_gen, "gathering"),
            gathering_sec=param("gathering_sec_per_rotation") * param("gathering_number_of_rotation"),
            joint_difference_limit=param("gathering_joint_difference_limit_for_motion_planning"),
            max_ik_retries_on_jump=max_attempts,
            wait_for_completion=True,
        )

    def send_grinding_and_gathering_command():
        send_grinding_command()
        send_gathering_command()

    ################### GUI ###################
    root = tk.Tk()
    font = ("Meiryo", 20)
    root.title("ROS Commands")
    motion_buttons = []

    def run_in_background(command):
        # Run the motion in a worker thread so that the GUI does not freeze
        def worker():
            try:
                command()
            except Exception as e:
                node.get_logger().error(f"Motion failed: {e}")
            finally:
                root.after(0, lambda: [b.config(state=tk.NORMAL) for b in motion_buttons])

        for b in motion_buttons:
            b.config(state=tk.DISABLED)
        threading.Thread(target=worker, daemon=True).start()

    def exit_process():
        node.get_logger().info("Exit grinding demo")
        root.destroy()

    for text, command in (
        ("Grinding", send_grinding_command),
        ("Gathering", send_gathering_command),
        ("Grinding & Gathering", send_grinding_and_gathering_command),
    ):
        button = tk.Button(root, text=text, command=lambda c=command: run_in_background(c), font=font)
        button.pack(pady=5)
        motion_buttons.append(button)

    tk.Button(root, text="Exit demo", command=exit_process, font=font).pack(pady=5)
    root.protocol("WM_DELETE_WINDOW", exit_process)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        jtc_helper.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
