#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import csv
from inputimeout import inputimeout, TimeoutOccurred
import threading

from math import pi
import copy
from scipy.spatial.transform import Rotation
import numpy as np
from ament_index_python.packages import get_package_share_directory


from grinding_motion_routines.grinding_motion_generator import MotionGenerator
from grinding_motion_routines.grinding_motion_primitive import GrindingMotionPrimitive
from grinding_motion_routines.marker_publisher import MarkerPublisher
from grinding_motion_routines.pose_publisher import PosePublisher
from grinding_robot_control.JTC_helper import JointTrajectoryControllerHelper, IKType

################### Fixed params ###################

# Global variables
debug_marker = None
debug_pose = None
node = None
jtc_helper = None
jtc_executor = None
jtc_thread = None

def display_debug_waypoints(waypoints, debug_type, tf_name="debug"):
    global debug_marker, debug_pose, node
    if debug_type == "mk":
        if node:
            node.get_logger().info("Display waypoints marker")
        if debug_marker:
            debug_marker.display_waypoints(waypoints, clear=True)
    elif debug_type == "pose":
        if node:
            node.get_logger().info("Display waypoints poses")
        if debug_pose:
            debug_pose.publish_poses_from_waypoints(waypoints)

def compute_grinding_waypoints(motion_generator, debug_type=False):
    global node
    waypoints = motion_generator.create_circular_waypoints(
        beginning_position=node.get_parameter("grinding_pos_beginning_mm").value,
        end_position=node.get_parameter("grinding_pos_end_mm").value,
        beginning_radius_z=node.get_parameter("grinding_rz_beginning_mm").value,
        end_radius_z=node.get_parameter("grinding_rz_end_mm").value,
        angle_scale=node.get_parameter("grinding_angle_scale").value,
        yaw_bias=node.get_parameter("grinding_yaw_bias").value,
        number_of_rotations=int(node.get_parameter("grinding_number_of_rotation").value),
        number_of_waypoints_per_circle=int(node.get_parameter("grinding_number_of_waypoints_per_circle").value),
        center_position=node.get_parameter("grinding_center_pos_mm").value,
    )
    if debug_type != False:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints

def compute_gathering_waypoints(motion_generator, debug_type=False):
    global node
    waypoints = motion_generator.create_circular_waypoints(
        beginning_position=node.get_parameter("gathering_pos_beginning_mm").value,
        end_position=node.get_parameter("gathering_pos_end_mm").value,
        beginning_radius_z=node.get_parameter("gathering_rz_beginning_mm").value,
        end_radius_z=node.get_parameter("gathering_rz_end_mm").value,
        angle_scale=node.get_parameter("gathering_angle_scale").value,
        yaw_bias=node.get_parameter("gathering_yaw_bias").value,
        number_of_rotations=int(node.get_parameter("gathering_number_of_rotation").value),
        number_of_waypoints_per_circle=int(node.get_parameter("gathering_number_of_waypoints_per_circle").value),
        center_position=node.get_parameter("gathering_center_pos_mm").value,
    )
    if debug_type != False:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints


def display_grinding_parameters():
    """Display grinding parameters"""
    global node
    if not node:
        return
    
    print("\n=== Grinding Parameters ===")
    print("--- Position Parameters (in mm) ---")
    print(f"Beginning position (grinding_pos_beginning_mm): {node.get_parameter('grinding_pos_beginning_mm').value}")
    print(f"End position (grinding_pos_end_mm): {node.get_parameter('grinding_pos_end_mm').value}")
    print(f"Center position (grinding_center_pos_mm): {node.get_parameter('grinding_center_pos_mm').value}")
    print(f"Beginning radius Z (grinding_rz_beginning_mm): {node.get_parameter('grinding_rz_beginning_mm').value}")
    print(f"End radius Z (grinding_rz_end_mm): {node.get_parameter('grinding_rz_end_mm').value}")
    
    print("\n--- Motion Parameters ---")
    print(f"Number of rotations (grinding_number_of_rotation): {node.get_parameter('grinding_number_of_rotation').value}")
    print(f"Waypoints per circle (grinding_number_of_waypoints_per_circle): {node.get_parameter('grinding_number_of_waypoints_per_circle').value}")
    print(f"Time per rotation (grinding_sec_per_rotation): {node.get_parameter('grinding_sec_per_rotation').value}")
    
    print("\n--- Orientation Parameters ---")
    print(f"Angle scale (grinding_angle_scale): {node.get_parameter('grinding_angle_scale').value}")
    print(f"Yaw bias (grinding_yaw_bias): {node.get_parameter('grinding_yaw_bias').value}")
    
    print("\n--- Mortar Information ---")
    print(f"Mortar top position (mortar_top_position): {node.get_parameter('mortar_top_position').value}")
    print(f"Mortar inner scale (mortar_inner_scale): {node.get_parameter('mortar_inner_scale').value}")
    
    print("\n--- Other Parameters ---")
    print(f"End effector link (grinding_ee_link): {node.get_parameter('grinding_ee_link').value}")
    print(f"Joint difference limit (grinding_joint_difference_limit_for_motion_planning): {node.get_parameter('grinding_joint_difference_limit_for_motion_planning').value}")
    print("============================\n")


def start_jtc_executor():
    """JTC Helperを別スレッドのエクゼキューターで実行"""
    global jtc_helper, jtc_executor, jtc_thread
    
    def run_executor():
        global jtc_executor
        try:
            jtc_executor.spin()
        except Exception as e:
            print(f"JTC Executor error: {e}")
    
    if jtc_helper:
        jtc_executor = MultiThreadedExecutor()
        jtc_executor.add_node(jtc_helper)
        jtc_thread = threading.Thread(target=run_executor, daemon=True)
        jtc_thread.start()
        print("JTC Helper executor started in separate thread")


def stop_jtc_executor():
    """JTC Helperのエクゼキューターを停止"""
    global jtc_executor, jtc_thread
    
    if jtc_executor:
        jtc_executor.shutdown()
        jtc_executor = None
    
    if jtc_thread:
        jtc_thread.join(timeout=1.0)
        jtc_thread = None


def display_current_joint_positions():
    """Display current joint positions"""
    global jtc_helper
    if not jtc_helper:
        print("JTC Helper not initialized")
        return
    
    try:
        current_positions = jtc_helper.get_current_joint_positions()
        if current_positions is not None:
            print("\n=== Current Joint Positions ===")
            joint_names = jtc_helper.valid_joint_names
            for i, (name, pos) in enumerate(zip(joint_names, current_positions)):
                print(f"{name}: {pos:.4f} rad ({pos * 180.0 / 3.14159:.2f} deg)")
            print("===============================\n")
        else:
            print("Failed to get current joint positions")
    except Exception as e:
        print(f"Error getting joint positions: {e}")


def exit_process(msg=""):
    global node
    if msg != "":
        if node:
            node.get_logger().info(msg)
    if node:
        node.get_logger().info("Exit mechano grinding")
    rclpy.shutdown()
    exit()

def command_to_execute(cmd):
    if cmd == "y":
        return True
    elif cmd == "mk":
        return False
    elif cmd == "pose":
        return False
    else:
        return None

def main():
    global debug_marker, debug_pose, node, jtc_helper
    
    rclpy.init(args=sys.argv)
    node = Node("mechano_grinding")
    
    # Declare parameters with default values
    node.declare_parameter("log_file_dir", "")
    node.declare_parameter("experiment_time", 60.0)
    node.declare_parameter("pouse_time_list", [10.0, 20.0, 30.0])
    
    # Declare position parameters (will be overridden by YAML file)
    node.declare_parameter("mortar_top_position", [0.0, 0.0, 0.0])
    node.declare_parameter("mortar_inner_scale", [0.04, 0.04, 0.035])
    node.declare_parameter("move_group_name", "manipulator")
    node.declare_parameter("grinding_ee_link", "pestle_tip")
    node.declare_parameter("gathering_ee_link", "spatula_tip")
    node.declare_parameter("grinding_joint_difference_limit_for_motion_planning", 0.03)
    node.declare_parameter("gathering_joint_difference_limit_for_motion_planning", 0.03)
    node.declare_parameter("motion_planner_id", "TRRT")
    node.declare_parameter("planning_time", 20.0)
    node.declare_parameter("max_attempts", 5)
    
    # Grinding parameters (positions in mm)
    node.declare_parameter("grinding_pos_beginning_mm", [-8.0, 0.0])
    node.declare_parameter("grinding_pos_end_mm", [-8.0, 0.0001])
    node.declare_parameter("grinding_rz_beginning_mm", 35.0)
    node.declare_parameter("grinding_rz_end_mm", 35.0)
    node.declare_parameter("grinding_angle_scale", 0.3)
    node.declare_parameter("grinding_yaw_bias", pi)
    node.declare_parameter("grinding_number_of_rotation", 10.0)
    node.declare_parameter("grinding_number_of_waypoints_per_circle", 100)
    node.declare_parameter("grinding_center_pos_mm", [0.0, 0.0])
    node.declare_parameter("grinding_sec_per_rotation", 0.5)
    
    # Gathering parameters (positions in mm)
    node.declare_parameter("gathering_pos_beginning_mm", [30.0, 0.0])
    node.declare_parameter("gathering_pos_end_mm", [-22.0, 0.0001])
    node.declare_parameter("gathering_rz_beginning_mm", 35.0)
    node.declare_parameter("gathering_rz_end_mm", 35.0)
    node.declare_parameter("gathering_angle_scale", 0.0)
    node.declare_parameter("gathering_yaw_bias", pi)
    node.declare_parameter("gathering_number_of_rotation", 5.0)
    node.declare_parameter("gathering_number_of_waypoints_per_circle", 100)
    node.declare_parameter("gathering_center_pos_mm", [0.0, 0.0])
    node.declare_parameter("gathering_sec_per_rotation", 2.0)
    
    # Robot parameters
    node.declare_parameter("ik_solver", "trac_ik")
    node.declare_parameter("robot_urdf_file_name", "ur/ur5e_with_pestle")
    node.declare_parameter("joint_trajectory_controller_name", "scaled_joint_trajectory_controller")
    node.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
    node.declare_parameter("joint_names", [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    ])
    node.declare_parameter("base_link", "base_link")
    node.declare_parameter("robot_description_package", "grinding_robot_description")
    node.declare_parameter(
        "robot_description_file_path", "/urdf/ur/ur5e_with_pestle.urdf"
    )
    
    
    # Get parameters
    log_file_dir = node.get_parameter("log_file_dir").value
    target_experiment_time = node.get_parameter("experiment_time").value
    pouse_time_list = node.get_parameter("pouse_time_list").value
    TIMEOUT_SEC = 0.1
    current_experiment_time = 0
    
    ################### motion generator ###################
    # Build dictionary parameters from array parameter values
    mortar_top_pos_array = node.get_parameter("mortar_top_position").value
    mortar_inner_scale_array = node.get_parameter("mortar_inner_scale").value
    
    mortar_top_pos = {
        "x": mortar_top_pos_array[0],
        "y": mortar_top_pos_array[1],
        "z": mortar_top_pos_array[2]
    }
    mortar_inner_size = {
        "x": mortar_inner_scale_array[0],
        "y": mortar_inner_scale_array[1],
        "z": mortar_inner_scale_array[2]
    }
    motion_gen = MotionGenerator(mortar_top_pos, mortar_inner_size)

    ################### motion executor ###################
    grinding_ee_link = node.get_parameter("grinding_ee_link").value
    gathering_ee_link = node.get_parameter("gathering_ee_link").value
    grinding_joint_difference_limit_for_motion_planning = node.get_parameter("grinding_joint_difference_limit_for_motion_planning").value
    gathering_joint_difference_limit_for_motion_planning = node.get_parameter("gathering_joint_difference_limit_for_motion_planning").value
    motion_planner_id = node.get_parameter("motion_planner_id").value
    planning_time = node.get_parameter("planning_time").value
    max_attempts = node.get_parameter("max_attempts").value
    
    ################### init pose ###################
    init_pos = copy.deepcopy(mortar_top_pos)
    node.get_logger().info("Mortar pos: " + str(init_pos))
    init_pos["z"] += 0.05
    yaw = np.arctan2(mortar_top_pos["y"], mortar_top_pos["x"]) + pi
    euler = [pi, 0, yaw]
    r = Rotation.from_euler("xyz", euler, degrees=False)
    quat = r.as_quat()
    init_pose = list(init_pos.values()) + list(quat)
    
    # Initialize JTC Helper
    controller_name = node.get_parameter("controller_name").value
    joint_names = node.get_parameter("joint_names").value
    base_link = node.get_parameter("base_link").value
    robot_description_package = node.get_parameter("robot_description_package").value
    urdf_file_path = node.get_parameter("robot_description_file_path").value
    ik_solver_name = node.get_parameter("ik_solver").value
    
    jtc_helper = JointTrajectoryControllerHelper(
        controller_name=controller_name,
        joints_name=joint_names,
        tool_link=grinding_ee_link,
        base_link=base_link,
        robot_urdf_file_path=get_package_share_directory(robot_description_package) + urdf_file_path,
        ik_solver=IKType.TRACK_IK if ik_solver_name == "trac_ik" else IKType.BIO_IK,
        node_name="grinding_jtc_helper"
    )

    # Initialize motion primitive
    primitive = GrindingMotionPrimitive(
        node=node,
        jtc_helper=jtc_helper,
        init_pose=init_pose,
        grinding_ee_link=grinding_ee_link,
        gathering_ee_link=gathering_ee_link,
    )
    
    # Go to initial pose
    primitive.go_to_init_pose()
    node.get_logger().info("Goto init pose")

    # Initialize debug tools with unique node names
    debug_marker = MarkerPublisher(node_name="grinding_marker_display")
    debug_pose = PosePublisher(node_name="grinding_pose_publisher")

    # Start JTC Helper executor in separate thread
    start_jtc_executor()

    grinding_sec = node.get_parameter("grinding_sec_per_rotation").value * node.get_parameter("grinding_number_of_rotation").value
    gathering_sec = node.get_parameter("gathering_sec_per_rotation").value * node.get_parameter("gathering_number_of_rotation").value
    
    try:
        while rclpy.ok():
            motion_command = input(
                "q \t= exit.\n"
                + "p \t= display grinding parameters.\n"
                + "j \t= display current joint positions.\n"
                + "top \t= go to mortar top position.\n"
                + "g \t= grinding demo.\n"
                + "G \t= Gathering demo.\n"
                + "Rg \t= Repeate Grinding  motion during the experiment time.\n"
                + "RGG \t= Repeate Grinding and Gathering motion during the experiment time.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_process()
            elif motion_command == "p":
                display_grinding_parameters()
            elif motion_command == "j":
                display_current_joint_positions()

            elif motion_command == "top":
                node.get_logger().info("Go to caliblation pose of pestle tip position")
                pos = copy.deepcopy(mortar_top_pos)
                quat = init_pose[3:]
                calib_pose = list(pos.values()) + quat
                primitive.execute_cartesian_path_to_pose(calib_pose)

            elif motion_command == "g":
                key = input(
                    "Start grinding demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints poses = 'pose', canncel = other\n"
                )
                exec_result = command_to_execute(key)
                if exec_result:
                    waypoints = compute_grinding_waypoints(motion_gen)
                    waypoints_list = waypoints.tolist() if hasattr(waypoints, 'tolist') else waypoints
                    primitive.execute_grinding(
                        waypoints=waypoints_list,
                        grinding_sec=grinding_sec,
                        joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                        max_ik_retries_on_jump=max_attempts,
                        wait_for_completion=True
                    )
                elif exec_result == False:
                    compute_grinding_waypoints(motion_gen, debug_type=key)
                    
            elif motion_command == "G":
                key = input(
                    "Start circular gathering demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints poses = 'pose', canncel = other\n"
                )
                exec_result = command_to_execute(key)
                if exec_result:
                    waypoints = compute_gathering_waypoints(motion_gen)
                    waypoints_list = waypoints.tolist() if hasattr(waypoints, 'tolist') else waypoints
                    primitive.execute_gathering(
                        waypoints=waypoints_list,
                        gathering_sec=gathering_sec,
                        joint_difference_limit=gathering_joint_difference_limit_for_motion_planning,
                        max_ik_retries_on_jump=max_attempts,
                        wait_for_completion=True
                    )
                elif exec_result == False:
                    compute_gathering_waypoints(motion_gen, debug_type=key)

            elif motion_command == "Rg":
                i = 0
                motion_counts = 0
                grinding_trajectory = []

                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding -> 'q', pose -> 'p'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_process()
                        elif key == "p":
                            input("Press Enter to continue...")

                    except TimeoutOccurred:
                        st = time.time()
                        if len(grinding_trajectory) == 0:
                            waypoints = compute_grinding_waypoints(motion_gen)
                            waypoints_list = waypoints.tolist() if hasattr(waypoints, 'tolist') else waypoints
                            grinding_trajectory = waypoints_list

                        trajectory_success, pestle_ready_joints = primitive.execute_grinding(
                            waypoints=grinding_trajectory,
                            grinding_sec=grinding_sec,
                            joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                            wait_for_completion=True
                        )

                        motion_counts += 1

                    current_experiment_time += (time.time() - st) / 60
                    node.get_logger().info("Experiment time: " + str(current_experiment_time) + " min")
                    
                    if i < len(pouse_time_list) and pouse_time_list[i] < current_experiment_time:
                        i += 1
                        input("Pouse experiment on pouse settings. Press Enter to continue...")
                        
                    if current_experiment_time > target_experiment_time:
                        node.get_logger().info("Over experiment time")
                        exit_process("Motion counts: " + str(motion_counts))

            elif motion_command == "RGG":
                i = 0
                motion_counts = 0
                grinding_trajectory = []
                gathering_trajectory = []

                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding -> 'q', pose -> 'p'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_process()
                        elif key == "p":
                            input("Press Enter to continue...")

                    except TimeoutOccurred:
                        st = time.time()
                        
                        if len(grinding_trajectory) == 0:
                            waypoints = compute_grinding_waypoints(motion_gen)
                            waypoints_list = waypoints.tolist() if hasattr(waypoints, 'tolist') else waypoints
                            grinding_trajectory = waypoints_list

                        trajectory_success, pestle_ready_joints = primitive.execute_grinding(
                            waypoints=grinding_trajectory,
                            grinding_sec=grinding_sec,
                            joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                            wait_for_completion=True
                        )

                        if len(gathering_trajectory) == 0:
                            waypoints = compute_gathering_waypoints(motion_gen)
                            waypoints_list = waypoints.tolist() if hasattr(waypoints, 'tolist') else waypoints
                            gathering_trajectory = waypoints_list

                        trajectory_success, spatula_ready_joints = primitive.execute_gathering(
                            waypoints=gathering_trajectory,
                            gathering_sec=gathering_sec,
                            joint_difference_limit=gathering_joint_difference_limit_for_motion_planning,
                            wait_for_completion=True
                        )
                        
                        motion_counts += 1
                        
                        if log_file_dir != "":
                            with open(log_file_dir + "pestle_and_spatula_joints_log.csv", "a") as file:
                                writer = csv.writer(file)
                                writer.writerow([pestle_ready_joints, spatula_ready_joints])

                    current_experiment_time += (time.time() - st) / 60
                    node.get_logger().info("Experiment time: " + str(current_experiment_time) + " min")
                    
                    if i < len(pouse_time_list) and pouse_time_list[i] < current_experiment_time:
                        i += 1
                        input("Pouse experiment on pouse settings. Press Enter to continue...")
                        
                    if current_experiment_time > target_experiment_time:
                        node.get_logger().info("Over experiment time")
                        exit_process("Motion counts: " + str(motion_counts))

            # Allow ROS2 to process callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt as err:
        exit_process(str(err))
    except Exception as err:
        exit_process(str(err))
    finally:
        # Stop JTC Helper executor
        stop_jtc_executor()
        
        if debug_marker:
            debug_marker.destroy_node()
        if debug_pose:
            debug_pose.destroy_node()
        if jtc_helper:
            jtc_helper.destroy_node()
        if node:
            node.destroy_node()

if __name__ == "__main__":
    main()