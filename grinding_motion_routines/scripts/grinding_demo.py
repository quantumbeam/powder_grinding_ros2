#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import traceback # For formatting exception tracebacks
from typing import List, Optional, Tuple
import math # For deg2rad

# 必要なクラスをインポート
from grinding_motion_routines.grinding_motion_primitive import GrindingMotionPrimitive
from grinding_motion_routines.grinding_motion_generator import MotionGenerator
from grinding_motion_routines.display_marker import DisplayMarker # マーカー表示用
from grinding_robot_control.JTC_helper import JointTrajectoryControllerHelper, IKType

def main(args=None):
    rclpy.init(args=args)
    # メインとなるノードを作成
    main_node = Node('grinding_demo_node')
    logger = main_node.get_logger()

    logger.info("Starting Grinding Demo Node...")

    # --- パラメータの宣言・取得 ---
    try:
        logger.info("Declaring and getting parameters...")
        # Robot and Controller Parameters
        controller_name = main_node.declare_parameter('controller_name', 'scaled_joint_trajectory_controller').get_parameter_value().string_value
        joint_names = main_node.declare_parameter('joint_names', [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]).get_parameter_value().string_array_value
        base_link = main_node.declare_parameter('base_link', 'base_link').get_parameter_value().string_value
        grinding_ee_link = main_node.declare_parameter('grinding_ee_link', 'pestle_tip').get_parameter_value().string_value
        gathering_ee_link = main_node.declare_parameter('gathering_ee_link', 'spatula_tip').get_parameter_value().string_value
        robot_description_package = main_node.declare_parameter('robot_description_package', 'grinding_robot_description').get_parameter_value().string_value # Snippet: "grinding_robot_description"
        robot_description_file = main_node.declare_parameter('robot_description_file', 'ur/ur5e_with_pestle').get_parameter_value().string_value # Snippet: "ur/ur5e_with_pestle"

        # Mortar Parameters
        mortar_inner_size_x = main_node.declare_parameter('mortar.inner_size.x', 0.04).get_parameter_value().double_value  # Snippet: 0.04
        mortar_inner_size_y = main_node.declare_parameter('mortar.inner_size.y', 0.04).get_parameter_value().double_value  # Snippet: 0.04
        mortar_inner_size_z = main_node.declare_parameter('mortar.inner_size.z', 0.035).get_parameter_value().double_value # Snippet: 0.035
        mortar_top_pos_x = main_node.declare_parameter('mortar.top_position.x', -0.2).get_parameter_value().double_value    # Snippet: -0.2
        mortar_top_pos_y = main_node.declare_parameter('mortar.top_position.y', 0.4).get_parameter_value().double_value     # Snippet: 0.4
        mortar_top_pos_z = main_node.declare_parameter('mortar.top_position.z', 0.3).get_parameter_value().double_value     # Snippet: 0.3

        mortar_inner_size = {"x": mortar_inner_size_x, "y": mortar_inner_size_y, "z": mortar_inner_size_z}
        mortar_top_position = {"x": mortar_top_pos_x, "y": mortar_top_pos_y, "z": mortar_top_pos_z}

        # Initial Pose Parameters
        init_pose_offset_z = main_node.declare_parameter('init_pose.offset_z', 0.05).get_parameter_value().double_value # Keep offset, base pose will change due to mortar_top_position
        init_pose_quat_x = main_node.declare_parameter('init_pose.orientation.x', 1.0).get_parameter_value().double_value
        init_pose_quat_y = main_node.declare_parameter('init_pose.orientation.y', 0.0).get_parameter_value().double_value
        init_pose_quat_z = main_node.declare_parameter('init_pose.orientation.z', 0.0).get_parameter_value().double_value
        init_pose_quat_w = main_node.declare_parameter('init_pose.orientation.w', 0.0).get_parameter_value().double_value

        init_pose = [
            mortar_top_position["x"],
            mortar_top_position["y"],
            mortar_top_position["z"] + init_pose_offset_z,
            init_pose_quat_x, init_pose_quat_y, init_pose_quat_z, init_pose_quat_w
        ]
        logger.info(f"Using initial pose: {init_pose}")

        # Grinding Parameters
        grinding_start_offset_xy = main_node.declare_parameter('grinding.start_pos_xy_mm', [-8.0, 0.0]).get_parameter_value().double_array_value
        grinding_end_offset_xy = main_node.declare_parameter('grinding.end_pos_xy_mm', [-8.0, 0.0001]).get_parameter_value().double_array_value
        grinding_depth_st_mm = main_node.declare_parameter('grinding.depth_st_mm', -35.0).get_parameter_value().double_value
        grinding_depth_end_mm = main_node.declare_parameter('grinding.depth_end_mm', -35.0).get_parameter_value().double_value
        grinding_rotations = main_node.declare_parameter('grinding.rotations', 5.0).get_parameter_value().double_value
        grinding_angle_scale = main_node.declare_parameter('grinding.angle_scale', 0.3).get_parameter_value().double_value
        grinding_yaw_twist_deg = main_node.declare_parameter('grinding.yaw_twist_per_rotation_deg', 90.0).get_parameter_value().double_value # Snippet: np.pi/2 rad = 90 deg
        grinding_waypoints_per_circle = main_node.declare_parameter('grinding.waypoints_per_circle', 100).get_parameter_value().integer_value
        grinding_sec_per_rotation = main_node.declare_parameter('grinding.sec_per_rotation', 0.5).get_parameter_value().double_value
        grinding_center_offset_x = main_node.declare_parameter('grinding.center_offset.x', 0.0).get_parameter_value().double_value # Snippet: center_position[0]
        grinding_center_offset_y = main_node.declare_parameter('grinding.center_offset.y', 0.0).get_parameter_value().double_value # Snippet: center_position[1]
        grinding_yaw_bias_rad = main_node.declare_parameter('grinding.yaw_bias_rad', math.pi).get_parameter_value().double_value
        grinding_vel_scale = main_node.declare_parameter('grinding.vel_scale', 1.0).get_parameter_value().double_value
        grinding_acc_scale = main_node.declare_parameter('grinding.acc_scale', 1.0).get_parameter_value().double_value
        grinding_yaw_twist_rad = math.radians(grinding_yaw_twist_deg) # Convert to radians

        # Gathering Parameters
        gathering_start_offset_xy = main_node.declare_parameter('gathering.start_pos_xy_mm', [30.0, 0.0]).get_parameter_value().double_array_value
        gathering_end_offset_xy = main_node.declare_parameter('gathering.end_pos_xy_mm', [-22.0, 0.0001]).get_parameter_value().double_array_value
        gathering_depth_st_mm = main_node.declare_parameter('gathering.depth_st_mm', -35.0).get_parameter_value().double_value
        gathering_depth_end_mm = main_node.declare_parameter('gathering.depth_end_mm', -35.0).get_parameter_value().double_value
        gathering_rotations = main_node.declare_parameter('gathering.rotations', 5.0).get_parameter_value().double_value
        gathering_angle_scale = main_node.declare_parameter('gathering.angle_scale', 0.0).get_parameter_value().double_value
        gathering_waypoints_per_circle = main_node.declare_parameter('gathering.waypoints_per_circle', 100).get_parameter_value().integer_value
        gathering_sec_per_rotation = main_node.declare_parameter('gathering.sec_per_rotation', 2.0).get_parameter_value().double_value
        gathering_center_offset_x = main_node.declare_parameter('gathering.center_offset.x', 0.0).get_parameter_value().double_value
        gathering_center_offset_y = main_node.declare_parameter('gathering.center_offset.y', 0.0).get_parameter_value().double_value
        gathering_yaw_bias_rad = main_node.declare_parameter('gathering.yaw_bias_rad', math.pi).get_parameter_value().double_value
        gathering_vel_scale = main_node.declare_parameter('gathering.vel_scale', 0.1).get_parameter_value().double_value
        gathering_acc_scale = main_node.declare_parameter('gathering.acc_scale', 0.1).get_parameter_value().double_value

        # Motion Execution Parameters
        joint_difference_limit_rad = main_node.declare_parameter('motion.joint_difference_limit_rad', 0.03).get_parameter_value().double_value
        max_ik_retries_on_jump = main_node.declare_parameter('motion.max_ik_retries_on_jump', 100).get_parameter_value().integer_value
        ik_retry_perturbation_rad = main_node.declare_parameter('motion.ik_retry_perturbation_rad', 0.05).get_parameter_value().double_value
        pre_motion = main_node.declare_parameter('motion.pre_motion', True).get_parameter_value().bool_value
        post_motion = main_node.declare_parameter('motion.post_motion', True).get_parameter_value().bool_value
        wait_for_completion = main_node.declare_parameter('motion.wait_for_completion', True).get_parameter_value().bool_value

        logger.info("Parameters loaded successfully.")

    except Exception as e:
        logger.error(f"Failed to declare or get parameters: {e}\n{traceback.format_exc()}")
        rclpy.shutdown()
        return

    # --- ヘルパーとジェネレータの初期化 ---
    jtc_helper = None # Define outside try block for finally clause
    display_marker = None
    try:
        # JTC Helper を初期化 (最初は研削用EEリンクを使用)
        jtc_helper = JointTrajectoryControllerHelper(
            controller_name=controller_name,
            joints_name=joint_names,
            tool_link=grinding_ee_link, # 初期は研削用
            base_link=base_link,
            robot_urdf_package=robot_description_package,
            robot_urdf_file_name=robot_description_file,
            ik_solver=IKType.TRACK_IK # または BIO_IK
        )
        logger.info("JointTrajectoryControllerHelper initialized.")

        # JTC Helper が準備できるまで待機 (オプション)
        # logger.info("Waiting for JTC Helper to be ready...")
        # rclpy.spin_once(jtc_helper, timeout_sec=1.0) # Spin once to allow subscriptions
        # if not jtc_helper.wait_for_joint_state(timeout_sec=10.0):
        #      logger.error("JTC Helper failed to initialize (timeout waiting for joint states).")
        #      raise RuntimeError("JTC Helper initialization failed")
        # logger.info("JTC Helper is ready.")

        # MotionGenerator を初期化
        motion_generator = MotionGenerator(mortar_top_position, mortar_inner_size)
        logger.info("MotionGenerator initialized.")

        # GrindingMotionPrimitive を初期化
        motion_primitive = GrindingMotionPrimitive(
            node=main_node,
            jtc_helper=jtc_helper, # 作成したヘルパーを渡す
            init_pose=init_pose,
            grinding_ee_link=grinding_ee_link,
            gathering_ee_link=gathering_ee_link,
        )
        logger.info("GrindingMotionPrimitive initialized.")

        # マーカー表示用
        display_marker = DisplayMarker()
        logger.info("DisplayMarker initialized.")

    except Exception as e:
        logger.error(f"Failed to initialize helpers or generators: {e}\n{traceback.format_exc()}")
        if jtc_helper:
            jtc_helper.destroy_node()
        rclpy.shutdown()
        return

    # --- デモ動作の実行 ---
    try:
        # --- 1. 研削動作 ---
        logger.info("--- Generating Grinding Waypoints ---")
      
        grinding_waypoints = motion_generator.create_circular_waypoints(
                    beginning_position=grinding_start_offset_xy,
                    end_position=grinding_end_offset_xy,
                    beginning_radius_z=grinding_depth_st_mm,
                    end_radius_z=grinding_depth_end_mm,
                    number_of_rotations=grinding_rotations,
                    number_of_waypoints_per_circle=grinding_waypoints_per_circle,
                    angle_scale=grinding_angle_scale,
                    yaw_bias=grinding_yaw_bias_rad,
                    yaw_twist_per_rotation=grinding_yaw_twist_rad,
                    center_position=[grinding_center_offset_x, grinding_center_offset_y]
                )
        
        # マーカー表示
        display_marker.display_waypoints(grinding_waypoints, scale=0.002) # Added scale from snippet
        logger.info("Published grinding path markers.")

        # 研削動作実行
        logger.info("--- Executing Grinding Motion ---")
        success_grind, _ = motion_primitive.execute_grinding(
            waypoints=grinding_waypoints, # Pass numpy array
            grinding_sec=grinding_sec_per_rotation * grinding_rotations,
            joint_difference_limit=joint_difference_limit_rad,
            max_ik_retries_on_jump=max_ik_retries_on_jump,
            ik_retry_perturbation=ik_retry_perturbation_rad,
            pre_motion=pre_motion,
            post_motion=post_motion,
            wait_for_completion=wait_for_completion
        )

        if not success_grind:
            logger.error("Grinding motion failed.")
            # デモを継続するか、ここで終了するか選択
            # raise RuntimeError("Grinding motion failed") # 例: エラーで終了
        else:
            logger.info("Grinding motion completed.")


        # --- 2. 収集動作 ---
        logger.info("--- Generating Gathering Waypoints ---")
        # ウェイポイント生成 (開始半径から終了半径へ)
        gathering_waypoints = motion_generator.create_circular_waypoints(
            beginning_position=list(gathering_start_offset_xy),
            end_position=list(gathering_end_offset_xy),
            beginning_radius_z=gathering_depth_st_mm,
            end_radius_z=gathering_depth_end_mm,
            number_of_rotations=gathering_rotations,
            number_of_waypoints_per_circle=gathering_waypoints_per_circle,
            angle_scale=gathering_angle_scale,
            yaw_bias=gathering_yaw_bias_rad,
            yaw_twist_per_rotation=0.0, # No twist for gathering
            center_position=[gathering_center_offset_x, gathering_center_offset_y],
        )

        # マーカー表示
        display_marker.display_waypoints(gathering_waypoints)
        logger.info("Published gathering path markers.")

        # 収集動作実行
        logger.info("--- Executing Gathering Motion ---")
        # GrindingMotionPrimitive 内で EE リンクが gathering_ee_link に切り替わるはず
        success_gather, _ = motion_primitive.execute_gathering(
            waypoints=gathering_waypoints, # Pass numpy array
            gathering_sec=gathering_sec_per_rotation * gathering_rotations,
            joint_difference_limit=joint_difference_limit_rad,
            max_ik_retries_on_jump=max_ik_retries_on_jump,
            ik_retry_perturbation=ik_retry_perturbation_rad,
            wait_for_completion=wait_for_completion
        )

        if not success_gather:
            logger.error("Gathering motion failed.")
            # raise RuntimeError("Gathering motion failed")
        else:
            logger.info("Gathering motion completed.")

        logger.info("Grinding Demo finished successfully.")

    except Exception as e:
        logger.error(f"An error occurred during the demo: {e}\n{traceback.format_exc()}") # トレースバックも表示

    finally:
        # --- クリーンアップ ---
        logger.info("Shutting down...")
        if display_marker:
            display_marker.destroy_node() # マーカーノードも破棄
        if jtc_helper:
            jtc_helper.destroy_node() # JTC Helper ノードも破棄
        # main_node は最後に破棄
        if 'main_node' in locals() and main_node and rclpy.ok():
             main_node.destroy_node()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()
