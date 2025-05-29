#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# tf_transformations は ROS 1/2 共通で使えることが多い
import numpy as np

from typing import List, Optional, Tuple

# ROS 2 版の JTC ヘルパーをインポート
from grinding_robot_control.JTC_helper import JointTrajectoryControllerHelper, IKType

# MoveIt ヘルパーは未実装のためコメントアウト
# from .moveit_helper_ros2 import MoveitHelper # ROS 2 MoveIt ラッパーのプレースホルダー

from geometry_msgs.msg import Pose, PoseStamped


class GrindingMotionPrimitive:
    """
    ROS 2 用のモーションプリミティブクラス。
    JTC_helper を使用して基本的な動作を実行する。
    MoveIt 関連の機能は未実装。
    """

    def __init__(
        self,
        node: Node,  # 呼び出し元の ROS 2 ノード
        jtc_helper: JointTrajectoryControllerHelper,
        init_pose: List[float] = [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],  # デフォルト値
        grinding_ee_link: str = "pestle_tip",  # デフォルト値
        gathering_ee_link: str = "spatula_tip",  # デフォルト値
    ):
        """
        GrindingMotionPrimitive を初期化する。

        Args:
            node: ログ出力やパラメータ取得に使用する ROS 2 ノード。
            jtc_helper: 初期化済みの JointTrajectoryControllerHelper インスタンス。
            # moveit_helper: 初期化済みの MoveitHelper インスタンス (オプション)。
            init_pose: 初期姿勢 [x, y, z, qx, qy, qz, qw]。
            grinding_ee_link: 研削時のエンドエフェクタリンク名。
            gathering_ee_link: 収集時のエンドエフェクタリンク名。
        """
        self.node = node
        self.logger = node.get_logger()
        self.jtc_helper = jtc_helper
        # self.moveit_helper = moveit_helper

        if len(init_pose) != 7:
            raise ValueError("init_pose must be [x, y, z, qx, qy, qz, qw]")
        self.init_pose = init_pose
        # JTCヘルパーが使用するEEリンクを確認 (デバッグ用)
        self.logger.info(
            f"JTC Helper initialized with tool_link: {self.jtc_helper.tool_link}"
        )
        # 注意: grinding/gathering ee_link は JTC Helper の初期化と一致している必要がある
        self.grinding_ee_link = grinding_ee_link
        self.gathering_ee_link = gathering_ee_link
        # JTCヘルパーのEEリンクがどちらのタスク用か明確にする必要がある
        # ここでは、渡されたJTCヘルパーが現在のタスクに適したEEリンクを持っていると仮定する

    def _pose_to_list(self, pose: Pose) -> List[float]:
        """geometry_msgs/Pose をリスト [x,y,z,qx,qy,qz,qw] に変換する。"""
        return [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]

    def _list_to_pose(self, pose_list: List[float]) -> Pose:
        """リスト [x,y,z,qx,qy,qz,qw] を geometry_msgs/Pose に変換する。"""
        if len(pose_list) != 7:
            raise ValueError("Pose list must have 7 elements [x,y,z,qx,qy,qz,qw]")
        pose = Pose()
        pose.position.x = pose_list[0]
        pose.position.y = pose_list[1]
        pose.position.z = pose_list[2]
        pose.orientation.x = pose_list[3]
        pose.orientation.y = pose_list[4]
        pose.orientation.z = pose_list[5]
        pose.orientation.w = pose_list[6]
        return pose

    def execute_grinding(
        self,
        waypoints: List[List[float]],  # List of pose lists [x,y,z,qx,qy,qz,qw]
        grinding_sec: float,
        joint_difference_limit: Optional[
            float
        ] = 0.03,  # JTC_helperのmax_joint_change_per_stepに対応
        max_ik_retries_on_jump: int = 100,  # JTC_helperのmax_ik_retries_on_jumpに対応
        ik_retry_perturbation: float = 0.05,  # JTC_helperのik_retry_perturbationに対応
        pre_motion: bool = True,
        post_motion: bool = True,
        wait_for_completion: bool = True,
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        研削動作を実行する。JTC_helper を使用する。
        JTC_helper は grinding_ee_link で初期化されている必要がある。

        Args:
            waypoints: 研削パスのウェイポイント (姿勢リストのリスト)。
            grinding_sec: 研削パスの実行時間 (秒)。
            joint_difference_limit: ウェイポイント間の許容最大関節角度変化量 (ラジアン)。Noneで無効。
            max_ik_retries_on_jump: 角度変化超過時のIKリトライ回数上限。
            ik_retry_perturbation: IKリトライ時のシード値摂動量 (ラジアン)。
            pre_motion: 動作前に初期姿勢に移動するかどうか。
            post_motion: 動作後に初期姿勢に戻るかどうか。
            wait_for_completion: 軌道実行の完了を待つかどうか。

        Returns:
            (成功フラグ, 初期姿勢の関節角度リスト or None)
        """
        self.logger.info("Starting grinding execution...")

        # 1. Pre-motion: 初期姿勢へ移動
        if pre_motion:
            moving_init_pose_success = self.jtc_helper.set_goal_pose(
                goal_pose=self.init_pose,
                num_axes_to_check_for_goal=self.jtc_helper.num_joints - 1,
                send_immediately=True,
                wait=True,
                target_ee_link=self.grinding_ee_link,
            )
            if moving_init_pose_success is None:
                self.logger.error("Failed to move to initial pose.")
                return False, None
            self.logger.info("Moved to initial pose for grinding.")

        # 2. 研削パスの開始点へ移動
        first_waypoint_pose = waypoints[0]
        self.logger.info("Moving to the first grinding waypoint...")
        # set_goal_pose を使用 (内部で IK を解く)
        q_init = self.jtc_helper.set_goal_pose(
            first_waypoint_pose,
            time_to_reach=2,
            num_axes_to_check_for_goal=self.jtc_helper.num_joints - 1,
            send_immediately=True,
            wait=True,
        )
        # set_goal_pose の成功/失敗をハンドリングする必要があるかもしれない
        self.logger.info("Reached the first grinding waypoint.")

        # 3. 研削軌道を実行
        self.logger.info(
            f"Executing grinding trajectory ({len(waypoints)} points, {grinding_sec:.2f} sec)..."
        )
        # set_waypoints を使用 (内部で IK、角度変化チェック、リトライを行う)
        try:
            self.jtc_helper.set_waypoints(
                waypoints=waypoints,
                time_to_reach=grinding_sec,
                max_joint_change_per_step=joint_difference_limit,
                max_ik_retries_on_jump=max_ik_retries_on_jump,
                ik_retry_perturbation=ik_retry_perturbation,
                send_immediately=True,
                wait=wait_for_completion,
                q_init=q_init,
            )
            # JTC_helper.set_waypoints がエラーを raise しなければ成功とみなす
            self.logger.info("Grinding trajectory sent/executed.")
        except Exception as e:
            self.logger.error(f"Error during set_waypoints for grinding: {e}")
            # 失敗した場合でも post_motion を試みるか？ここでは False を返す
            return False

        # 4. Post-motion: 初期姿勢へ戻る
        if post_motion:
            self.jtc_helper.set_goal_pose(
                goal_pose=self.init_pose,
                num_axes_to_check_for_goal=self.jtc_helper.num_joints - 1,
                send_immediately=True,
                wait=True,
            )
            self.logger.info("Returned to initial pose after grinding.")

        return True

    def execute_gathering(
        self,
        waypoints: List[List[float]],
        gathering_sec: float,
        joint_difference_limit: Optional[float] = 0.03,
        max_ik_retries_on_jump: int = 100,
        ik_retry_perturbation: float = 0.05,
        wait_for_completion: bool = True,
    ) -> Tuple[bool, Optional[List[float]]]:
        """
        収集動作を実行する。JTC_helper を使用する。
        JTC_helper は gathering_ee_link で初期化されている必要がある。

        Args:
            waypoints: 収集パスのウェイポイント (姿勢リストのリスト)。
            gathering_sec: 収集パスの実行時間 (秒)。
            joint_difference_limit: ウェイポイント間の許容最大関節角度変化量 (ラジアン)。Noneで無効。
            max_ik_retries_on_jump: 角度変化超過時のIKリトライ回数上限。
            ik_retry_perturbation: IKリトライ時のシード値摂動量 (ラジアン)。
            wait_for_completion: 軌道実行の完了を待つかどうか。

        Returns:
            (成功フラグ, 初期姿勢の関節角度リスト or None)
        """
        self.logger.info("Starting gathering execution...")

        # 1. 初期姿勢へ移動 (収集用EEリンクを使用)
        self.logger.info(
            f"Moving to initial pose for gathering (EE: {self.gathering_ee_link})..."
        )
        self.jtc_helper.set_goal_pose(
            goal_pose=self.init_pose,
            num_axes_to_check_for_goal=self.jtc_helper.num_joints - 3,
            send_immediately=True,
            wait=True,
            target_ee_link=self.gathering_ee_link,
            time_to_reach=3
        )
        self.logger.info("Moved to initial pose for gathering.")

        # 2. 収集パスの開始点へ移動
        if waypoints.shape[0] == 0:
            self.logger.error("No waypoints provided for gathering.")
            return False

        first_waypoint_pose = waypoints[0]
        self.logger.info("Moving to the first gathering waypoint...")
        self.jtc_helper.set_goal_pose(
            first_waypoint_pose,
            num_axes_to_check_for_goal=self.jtc_helper.num_joints,
            send_immediately=True,
            wait=True,
        )
        self.logger.info("Reached the first gathering waypoint.")

        # 3. 収集軌道を実行
        self.logger.info(
            f"Executing gathering trajectory ({len(waypoints)} points, {gathering_sec:.2f} sec)..."
        )
        try:
            self.jtc_helper.set_waypoints(
                waypoints=waypoints,
                time_to_reach=gathering_sec,
                max_joint_change_per_step=joint_difference_limit,
                max_ik_retries_on_jump=max_ik_retries_on_jump,
                ik_retry_perturbation=ik_retry_perturbation,
                send_immediately=True,
                wait=wait_for_completion,
            )
            self.logger.info("Gathering trajectory sent/executed.")
        except Exception as e:
            self.logger.error(f"Error during set_waypoints for gathering: {e}")
            return False

        # 4. Post-motion: 初期姿勢へ戻る
        self.jtc_helper.set_goal_pose(
            goal_pose=self.init_pose,
            num_axes_to_check_for_goal=self.jtc_helper.num_joints,
            send_immediately=True,
            wait=True,
        )
        self.logger.info("Returned to initial pose after gathering.")

        self.jtc_helper.change_ee_link(self.grinding_ee_link)
        self.jtc_helper.set_goal_pose(
            goal_pose=self.init_pose,
            num_axes_to_check_for_goal=self.jtc_helper.num_joints - 3,
            send_immediately=True,
            wait=True,
            target_ee_link=self.grinding_ee_link,
            time_to_reach=2
        )
        self.logger.info("Returned to initial pose.")

        return True


# --- Example Usage (within a ROS 2 node context) ---
def main(args=None):
    rclpy.init(args=args)
    # メインとなるノードを作成
    main_node = Node("motion_primitive_user")  # Nodeを直接作成

    # パラメータを宣言・取得 (例)
    # 実際の値は launch ファイルや YAML から読み込むのが一般的
    controller_name = main_node.declare_parameter(
        "controller_name", "scaled_joint_trajectory_controller"
    ).value
    joints_name = main_node.declare_parameter(
        "joints_name",
        [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ],
    ).value
    grinding_ee_link_param = main_node.declare_parameter(
        "grinding_ee_link", "pestle_tip"
    ).value
    gathering_ee_link_param = main_node.declare_parameter(
        "gathering_ee_link", "spatula_tip"
    ).value
    base_link_param = main_node.declare_parameter("base_link", "base_link").value
    urdf_pkg_param = main_node.declare_parameter(
        "robot_urdf_package", "grinding_robot_description"
    ).value
    urdf_file_param = main_node.declare_parameter(
        "robot_urdf_file_name", "ur/ur5e_with_pestle"
    ).value
    mortar_inner_size = {"x": 0.04, "y": 0.04, "z": 0.035}
    mortar_top_position = {
        "x": -0.2,
        "y": 0.4,
        "z": 0.05,
    }
    init_pose = [
        mortar_top_position["x"],
        mortar_top_position["y"],
        mortar_top_position["z"] + 0.03,
        1,
        0,
        0,
        0,
    ]  # 初期姿勢 (x,y,z,qx,qy,qz,qw)

    # JTC Helper を初期化 (研削用EEリンクを使用する場合)
    # 注意: 収集動作を実行する場合は、gathering_ee_link で初期化された別のインスタンスが必要になる可能性がある
    jtc_helper_grinding = JointTrajectoryControllerHelper(
        controller_name=controller_name,
        joints_name=joints_name,
        tool_link=grinding_ee_link_param,  # 研削用EEリンク
        base_link=base_link_param,
        robot_urdf_package=urdf_pkg_param,
        robot_urdf_file_name=urdf_file_param,
        ik_solver=IKType.TRACK_IK,  # 例
    )

    # GrindingMotionPrimitive を初期化
    try:
        from grinding_motion_routines.grinding_motion_generator import MotionGenerator
        from grinding_motion_routines.display_marker import DisplayMarker
    except ImportError as e:
        print(f"ImportError: {e}")
        main_node.get_logger().error(
            "Failed to import MotionGenerator or DisplayMarker."
        )
        rclpy.shutdown()
        return
    motion_primitive = GrindingMotionPrimitive(
        node=main_node,
        jtc_helper=jtc_helper_grinding,  # 研削用ヘルパーを渡す
        # moveit_helper=moveit_helper,
        init_pose=init_pose,
        grinding_ee_link=grinding_ee_link_param,
        gathering_ee_link=gathering_ee_link_param,
    )

    # --- 研削動作の実行例 ---
    main_node.get_logger().info("--- Executing Grinding Example ---")
    grinding_pos_beginning = [-8, 0]
    grinding_pos_end = [-8, 0.001]
    grinding_radius_z = 36
    number_of_rotations = 1
    angle_scale = 0.5
    yaw_bias = 0
    yaw_twist_per_rotation = np.pi / 2
    number_of_waypoints_per_circle = 50
    center_position = [0, 0]
    sec_per_rotation = 1

    motion_generator = MotionGenerator(mortar_top_position, mortar_inner_size)
    try:
        grinding_waypoints_example = motion_generator.create_circular_waypoints(
            beginning_position=grinding_pos_beginning,
            end_position=grinding_pos_end,
            beginning_radius_z=grinding_radius_z,
            end_radius_z=grinding_radius_z,
            number_of_rotations=number_of_rotations,
            number_of_waypoints_per_circle=number_of_waypoints_per_circle,
            angle_scale=angle_scale,
            yaw_bias=yaw_bias,
            yaw_twist_per_rotation=yaw_twist_per_rotation,
            center_position=center_position,
        )
    except ValueError as e:
        print(f"Error generating circular waypoints: {e}")
        rclpy.shutdown()
        return
    success = motion_primitive.execute_grinding(
        waypoints=grinding_waypoints_example,
        grinding_sec=number_of_rotations * sec_per_rotation,
        joint_difference_limit=np.pi / 4,
        pre_motion=True,
        post_motion=True,
        wait_for_completion=True,
    )
    if success:
        main_node.get_logger().info("Grinding example finished successfully.")
    else:
        main_node.get_logger().error("Grinding example failed.")

    # --- 収集動作の実行例 ---
    # 注意: 収集動作には gathering_ee_link で初期化された JTC Helper が必要
    # ここでは、デモのために同じヘルパーを使い回すが、EEリンクが異なるため期待通りに動作しない可能性がある
    main_node.get_logger().warn(
        "--- Executing Gathering Example (Using potentially incorrect EE link) ---"
    )
    # 収集用に JTC Helper を再初期化するか、別のインスタンスを使うべき
    # motion_primitive.jtc_helper = jtc_helper_gathering # 収集用ヘルパーに切り替え
    try:
        gathering_waypoints_example = motion_generator.create_circular_waypoints(
            beginning_position=grinding_pos_beginning,
            end_position=grinding_pos_end,
            beginning_radius_z=grinding_radius_z,
            end_radius_z=grinding_radius_z,
            number_of_rotations=number_of_rotations,
            number_of_waypoints_per_circle=number_of_waypoints_per_circle,
            angle_scale=0,
            yaw_bias=yaw_bias,
            yaw_twist_per_rotation=0,
            center_position=center_position,
        )
    except ValueError as e:
        print(f"Error generating circular waypoints: {e}")
        rclpy.shutdown()
        return
    success_gather = motion_primitive.execute_gathering(
        waypoints=gathering_waypoints_example,
        gathering_sec=number_of_rotations * sec_per_rotation,
        joint_difference_limit=np.pi / 4,
        wait_for_completion=True,
    )
    if success_gather:
        main_node.get_logger().info("Gathering example finished successfully.")
    else:
        main_node.get_logger().error("Gathering example failed.")

    # クリーンアップ
    main_node.get_logger().info("Shutting down...")
    jtc_helper_grinding.destroy_node()  # JTC Helper ノードも破棄
    main_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    # このファイル自体を実行するためのサンプルメイン関数
    main()
