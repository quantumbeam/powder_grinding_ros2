import sys
import time
from enum import Enum, auto
from typing import List, Optional, Union

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# パスを通して pytracik を参照
sys.path.append('/home/ubuntu/user/grinding_ws/src/powder_grinding/pytracik')
from pytracik.trac_ik import TracIK as TRACK_IK_SOLVER


class IKType(Enum):
    TRACK_IK = auto()
    # 他の IK ソルバーは未実装
    # PICK_IK = auto()
    # QU_IK = auto()


class ArmPositionController(Node):
    """
    位置制御を行うアームコントローラー
    """
    def __init__(self,
                 controller_name: str = "joint_trajectory_controller",
                 joints_name: List[str] = [""],
                 tool_link: str = "tool0",
                 base_link: str = "base_link",
                 robot_urdf_package: str = "grinding_robot_description",
                 robot_urdf_file_name: str = "ur5e_with_pestle",
                 ik_solver: IKType = IKType.TRACK_IK) -> None:
        """
        ノードの初期化と IK ソルバーのセットアップを行う

        Args:
            controller_name: コントローラーの名称
            joints_name: ジョイント名のリスト
            tool_link: ツールリンク名
            base_link: ベースリンク名
            robot_urdf_package: URDF を含むパッケージ名
            robot_urdf_file_name: URDF ファイル名（拡張子不要）
            ik_solver: 使用するIKソルバーの種類
        """
        super().__init__('arm_position_controller')

        self.controller_name = controller_name
        self.joints_name = joints_name
        if not self.joints_name:
            raise Exception('"joints" parameter is not set!')

        # アクションクライアントの設定
        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.goals: List[JointTrajectoryPoint] = []
        self.i = 0

        # IK ソルバー設定
        self.ik_solver = ik_solver
        pkg_share_dir = get_package_share_directory(robot_urdf_package)
        urdf_path = f"{pkg_share_dir}/urdf/{robot_urdf_file_name}.urdf"
        self.declare_parameters(
            namespace='',
            parameters=[
                ('trac_ik_timeout', 0.02),
                ('trac_ik_epsilon', 1e-5),
                ('trac_ik_solver_type', "Distance"),
            ]
        )

        if self.ik_solver == IKType.TRACK_IK:
            self.get_logger().info("TRACK IK selected")
            self.trac_ik_timeout = self.get_parameter('trac_ik_timeout').value
            self.trac_ik_epsilon = self.get_parameter('trac_ik_epsilon').value
            self.trac_ik_solver_type = self.get_parameter('trac_ik_solver_type').value
            self._init_ik_solver(base_link, tool_link, urdf_path)
        else:
            raise Exception("unsupported ik_solver: ", self.ik_solver.name)

    def _init_ik_solver(self, base_link: str, ee_link: str, urdf_path: str) -> None:
        """
        Trac-IK ソルバーの初期化

        Args:
            base_link: ベースリンク名
            ee_link: エンドエフェクタリンク名
            urdf_path: URDF のパス
        """
        if self.ik_solver == IKType.TRACK_IK:
            try:
                self.trac_ik = TRACK_IK_SOLVER(
                    base_link_name=base_link,
                    tip_link_name=ee_link,
                    urdf_path=urdf_path,
                    timeout=self.trac_ik_timeout,
                    epsilon=self.trac_ik_epsilon,
                    solver_type=self.trac_ik_solver_type
                )
            except Exception as e:
                self.get_logger().error(f"Could not instantiate TRAC_IK: {e}")
        else:
            raise Exception("unsupported ik_solver: ", self.ik_solver.name)

    def solve_ik(self, pose: List[float], q_init: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        指定したポーズに対して IK を解く

        Args:
            pose: [x, y, z, qx, qy, qz, qw] のリスト
            q_init: 初期ジョイント値。未指定の場合はゼロ埋めする

        Returns:
            正常な解が得られた場合はジョイント角度のリスト、解が見つからなければ None 
        """
        self.get_logger().debug(f"Input pose: {pose} | pos: {pose[:3]} | rot: {pose[3:]}")
        if self.ik_solver == IKType.TRACK_IK:
            try:
                if q_init is None:
                    q_init = [0.0] * len(self.joints_name)
                joint_positions = self.trac_ik.ik(pose[:3], pose[3:], seed_jnt_values=q_init)
                if joint_positions is not None:
                    return joint_positions
                else:
                    self.get_logger().warn("No IK solution found.")
                    return None
            except Exception as e:
                self.get_logger().error(f"Could not solve IK: {e}")
                return None
        else:
            raise Exception("unsupported ik_solver: ", self.ik_solver.name)

    def set_waypoints(self, waypoints: List[List[float]], time_to_reach: int,
                      send_immediately: bool = False, wait: bool = True) -> None:
        """
        複数のウェイポイントに対して軌道を生成し送信する

        Args:
            waypoints: 各ウェイポイントは [x, y, z, qx, qy, qz, qw]
            time_to_reach: 各ウェイポイントに到達する時間 (秒)
            send_immediately: True の場合、生成と同時に送信する
            wait: True の場合、ゴール結果を待つ
        """
        joint_trajectory = []
        for waypoint in waypoints:
            joint_positions = self.solve_ik(waypoint)
            if joint_positions is not None:
                joint_trajectory.append(joint_positions)
            else:
                self.get_logger().warn("No joint positions found for a waypoint.")
                return  # 途中で解が得られない場合は中断

        self.set_joint_trajectory(joint_trajectory, time_to_reach, send_immediately, wait)

    def set_goal_pose(self, goal_pose: List[float], time_to_reach: int,
                      send_immediately: bool = False, wait: bool = True) -> None:
        """
        単一のゴールポーズに対して軌道を生成し送信する

        Args:
            goal_pose: ゴールポーズ [x, y, z, qx, qy, qz, qw]
            time_to_reach: 到達にかかる時間 (秒)
            send_immediately: True の場合、生成と同時に送信する
            wait: True の場合、ゴール結果を待つ
        """
        joint_positions = self.solve_ik(goal_pose)
        if joint_positions is not None:
            self.set_joint_trajectory([joint_positions], time_to_reach, send_immediately, wait)
        else:
            self.get_logger().warn("No joint positions found for goal pose.")

    def set_joint_trajectory(self, joint_trajectory: List[List[float]], time_to_reach: int,
                             send_immediately: bool = False, wait: bool = True) -> None:
        """
        生成済みの joint trajectory をアクションとして送信する

        Args:
            joint_trajectory: 各リストが各ウェイポイントのジョイント角度を表す
            time_to_reach: 各ウェイポイントに到達する時間 (秒)
            send_immediately: True の場合、生成と同時に送信する
            wait: True の場合、ゴール結果を待つ
        """
        if not joint_trajectory or len(joint_trajectory[0]) != len(self.joints_name):
            self.get_logger().error("Joint trajectory has an unexpected number of joint positions.")
            return

        self.goals.clear()
        for goal_joints in joint_trajectory:
            point = JointTrajectoryPoint()
            point.positions = goal_joints
            point.time_from_start = Duration(sec=time_to_reach)
            self.goals.append(point)
        self.i = 0

        self.get_logger().info(f"Joint trajectory: {joint_trajectory}")
        if send_immediately:
            self._send_joint_trajectory_goal(wait)

    def _send_joint_trajectory_goal(self, wait: bool = True) -> None:
        """
        アクションクライアントへゴールを送信する

        Args:
            wait: ゴール結果を待つかどうか
        """
        if not self.goals:
            self.get_logger().warn("No goals set to send.")
            return

        self.get_logger().info(f"Sending goal {self.goals[self.i]}.")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joints_name
        goal_msg.trajectory.points = [self.goals[self.i]]

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        if wait:
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                get_result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, get_result_future)
                result = get_result_future.result()
                self.get_logger().info(f"Result: {result}")
            else:
                self.get_logger().info('Goal rejected.')

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info(f"Result: {result}")

    def _feedback_callback(self, feedback_msg) -> None:
        # フィードバックは必要に応じて処理
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Received feedback: {feedback}")


def main(args: Optional[List[str]] = None) -> None:
    """
    ArmPositionController ノードを初期化し、インタラクティブにテストする
    """
    rclpy.init(args=args)
    arm_controller = ArmPositionController(
        controller_name="scaled_joint_trajectory_controller",
        joints_name=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
    )

    while rclpy.ok():
        print("\nMenu:")
        print("0. Exit")
        print("1. Solve IK")
        print("2. Test JTC with one target pose")
        print("3. Test JTC with waypoints")
        choice = input("Enter your choice: ")

        if choice == '1':
            # 例： [x, y, z, qx, qy, qz, qw]
            target_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
            joint_positions = arm_controller.solve_ik(target_pose)
            if joint_positions is not None:
                print(f"IK solution: {joint_positions}")
            else:
                print("No IK solution found.")
        elif choice == '2':
            target_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
            time_to_reach = 5
            arm_controller.set_goal_pose(target_pose, time_to_reach, send_immediately=True)
        elif choice == '3':
            waypoints = [
                [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0],
                [0.6, 0.1, 0.6, 0.0, 0.0, 0.0, 1.0],
                [0.7, 0.2, 0.7, 0.0, 0.0, 0.0, 1.0]
            ]
            time_to_reach = 5
            arm_controller.set_waypoints(waypoints, time_to_reach, send_immediately=True)
        elif choice == '0':
            break
        else:
            print("Invalid choice. Please try again.")

        rclpy.spin_once(arm_controller, timeout_sec=0.1)

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
