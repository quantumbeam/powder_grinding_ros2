#!/home/ubuntu/user/grinding_ws/venv/bin/python3
import sys
sys.path.append('/home/ubuntu/user/grinding_ws/venv/lib/python3.10/site-packages')

import time
from enum import Enum, auto
from typing import List, Optional, Union

import numpy as np
from tqdm import tqdm

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

# パスを通して pytracik を参照
sys.path.append('/home/ubuntu/user/grinding_ws/src/powder_grinding/pytracik')
from pytracik.trac_ik import TracIK as TRACK_IK_SOLVER


class IKType(Enum):
    TRACK_IK = auto()
    # 今後、他の IK ソルバーを追加する場合はこちらに拡張する


class JointTrajectoryControllerHelper(Node):
    """
    アームの位置制御および IK 計算、軌道生成を補助するノード
    """
    def __init__(self,
                 controller_name: str,
                 joints_name: List[str],
                 tool_link: str = "tool0",
                 base_link: str = "base_link",
                 robot_urdf_package: str = "grinding_robot_description",
                 robot_urdf_file_name: str = "ur5e_with_pestle",
                 ik_solver: IKType = IKType.TRACK_IK) -> None:
        super().__init__('arm_position_controller')
        
        self.controller_name = controller_name
        self.valid_joint_names = joints_name
        if not self.valid_joint_names:
            raise Exception('"joints" parameter is not set!')

        # アクションクライアント設定
        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.goals: List[JointTrajectoryPoint] = []

        # IK ソルバー設定
        self.ik_solver = ik_solver
        self.base_link = base_link
        self.tool_link = tool_link
        pkg_share_dir = get_package_share_directory(robot_urdf_package)
        self.urdf_path = f"{pkg_share_dir}/urdf/{robot_urdf_file_name}.urdf"
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
            self._init_ik_solver(base_link, tool_link)
        else:
            raise Exception(f"Unsupported IK solver: {self.ik_solver.name}")

        # Joint State サブスクライバーの設定
        self._current_jnt_positions = None
        self.first_joint_state_received = Future()
        self.create_joint_state_subscription()

    def _joint_states_cb(self, msg: JointState) -> None:
        """
        /joint_states トピック受信時のコールバック
        """
        # 最初の受信は Future を完了させる
        if not self.first_joint_state_received.done():
            self.first_joint_state_received.set_result(True)
        pos, vel, eff, names = [], [], [], []
        for joint in self.valid_joint_names:
            if joint in msg.name:
                idx = msg.name.index(joint)
                names.append(msg.name[idx])
                eff.append(msg.effort[idx])
                vel.append(msg.velocity[idx])
                pos.append(msg.position[idx])
        if set(names) == set(self.valid_joint_names):
            self._current_jnt_positions = np.array(pos)
            self._current_jnt_velocities = np.array(vel)
            self._current_jnt_efforts = np.array(eff)
            self._joint_names = names

    def create_joint_state_subscription(self, timeout_sec: int = 10) -> bool:
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',  # 絶対パスで指定
            self._joint_states_cb,
            10  # QoS depth
        )
        self.get_logger().info("Waiting for joint state subscription...")
        rclpy.spin_until_future_complete(self, self.first_joint_state_received, timeout_sec=timeout_sec)
        if self.first_joint_state_received.done():
            self.get_logger().info("Joint state subscription created successfully.")
            return True
        else:
            self.get_logger().error("Failed to create joint state subscription within timeout.")
            self.destroy_subscription(self.subscription)
            return False

    def get_joint_names(self) -> List[str]:
        """現在のジョイント名のリストを返す"""
        return self.valid_joint_names

    def get_current_joint_positions(self) -> Optional[List[float]]:
        """最新のジョイント位置をリストとして返す"""
        if self._current_jnt_positions is None:
            self.get_logger().warn("Joint state not received yet.")
            return None
        return self._current_jnt_positions.tolist()

    def _init_ik_solver(self, base_link: str, ee_link: str, urdf_path: str=None) -> None:
        """
        Trac-IK ソルバーの初期化
        """
        if urdf_path is None:
            urdf_path = self.urdf_path
        self.get_logger().info(f"Initializing IK solver with base link: {base_link}, end effector link: {ee_link}")
        self.get_logger().info(f"URDF path: {urdf_path}")
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
            raise Exception(f"Unsupported IK solver: {self.ik_solver.name}")
        
    def change_ee_link(self, new_ee_link: str) -> None:
        """
        エンドエフェクタリンクを変更する
        """
        self.tool_link = new_ee_link
        self.get_logger().info(f"Changing end effector link to: {new_ee_link}")
        self._init_ik_solver(
            base_link=self.base_link,
            ee_link=new_ee_link,
            urdf_path=self.urdf_path
        )
        

    def solve_ik(self, pose: List[float],
                 q_init: Optional[List[float]] = None,
                 number_of_attempts: int = 100) -> Optional[List[float]]:
        """
        指定したポーズに対して IK を解く
        """
        self.get_logger().debug(f"Input pose: {pose} | pos: {pose[:3]} | rot: {pose[3:]}")
        for _ in range(number_of_attempts):
            if self.ik_solver == IKType.TRACK_IK:
                try:
                    if q_init is None:
                        q_init = self.get_current_joint_positions()
                        if q_init is None:
                            self.get_logger().warn("Current joint positions not available, using zero initialization.")
                            q_init = [0.0] * len(self.valid_joint_names)
                    joint_positions = self.trac_ik.ik(pose[:3], pose[3:], seed_jnt_values=q_init)
                    if joint_positions.size == len(self.valid_joint_names):
                        return joint_positions
                    else:
                        self.get_logger().warn("No IK solution found.")
                        return None
                except Exception as e:
                    self.get_logger().error(f"Could not solve IK: {e}")
                    return None
            else:
                raise Exception(f"Unsupported IK solver: {self.ik_solver.name}")
    
    def set_waypoints(self,
                      waypoints: List[List[float]],
                      time_to_reach: int,
                      max_joint_change_per_step: Optional[float] = 0.1, # OptionalにしてNone許容、デフォルト値を少し大きく
                      max_ik_retries_on_jump: int = 100, # 閾値超過時のIKリトライ回数上限
                      ik_retry_perturbation: float = 0.05, # IKリトライ時のシード値の摂動量 (ラジアン)
                      send_immediately: bool = False,
                      wait: bool = True) -> None:
        """
        複数のウェイポイントに対して軌道を生成し送信する

        Args:
            waypoints: 目標姿勢 [x, y, z, qx, qy, qz, qw] のリスト
            time_to_reach: 全体の目標到達時間 (秒)
            max_joint_change_per_step: 連続するウェイポイント間で許容される
                                       各ジョイントの最大角度変化量 (ラジアン)。
                                       None の場合はチェックしない。
            max_ik_retries_on_jump: max_joint_change_per_step を超えた場合のIKリトライ回数上限
            ik_retry_perturbation: IKリトライ時にシード値に加える摂動の最大量 (ラジアン)
            send_immediately: すぐに軌道を送信するかどうか
            wait: 軌道実行の完了を待つかどうか
        """
        joint_trajectory = []
        q_prev = self.get_current_joint_positions() # 前のステップのジョイント角度 (最初は現在値)

        # Check if initial position is valid before starting the loop
        if q_prev is None:
             self.get_logger().error("Cannot start waypoint calculation: Initial joint positions not available.")
             return
        q_prev = np.array(q_prev) # numpy 配列に変換

        all_waypoints_solved = True # Flag to track if all waypoints were solved
        for i, waypoint in enumerate(tqdm(waypoints, desc="Calculating IK for waypoints"), start=1):

            solved_within_limit = False
            best_solution_found = None # リトライで見つかった最良の解を保持

            # --- IK計算とリトライループ ---
            for attempt in range(max_ik_retries_on_jump + 1): # 初回 + リトライ回数

                # IK計算に使用するシード値 (初回は q_prev, リトライ時は摂動を加える)
                if attempt == 0:
                    q_seed = q_prev.tolist() # solve_ik はリストを期待
                else:
                    # q_prev にランダムな摂動を加える
                    perturbation = np.random.uniform(-ik_retry_perturbation, ik_retry_perturbation, size=q_prev.shape)
                    q_seed_np = q_prev + perturbation
                    # オプション: ジョイントリミット内にクリップする処理を追加しても良い
                    q_seed = q_seed_np.tolist()

                # IK計算実行
                current_joint_positions_np = self.solve_ik(waypoint, q_init=q_seed)

                if current_joint_positions_np is not None:
                    # IK成功
                    if current_joint_positions_np.size == len(self.valid_joint_names):
                        # サイズチェックOK
                        best_solution_found = current_joint_positions_np # 少なくとも解は見つかった

                        # 角度変化量チェック (max_joint_change_per_step が指定されている場合のみ)
                        if max_joint_change_per_step is not None:
                            joint_change = np.abs(current_joint_positions_np - q_prev)
                            if np.any(joint_change > max_joint_change_per_step):
                                # 閾値超え -> リトライへ (ループの次の反復へ)
                                self.get_logger().debug(
                                    f"Joint change exceeds threshold for waypoint [{i}] (Attempt {attempt}). "
                                    f"Max change: {np.max(joint_change):.3f} > {max_joint_change_per_step:.3f} rad."
                                )
                                continue # 次のリトライへ
                            else:
                                # 閾値内 -> 成功
                                solved_within_limit = True
                                break # リトライループを抜ける (最適な解が見つかった)
                        else:
                            # 角度変化チェックなし -> 成功
                            solved_within_limit = True
                            break # リトライループを抜ける
                    else:
                        # IK成功したがサイズが不正 (solve_ik内で警告が出るはず)
                        self.get_logger().error(
                            f"IK solution for waypoint [{i}] (Attempt {attempt}) has unexpected size "
                            f"({current_joint_positions_np.size})."
                        )
                        # この場合はリトライしても無駄なのでループを抜ける
                        best_solution_found = None # 失敗扱い
                        break
                else:
                    # IK失敗 (solve_ik内で警告が出るはず)
                    self.get_logger().warn(f"IK failed for waypoint [{i}] (Attempt {attempt}).")
                    # リトライを続ける (別のシードで成功するかもしれない)
                    # continue は不要 (ループの次の反復へ)

            # --- リトライループの結果処理 ---
            if solved_within_limit and best_solution_found is not None:
                # 閾値内でIK成功
                joint_trajectory.append(best_solution_found.tolist())
                q_prev = best_solution_found # 次のステップの q_prev を更新
            else:
                # リトライしても閾値内の解が見つからない or IK自体が失敗
                if best_solution_found is not None: # IKは解けたが閾値を超えた場合
                     self.get_logger().error(
                         f"Failed to find IK solution within joint change limit ({max_joint_change_per_step:.3f} rad) "
                         f"for waypoint [{i}] after {max_ik_retries_on_jump} retries. "
                         f"Last valid solution had max change: {np.max(np.abs(best_solution_found - q_prev)):.3f} rad. "
                         "Stopping trajectory generation."
                     )
                else: # IKが一度も成功しなかった場合
                     self.get_logger().error(
                         f"IK failed for waypoint [{i}] after {max_ik_retries_on_jump + 1} attempts. "
                         "Stopping trajectory generation."
                     )
                all_waypoints_solved = False
                break # 外側の for ループ (ウェイポイントのループ) を抜ける

        # --- 軌道送信処理 (変更なし) ---
        if all_waypoints_solved and joint_trajectory:
            self.get_logger().info(f"Successfully calculated joint trajectory for {len(joint_trajectory)} waypoints.")
            self.set_joint_trajectory(joint_trajectory, time_to_reach, send_immediately, wait)
        elif not joint_trajectory:
             self.get_logger().error("Trajectory generation failed: No valid joint positions calculated.")
        else:
             self.get_logger().error("Trajectory generation incomplete due to IK failure(s) or joint change limit exceeded. Trajectory not sent.")



    def set_goal_pose(self, goal_pose: List[float],
                      time_to_reach: int,
                      send_immediately: bool = False,
                      wait: bool = True) -> None:
        """
        単一のゴールポーズに対して軌道を生成し送信する
        """
        joint_positions = self.solve_ik(goal_pose)

        # --- FIX: Check if solve_ik returned a valid result ---
        if joint_positions is not None:
            # IK が成功した場合のみサイズチェックと軌道設定を行う
            if joint_positions.size == len(self.valid_joint_names):
                # Convert numpy array to list for set_joint_trajectory if needed
                self.set_joint_trajectory([joint_positions.tolist()], time_to_reach, send_immediately, wait)
            else:
                # This case indicates an unexpected issue in solve_ik logic
                 self.get_logger().error(f"IK solution for goal pose has unexpected size ({joint_positions.size}). Trajectory not sent.")
        else:
            # solve_ik returned None (IK failed)
            self.get_logger().warn(f"No joint positions found for goal pose: {goal_pose}. Trajectory not sent.")



    def set_joint_trajectory(self, joint_trajectory: List[List[float]],
                             time_to_reach: int,
                             send_immediately: bool = False,
                             wait: bool = True) -> None:
        """
        生成済みの joint trajectory をアクションクライアントに送信する
        """
        if not joint_trajectory or len(joint_trajectory[0]) != len(self.valid_joint_names):
            self.get_logger().error("Joint trajectory has an unexpected number of joint positions.")
            return
        
        dt = float(time_to_reach) / len(joint_trajectory)
        self.goals.clear()
        for i, goal_joints in enumerate(joint_trajectory, start=1):
            point = JointTrajectoryPoint()
            point.positions = goal_joints
            sec = int(dt * i)
            nsec = int((dt * i - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            self.goals.append(point)
        self.get_logger().debug(f"All joint trajectory: {self.goals}")
        self.get_logger().info(f"Joint trajectory [0]: {joint_trajectory[0]}")
        if send_immediately:
            self.send_joint_trajectory_goal(wait)

    def send_joint_trajectory_goal(self, wait: bool = True) -> None:
        """
        アクションクライアントにゴールを送信する
        """
        if not self.goals:
            self.get_logger().warn("No goals set to send.")
            return

        self.get_logger().debug(f"Sending goal with trajectory: {self.goals}")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.valid_joint_names
        goal_msg.trajectory.points = self.goals

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
                self.get_logger().info("Goal rejected.")

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return
        self.get_logger().info("Goal accepted.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future) -> None:
        result = future.result().result
        self.get_logger().info(f"Result: {result}")

    def _feedback_callback(self, feedback_msg) -> None:
        self.get_logger().debug(f"Received feedback: {feedback_msg.feedback}")

    def clear_points(self) -> None:
        """
        ジョイントトラジェクトリのポイントをクリアする
        """
        self.goals.clear()
        self.get_logger().info("Cleared all points.")


def main(args: Optional[List[str]] = None) -> None:
    """
    ノードを初期化し、インタラクティブにテスト実行する
    """
    rclpy.init(args=args)
    arm_controller = JointTrajectoryControllerHelper(
        controller_name="scaled_joint_trajectory_controller",
        joints_name=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ],
        tool_link="pestle_tip",
        base_link="base_link",
        robot_urdf_package="grinding_robot_description",
        robot_urdf_file_name="ur5e_with_pestle",
        ik_solver=IKType.TRACK_IK
    )

    while rclpy.ok():
        print("\nMenu:")
        print("0. Exit")
        print("1. Solve IK")
        print("2. Test JTC with one target pose")
        print("3. Test JTC with waypoints")
        print("4. Test grinding motion")
        print("10. Get joint names")
        print("11. Get current joint positions")
        choice = input("Enter your choice: ")

        if choice == '0':
            break
        elif choice == '1':
            target_pose = [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
            joint_positions = arm_controller.solve_ik(target_pose)
            print(f"IK solution: {joint_positions}")
        elif choice == '2':
            target_pose = [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
            arm_controller.set_goal_pose(target_pose, time_to_reach=5, send_immediately=True)
        elif choice == '3':
            waypoints = [
                [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0],
                [0.5, 0.1, 0.6, 1.0, 0.0, 0.0, 0.0]
            ]
            arm_controller.set_waypoints(waypoints, time_to_reach=5, send_immediately=True)
        elif choice == '4':
            print("Testing grinding motion ...")
            try:
                from grinding_motion_routines.grinding_motion_generator import GrindingMotionGenerator
                from grinding_motion_routines.display_marker import DisplayMarker
            except ImportError as e:
                print(f"ImportError: {e}")
                continue

            mortar_inner_size = {"x": 0.04, "y": 0.04, "z": 0.035}
            mortar_top_position = {
                "x": -0.24487948173594054,
                "y": 0.3722676198635453,
                "z": 0.045105853329747,
            }
            grinding_pos_beginning = [-8, 0]
            grinding_pos_end = [-8, 0.001]
            number_of_rotations = 1
            angle_scale = 1
            yaw_bias = 0
            yaw_twist_per_rotation = np.pi / 2
            number_of_waypoints_per_circle = 50
            center_position = [0, 0]
            sec_per_rotation = 1

            motion_generator = GrindingMotionGenerator(mortar_top_position, mortar_inner_size)
            try:
                waypoints = motion_generator.create_circular_waypoints(
                    beginning_position_mm=grinding_pos_beginning,
                    end_position_mm=grinding_pos_end,
                    number_of_rotations=number_of_rotations,
                    number_of_waypoints_per_circle=number_of_waypoints_per_circle,
                    angle_scale=angle_scale,
                    yaw_bias=yaw_bias,
                    yaw_twist_per_rotation=yaw_twist_per_rotation,
                    center_position_mm=center_position
                )
            except ValueError as e:
                print(f"Error generating circular waypoints: {e}")
                continue
            print("Generated grinding motion waypoints")
            print(f"Number of waypoints: {len(waypoints)}")
            print(f"Waypoints: {waypoints}")
            print("Displaying waypoints ...")
            display_marker = DisplayMarker()
            display_marker.display_waypoints(waypoints, scale=0.002)
            print("Go to first grinding position")
            arm_controller.set_goal_pose(waypoints[0], time_to_reach=5, send_immediately=True)
            print(f"Executing grinding motion with {len(waypoints)} waypoints")
            total_time = sec_per_rotation * number_of_rotations
            arm_controller.set_waypoints(waypoints, time_to_reach=total_time, send_immediately=True)
            print("Grinding motion test completed.")
        elif choice == '10':
            print(f"Joint names: {arm_controller.get_joint_names()}")
        elif choice == '11':
            current_joint_positions = arm_controller.get_current_joint_positions()
            print(f"Current joint positions: {current_joint_positions}")
        else:
            print("Invalid choice. Please try again.")

        rclpy.spin_once(arm_controller)

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()