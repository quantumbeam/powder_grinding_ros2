#!/usr/bin/env python3

import sys
import time
from enum import Enum, auto
from typing import List, Optional, Union

import numpy as np
from tqdm import tqdm

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R # For FK, matrix to quaternion

from pytracik.trac_ik import TracIK as TRACK_IK_SOLVER


class IKType(Enum):
    TRACK_IK = auto()
    # 今後、他の IK ソルバーを追加する場合はこちらに拡張する


class JointChangeCheckMode(Enum):
    ALL_INDIVIDUAL_WITHIN_LIMIT = auto()  # 全ての関節が個別の閾値内
    SUM_TOTAL_WITHIN_LIMIT = auto()  # 全関節の変化量の合計が閾値内
    # 今後、他の IK ソルバーを追加する場合はこちらに拡張する


class JointTrajectoryControllerHelper(Node):
    """
    ROS 2のJointTrajectoryControllerと連携し、ロボットアームの制御を補助するクラス。

    主な機能:
    - JointTrajectoryControllerへのアクションクライアントを提供し、関節軌道を送信します。
    - Trac-IKライブラリを利用した逆運動学 (IK) 計算機能を提供します。
    - 現在の関節状態 (/joint_states) を購読し、IK計算のシード値などに利用します。
    - 指定されたエンドエフェクタリンクに基づいてIKソルバーを初期化・変更する機能を持ちます。
    - デカルト座標系のウェイポイントリストから軌道を生成し、関節角度の急変をチェックしながら
      安全な動作を試みます。

    使用目的:
    - ロボットアームに対して、デカルト座標系での目標姿勢やウェイポイント列を指定して動作させる。
    - 異なるエンドエフェクタ（ツール）に合わせたIK計算と動作実行を容易にする。
    - 複雑な軌道生成や動作シーケンスの基盤として利用する。
    """

    def __init__(
        self,
        controller_name: str,
        joints_name: List[str],
        tool_link: str = "tool0",
        base_link: str = "base_link",
        robot_urdf_file_path: str = None,
        ik_solver: IKType = IKType.TRACK_IK,
        node_name: str = "arm_position_controller",
    ) -> None:
        super().__init__(node_name)

        self.controller_name = controller_name
        self.valid_joint_names = joints_name
        if not self.valid_joint_names:
            raise Exception('"joints" parameter is not set!')
        self.num_joints = len(self.valid_joint_names)


        # アクションクライアント設定
        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.goals: List[JointTrajectoryPoint] = []

        # IK ソルバー設定
        self.ik_solver = ik_solver
        self.base_link = base_link
        self.tool_link = tool_link
        
        # URDFファイルパスの設定
        if robot_urdf_file_path is None:
            # /robot_descriptionからURDFを取得
            self.urdf_path = None
            self.urdf_string = None
            try:
                # robot_descriptionパラメータの宣言と取得を試行
                self.declare_parameter('robot_description', '')
                urdf_param = self.get_parameter('robot_description')
                if urdf_param.type_ == urdf_param.Type.STRING:
                    self.urdf_string = urdf_param.get_parameter_value().string_value
                    if self.urdf_string:
                        self.get_logger().info("Successfully retrieved URDF from /robot_description parameter")
                    else:
                        self.get_logger().warn("robot_description parameter is empty")
                else:
                    self.get_logger().warn("robot_description parameter is not a string")
            except Exception as e:
                self.get_logger().warn(f"Could not retrieve robot_description parameter: {e}")
                self.urdf_string = None
        else:
            self.urdf_path = robot_urdf_file_path
            self.urdf_string = None
        self.declare_parameters(
            namespace="",
            parameters=[
                ("trac_ik_timeout", 0.02),
                ("trac_ik_epsilon", 1e-5),
                ("trac_ik_solver_type", "Distance"),
                ("default_cartesian_speed_for_pose", 0.03), # m/s, for single pose movement time calculation
                ("min_time_to_reach_for_pose", 1.0),   # seconds, minimum time for single pose movement
            ],
        )

        # KDL Helperを初期化（ヤコビアン計算とFK用）
        self.kdl_helper = None
        try:
            if self.urdf_path:
                from .kdl_helper import KDLHelper
                self.kdl_helper = KDLHelper(self.get_logger(), urdf_path=self.urdf_path, 
                                            base_link=self.base_link, ee_link=self.tool_link)
            elif self.urdf_string:
                from .kdl_helper import KDLHelper
                self.kdl_helper = KDLHelper(self.get_logger(), urdf_string=self.urdf_string, 
                                            base_link=self.base_link, ee_link=self.tool_link)
            else:
                self.get_logger().info("No URDF available for KDL helper initialization.")
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize KDL helper: {e}")
            self.kdl_helper = None
        if self.ik_solver == IKType.TRACK_IK:
            self.get_logger().info("TRACK IK selected")
            self.trac_ik_timeout = self.get_parameter("trac_ik_timeout").value
            self.trac_ik_epsilon = self.get_parameter("trac_ik_epsilon").value
            self.trac_ik_solver_type = self.get_parameter("trac_ik_solver_type").value
            self.default_cartesian_speed_for_pose = self.get_parameter(
                "default_cartesian_speed_for_pose"
            ).value
            self.min_time_to_reach_for_pose = self.get_parameter(
                "min_time_to_reach_for_pose"
            ).value
            self._init_ik_solver()
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
        else:
            self.get_logger().warn(
                f"Received joint states do not match expected names: {set(names)} != {set(self.valid_joint_names)}"
            )

    def create_joint_state_subscription(self, timeout_sec: int = 10) -> bool:
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",  # 絶対パスで指定
            self._joint_states_cb,
            10,  # QoS depth
        )
        self.get_logger().info("Waiting for joint state subscription...")
        rclpy.spin_until_future_complete(
            self, self.first_joint_state_received, timeout_sec=timeout_sec
        )
        if self.first_joint_state_received.done():
            self.get_logger().info("Joint state subscription created successfully.")
            return True
        else:
            self.get_logger().error(
                "Failed to create joint state subscription within timeout."
            )
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
        return self._current_jnt_positions

    def _init_ik_solver(self) -> None:
        """
        Trac-IK ソルバーの初期化
        """
        urdf_path = self.urdf_path
        self.get_logger().info(
            f"Initializing IK solver with base link: {self.base_link}, end effector link: {self.tool_link}"
        )
        
        if self.ik_solver == IKType.TRACK_IK:
            try:
                if urdf_path is None and self.urdf_string is not None:
                    # /robot_descriptionパラメータからURDFを使用
                    self.get_logger().info("Using /robot_description parameter for URDF")
                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link_name=self.base_link,
                        tip_link_name=self.tool_link,
                        urdf_string=self.urdf_string,
                        timeout=self.trac_ik_timeout,
                        epsilon=self.trac_ik_epsilon,
                        solver_type=self.trac_ik_solver_type,
                    )
                elif urdf_path is not None:
                    self.get_logger().info(f"Using URDF file: {urdf_path}")
                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link_name=self.base_link,
                        tip_link_name=self.tool_link,
                        urdf_path=urdf_path,
                        timeout=self.trac_ik_timeout,
                        epsilon=self.trac_ik_epsilon,
                        solver_type=self.trac_ik_solver_type,
                    )
                else:
                    raise ValueError("No URDF available: neither urdf_path nor urdf_string is set")
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
        self._init_ik_solver()

    def solve_fk(self, joint_positions: List[float]) -> Optional[List[float]]:
        """
        指定した関節角度に対して FK を解き、デカルト座標 [x,y,z,qx,qy,qz,qw] を返す。
        """
        if self.ik_solver == IKType.TRACK_IK:
            try:
                if not isinstance(joint_positions, np.ndarray):
                    joint_positions_np = np.array(joint_positions)
                else:
                    joint_positions_np = joint_positions

                pos_vec, rot_mat = self.trac_ik.fk(joint_positions_np)
                
                # 回転行列をクォータニオンに変換 (scipy)
                rotation = R.from_matrix(rot_mat)
                quat = rotation.as_quat() # Returns [x, y, z, w]

                return [pos_vec[0], pos_vec[1], pos_vec[2], quat[0], quat[1], quat[2], quat[3]]
            except Exception as e:
                self.get_logger().error(f"Could not solve FK: {e}")
                return None
        else:
            self.get_logger().warn(f"FK not supported for solver: {self.ik_solver.name}")
            return None

    def solve_ik(
        self,
        pose: List[float],
        q_init: Optional[List[float]] = None,
        number_of_attempts: int = 100,
    ) -> Optional[List[float]]:
        """
        指定したポーズに対して IK を解く
        """
        self.get_logger().debug(
            f"Input pose: {pose} | pos: {pose[:3]} | rot: {pose[3:]}"
        )
        for _ in range(number_of_attempts):
            if self.ik_solver == IKType.TRACK_IK:
                try:
                    if q_init is None:
                        q_init = self.get_current_joint_positions()
                        if q_init is None:
                            self.get_logger().warn(
                                "Current joint positions not available, using zero initialization."
                            )
                            q_init = [0.0] * len(self.valid_joint_names)
                    joint_positions = self.trac_ik.ik(
                        pose[:3], pose[3:], seed_jnt_values=q_init
                    )
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

    def _is_joint_change_within_limit(
        self,
        current_joint_positions: np.ndarray,
        previous_joint_positions: np.ndarray,
        check_mode: JointChangeCheckMode,
        max_joint_change: float,
        waypoint_index: int,
        attempt_number: int,
        num_axes_to_check: Optional[int] = None,  # チェックする軸数を追加
    ) -> bool:
        """
        連続するウェイポイント間のジョイント角度変化が許容範囲内かチェックする。

        Args:
            current_joint_positions: 現在のウェイポイントのIK解 (numpy配列)。
            previous_joint_positions: 前のウェイポイントのIK解 (numpy配列)。
            check_mode: チェックモード (ALL_INDIVIDUAL_WITHIN_LIMIT または SUM_TOTAL_WITHIN_LIMIT)。Best joint change
            max_joint_change: ALL_INDIVIDUAL_WITHIN_LIMIT モード時の許容される最大ジョイント角度変化量 (ラジアン)。
                                         Noneの場合はチェックしない。
            max_joint_change: SUM_TOTAL_WITHIN_LIMIT モード時の許容される全関節の合計最大角度変化量 (ラジアン)。
                                    Noneの場合はチェックしない。
            waypoint_index: 現在のウェイポイントのインデックス (ログ用)。
            attempt_number: 現在のIKリトライ試行回数 (ログ用)。
            num_axes_to_check: チェック対象とする関節の数。Noneの場合は全関節をチェック。

        Returns:
            角度変化が許容範囲内であれば True、そうでなければ False。
        """
        if num_axes_to_check is None or num_axes_to_check > len(current_joint_positions):
            num_axes_to_check = len(current_joint_positions) # 全軸チェック
        
        axes_to_check_str = f"first {num_axes_to_check} axes" if num_axes_to_check < len(current_joint_positions) else "all axes"

        current_positions_to_check = current_joint_positions[:num_axes_to_check]
        previous_positions_to_check = previous_joint_positions[:num_axes_to_check]

        joint_change = np.abs(current_positions_to_check - previous_positions_to_check)

        if check_mode == JointChangeCheckMode.ALL_INDIVIDUAL_WITHIN_LIMIT:
            if np.any(joint_change > max_joint_change):
                self.get_logger().error(
                    f"[Mode: ALL_INDIVIDUAL, Axes: {axes_to_check_str}, Max Change Axis No: {np.argmax(joint_change)+1}] "
                    f"Joint change exceeds individual threshold for waypoint [{waypoint_index}] (Attempt {attempt_number}). "
                    f"Max individual change: {np.max(joint_change):.3f} > {max_joint_change:.3f} rad."
                )
                return False
            self.get_logger().info(
                f"[Mode: ALL_INDIVIDUAL, Axes: {axes_to_check_str}] "
                f"Joint change within individual limits for waypoint [{waypoint_index}] (Attempt {attempt_number}). "
                f"Max individual change: {np.max(joint_change):.3f} <= {max_joint_change:.3f} rad."
            )
            return True
        elif check_mode == JointChangeCheckMode.SUM_TOTAL_WITHIN_LIMIT:
            current_total_joint_change = np.sum(joint_change)
            if current_total_joint_change > max_joint_change:
                self.get_logger().error(
                    f"[Mode: SUM_TOTAL, Axes: {axes_to_check_str}] "
                    f"Total joint change exceeds threshold for waypoint [{waypoint_index}] (Attempt {attempt_number}). "
                    f"Total change: {current_total_joint_change:.3f} > {max_joint_change:.3f} rad."
                )
                return False
            self.get_logger().debug(
                f"[Mode: SUM_TOTAL, Axes: {axes_to_check_str}] "
                f"Total joint change within limits for waypoint [{waypoint_index}] (Attempt {attempt_number}). "
                f"Total change: {current_total_joint_change:.3f} <= {max_joint_change:.3f} rad."
            )
            return True
        else:
            self.get_logger().error(
                f"Invalid JointChangeCheckMode: {check_mode}. Assuming check failed."
            )
            return False

    def set_waypoints(
        self,
        waypoints: List[List[float]],
        time_to_reach: float,
        q_init: Optional[List[float]] = None,
        max_joint_change_per_step: float = np.pi/8,
        max_ik_retries_on_jump: int = 100,  # 閾値超過時のIKリトライ回数上限
        ik_retry_perturbation: float = 0.05,  # IKリトライ時のシード値の摂動量 (ラジアン)
        send_immediately: bool = False,
        wait: bool = True,
    ) -> None:
        """
        複数のウェイポイントに対して軌道を生成し送信する

        Args:
            waypoints: 目標姿勢 [x, y, z, qx, qy, qz, qw] のリスト
            time_to_reach: 全体の目標到達時間 (秒)
            max_joint_change_per_step: 連続するウェイポイント間で許容される
                                       各ジョイントの最大角度変化量 (ラジアン)。
                                       None の場合はチェックしない。
            num_axes_to_check_per_step: ウェイポイント間の角度変化チェック対象とする軸の数。Noneの場合は全軸。
            max_ik_retries_on_jump: max_joint_change_per_step を超えた場合のIKリトライ回数上限
            ik_retry_perturbation: IKリトライ時にシード値に加える摂動の最大量 (ラジアン)
            send_immediately: すぐに軌道を送信するかどうか
            wait: 軌道実行の完了を待つかどうか
        """
        if not isinstance(waypoints, (list, np.ndarray)):
            self.get_logger().error(
                f"Waypoints must be a list or numpy.ndarray, but got {type(waypoints)}. Trajectory not sent."
            )
            return
        if q_init is None:
            q_prev = self.get_current_joint_positions()
        else :
            q_prev = np.array(q_init)

        joint_trajectory = []
        all_waypoints_solved = True  # Flag to track if all waypoints were solved
        for i, waypoint in enumerate(
            tqdm(waypoints[1:], desc="Calculating IK for waypoints"), start=1
        ):
            solved_within_limit = False
            best_solution_found = None  # リトライで見つかった最良の解を保持

            # --- IK計算とリトライループ ---
            for attempt in range(max_ik_retries_on_jump + 1):  # 初回 + リトライ回数
                # IK計算に使用するシード値 (初回は q_prev, リトライ時は摂動を加える)
                if attempt == 0:
                    q_seed = q_prev
                else:
                    # q_prev にランダムな摂動を加える
                    perturbation = np.random.uniform(
                        -ik_retry_perturbation, ik_retry_perturbation, size=q_prev.shape
                    )
                    q_seed_np = q_prev + perturbation
                    # オプション: ジョイントリミット内にクリップする処理を追加しても良い
                    q_seed = q_seed_np

                # IK計算実行
                current_joint_positions_np = self.solve_ik(waypoint, q_init=q_seed)

                if current_joint_positions_np is not None:
                    # IK成功
                    if current_joint_positions_np.size == len(self.valid_joint_names):
                        # サイズチェックOK
                        best_solution_found = (
                            current_joint_positions_np  # 少なくとも解は見つかった
                        )

                        # 新しいヘルパーメソッドで角度変化量チェック
                        if self._is_joint_change_within_limit(
                            current_joint_positions=current_joint_positions_np,
                            previous_joint_positions=q_prev,
                            check_mode=JointChangeCheckMode.SUM_TOTAL_WITHIN_LIMIT,  # デフォルトは個別チェック
                            max_joint_change=max_joint_change_per_step,
                            waypoint_index=i,
                            attempt_number=attempt,
                        ):
                            solved_within_limit = True
                            break  # リトライループを抜ける (最適な解が見つかった)
                        else:
                            continue  # 閾値超え -> 次のリトライへ
                    else:
                        # IK成功したがサイズが不正 (solve_ik内で警告が出るはず)
                        self.get_logger().error(
                            f"IK solution for waypoint [{i}] (Attempt {attempt}) has unexpected size "
                            f"({current_joint_positions_np.size})."
                        )
                        # この場合はリトライしても無駄なのでループを抜ける
                        best_solution_found = None  # 失敗扱い
                        break
                else:
                    # IK失敗 (solve_ik内で警告が出るはず)
                    self.get_logger().warn(
                        f"IK failed for waypoint [{i}] (Attempt {attempt})."
                    )
                    # リトライを続ける (別のシードで成功するかもしれない)
                    # continue は不要 (ループの次の反復へ)

            # --- リトライループの結果処理 ---
            if solved_within_limit and best_solution_found is not None:
                # 閾値内でIK成功
                joint_trajectory.append(best_solution_found.tolist())
                q_prev = best_solution_found  # 次のステップの q_prev を更新
            else:
                # リトライしても閾値内の解が見つからない or IK自体が失敗
                if best_solution_found is not None:  # IKは解けたが閾値を超えた場合
                    self.get_logger().error(
                        f"Failed to find IK solution within joint change limit ({max_joint_change_per_step:.3f} rad) "
                        f"for waypoint [{i}] after {max_ik_retries_on_jump} retries. "
                        f"Last valid solution had max change: {np.max(np.abs(best_solution_found - q_prev)):.3f} rad. "
                        "Stopping trajectory generation."
                    )
                else:  # IKが一度も成功しなかった場合
                    self.get_logger().error(
                        f"IK failed for waypoint [{i}] after {max_ik_retries_on_jump + 1} attempts. "
                        "Stopping trajectory generation."
                    )
                all_waypoints_solved = False
                break  # 外側の for ループ (ウェイポイントのループ) を抜ける

        # --- 軌道送信処理 (変更なし) ---
        if all_waypoints_solved and joint_trajectory:
            self.get_logger().info(
                f"Successfully calculated joint trajectory for {len(joint_trajectory)} waypoints."
            )
            self.set_joint_trajectory(
                joint_trajectory=joint_trajectory,
                time_to_reach=time_to_reach,
                velocities=None,
                accelerations=None,
                send_immediately=send_immediately,
                wait=wait
            )
        elif not joint_trajectory:
            self.get_logger().error(
                "Trajectory generation failed: No valid joint positions calculated."
            )
        else:
            self.get_logger().error(
                "Trajectory generation incomplete due to IK failure(s) or joint change limit exceeded. Trajectory not sent."
            )

    def set_goal_pose(
        self,
        goal_pose: List[float],
        time_to_reach: Optional[float] = None,
        max_joint_change_limit: Optional[float] = np.pi / 2,
        target_ee_link: Optional[str] = None, # ターゲットEEリンクを指定する引数を追加
        num_axes_to_check_for_goal: Optional[int] = None, # ゴールへのチェック軸数を追加
        max_ik_retries: int = 100,
        send_immediately: bool = False,
        wait: bool = True,
    ) -> Optional[np.ndarray]:
        """
        単一のゴールポーズ (デカルト座標系 [x,y,z,qx,qy,qz,qw]) に対してIKを解き、
        関節角度変化量チェック (ALL_INDIVIDUAL_WITHIN_LIMIT) を行った上で軌道を生成し送信する。
        このメソッドは set_waypoints を呼び出さずに直接処理を行う。

        Args:
            goal_pose: 単一の目標姿勢 [x,y,z,qx,qy,qz,qw]。
            time_to_reach: 目標到達時間 (秒)。Noneの場合、デカルト距離と速度から自動計算。
            max_joint_change_limit: 現在の関節角度から目標姿勢への移動時に許容される
                                               各関節の最大角度変化量 (ラジアン)。
                                               None の場合はチェックしない。Defaults to np.pi/4.
            target_ee_link: このゴールポーズ計算に使用するエンドエフェクタリンク。Noneの場合、現在のEEリンクを使用。
            num_axes_to_check_for_goal: 角度変化チェック対象とする軸の数。Noneの場合は全軸。
            max_attempts: IKソルバーが解を見つけるための最大試行回数。Defaults to 10.
            send_immediately: すぐに軌道を送信するかどうか。
            wait: 軌道実行の完了を待つかどうか。
        
        Returns:
            IKが成功し軌道が送信された場合はその関節角度 (np.ndarray)、そうでなければ None。
        """
        # Allow both list and numpy.ndarray for goal_pose
        if not isinstance(goal_pose, (list, np.ndarray)):
            self.get_logger().error(
                f"Goal pose must be a list or numpy.ndarray, but got {type(goal_pose)}. Trajectory not sent."
            )
            return None
        # Ensure goal_pose is a numpy array for easier manipulation
        expected_cartesian_pose_len = 7
        if len(goal_pose) != expected_cartesian_pose_len:
            self.get_logger().error(
                f"Cartesian goal pose has an unexpected number of elements: {len(goal_pose)}. "
                f"Expected {expected_cartesian_pose_len} (for [x,y,z,qx,qy,qz,qw]). Trajectory not sent."
            )
            return None

        # 指定されたEEリンクと現在のEEリンクが異なる場合は切り替える
        pre_tool_link=None
        if target_ee_link is not None and target_ee_link != self.tool_link:
            self.get_logger().info(f"Switching EE link for set_goal_pose from '{self.tool_link}' to '{target_ee_link}'.")
            pre_tool_link = self.tool_link
            self.change_ee_link(target_ee_link)


        # Solve IK for the goal_pose
        q_prev = self.get_current_joint_positions()
        if q_prev is None:
            self.get_logger().error("Current joint positions not available. Cannot calculate goal pose trajectory.")
            return None

        q_best = self.solve_ik(goal_pose, q_init=q_prev, number_of_attempts=max_ik_retries)

        if q_best is None:
            self.get_logger().error(
                f"Failed to find IK solution for the goal pose after {max_ik_retries} attempts (within solve_ik). "
                "Stopping trajectory generation."
            )
            return None
        else:
            self.get_logger().info(
                f"Found IK solution for the goal pose."
            )

            is_within_limit = self._is_joint_change_within_limit(
                current_joint_positions=q_best,
                previous_joint_positions=q_prev,
                check_mode=JointChangeCheckMode.ALL_INDIVIDUAL_WITHIN_LIMIT,
                max_joint_change=max_joint_change_limit,
                waypoint_index=0,  # Representing the single goal for logging
                attempt_number=0,  # No retry loop for angle check in this direct implementation
                num_axes_to_check=num_axes_to_check_for_goal, # 追加
            )

            if is_within_limit:
                calculated_time_to_reach = time_to_reach
                if calculated_time_to_reach is None:
                    if pre_tool_link is not None:
                        self.change_ee_link(pre_tool_link)
                        current_ee_pose_list = self.solve_fk(q_prev)
                        self.change_ee_link(self.tool_link)
                    else:
                        current_ee_pose_list = self.solve_fk(q_prev)
                    if current_ee_pose_list is None or len(current_ee_pose_list) < 3:
                        self.get_logger().warn("Could not get current EE pose via FK. Using min_time_to_reach.")
                        calculated_time = self.min_time_to_reach_for_pose
                    else:
                        current_pos = np.array(current_ee_pose_list[0:3]) # x, y, z
                        goal_pos_np = np.array(goal_pose[:3])
                        cartesian_distance = np.linalg.norm(goal_pos_np - current_pos)
                        self.get_logger().info(f"current_pos: {current_pos}")
                        self.get_logger().info(f"goal_pos: {goal_pos_np}")
                        if self.default_cartesian_speed_for_pose > 1e-6: # Avoid division by zero
                            calculated_time = cartesian_distance / self.default_cartesian_speed_for_pose
                        else:
                            calculated_time = self.min_time_to_reach_for_pose # Default to min time if speed is zero
                        self.get_logger().info(
                            f"Cartesian distance to goal: {cartesian_distance:.3f}m. "
                            f"Calculated time based on speed: {calculated_time:.2f}s."
                        )
                    calculated_time_to_reach = max(calculated_time, self.min_time_to_reach_for_pose)
                    self.get_logger().info(f"Using calculated time_to_reach: {calculated_time_to_reach:.2f}s")

                self.get_logger().info(
                    "Joint change is within limit. Sending trajectory to goal pose."
                )
                self.set_joint_trajectory(
                    joint_trajectory=[q_best.tolist()],
                    time_to_reach=calculated_time_to_reach,
                    velocities=None,
                    accelerations=None,
                    send_immediately=send_immediately,
                    wait=wait,
                )
                return q_best
            else:
                return None

    def set_joint_trajectory(
        self,
        joint_trajectory: List[List[float]],
        time_to_reach: float,
        velocities: Optional[List[float]] = None,
        accelerations: Optional[List[float]] = None,
        strict_velocity_control: bool = True,
        send_immediately: bool = False,
        wait: bool = True,
    ) -> None:
        """
        生成済みの joint trajectory をアクションクライアントに送信する
        
        補間方法:
        - position only (velocities=None, accelerations=None): 1次補間（線形補間）
        - position + velocity (accelerations=None): 3次補間（3次多項式補間, 位置と速度が滑らか）
        - position + velocity + acceleration: 5次補間（5次多項式補間, 位置と速度と動き出しの加速度が滑らか）
        
        Args:
            joint_trajectory: 関節軌道のリスト
            time_to_reach: 軌道実行時間
            velocities: 速度値（Noneの場合は設定しない、strict_velocity_controlがTrueの場合は無視される）
            accelerations: 加速度値（Noneの場合は設定しない）
            strict_velocity_control: 手先等速度制御を使用するかどうか（Trueの場合velocitiesパラメータは無視される）
            send_immediately: 即座に送信するかどうか
            wait: 完了を待つかどうか
        """
        if not joint_trajectory or len(joint_trajectory[0]) != len(
            self.valid_joint_names
        ):
            self.get_logger().error(
                "Joint trajectory has an unexpected number of joint positions."
            )
            return

        if len(joint_trajectory) == 0:
            self.get_logger().warn("Empty joint_trajectory provided to set_joint_trajectory. Nothing to send.")
            return

        # strict_velocity_controlが有効な場合、手先等速度制御のための関節速度を計算
        if strict_velocity_control:
            self.get_logger().info("Using strict velocity control for end-effector constant velocity.")
            # Forward Kinematicsで各関節位置からワールド座標系の手先姿勢を計算
            waypoints = []
            for joints in joint_trajectory:
                try:
                    pose = self.solve_fk(joints)
                    waypoints.append(pose)
                except Exception as e:
                    self.get_logger().warn(f"Error calculating FK: {e}")
                    waypoints.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])  # デフォルト姿勢

            # 手先等速運動のための関節速度を計算（velocitiesパラメータを上書き）
            velocities = self.generate_constant_velocity_vector(
                waypoints,
                joint_trajectory,
                time_to_reach,
            )
            
            # プロット機能を呼び出し（velocitiesが正常に計算された場合のみ）
            if velocities is not None:
                self._plot_joint_velocities_and_positions(velocities, joint_trajectory, time_to_reach)

        dt = time_to_reach / len(joint_trajectory)
        self.goals.clear()
        for i, goal_joints in enumerate(joint_trajectory, start=1):
            point = JointTrajectoryPoint()
            point.positions = goal_joints
            if velocities is not None:
                if len(velocities) == len(joint_trajectory):
                    # velocitiesが軌道点数と同じ場合：各軌道点に対して一つの速度セット
                    point.velocities = velocities[i - 1] if i - 1 < len(velocities) else velocities[-1]
                elif len(velocities) == len(goal_joints):
                    # velocitiesがジョイント数と同じ場合：全軌道点で同じ速度セット
                    point.velocities = list(velocities)
                else:
                    self.get_logger().warn(f"Velocities length ({len(velocities)}) doesn't match trajectory points ({len(joint_trajectory)}) or joints ({len(goal_joints)}). Using zero velocities.")
                    point.velocities = [0.0] * len(goal_joints)
            # velocitiesがNoneの場合は速度を設定しない（空のリストのままにする）
            if accelerations is not None:
                if len(accelerations) == len(joint_trajectory):
                    # accelerationsが軌道点数と同じ場合：各軌道点に対して一つの加速度セット
                    point.accelerations = accelerations[i - 1] if i - 1 < len(accelerations) else accelerations[-1]
                elif len(accelerations) == len(goal_joints):
                    # accelerationsがジョイント数と同じ場合：全軌道点で同じ加速度セット
                    point.accelerations = list(accelerations)
                else:
                    self.get_logger().warn(f"Accelerations length ({len(accelerations)}) doesn't match trajectory points ({len(joint_trajectory)}) or joints ({len(goal_joints)}). Using zero accelerations.")
                    point.accelerations = [0.0] * len(goal_joints)
            # accelerationsがNoneの場合は加速度を設定しない（空のリストのままにする）
            sec = int(dt * i)
            nsec = int((dt * i - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            self.goals.append(point)
        self.get_logger().debug(f"All joint trajectory: {self.goals}")
        self.get_logger().debug(f"Joint trajectory [0]: {joint_trajectory[0]}")
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
        send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
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

    def generate_constant_velocity_vector(
        self,
        waypoints: List[List[float]],
        joint_trajectory: List[List[float]],
        time_to_reach: float,
    ) -> Optional[List[List[float]]]:
        """
        時間ベースでリサンプリングされた軌道に対し、手先等速運動を実現する関節速度を計算します。

        軌道全体の開始・終了姿勢と所要時間から、一定の目標手先速度（並進・角）を算出します。
        その後、軌道上の各点において、その時点でのヤコビアンと、算出した一定の目標手先速度を
        用いて関節速度を計算します。これにより、数値微分によるノイズの増幅を防ぎます。

        Args:
            waypoints (list):
                リサンプリングされた手先姿勢のリスト。各要素は [x, y, z, qx, qy, qz, qw]。
                クォータニオンは [x, y, z, w] の順です。
            joint_trajectory (list):
                リサンプリングされた関節角度(rad)のリスト。waypointsと長さが一致している必要があります。
            time_to_reach (float):
                全ウェイポイントを通過するための合計目標時間（秒）。

        Returns:
            list or None:
                各時刻における関節速度ベクトル(List[float])のリスト。
        """
        if not waypoints or not joint_trajectory:
            self.get_logger().warn("ウェイポイントまたは関節軌道が空です。")
            return None
        
        if time_to_reach <= 0:
            self.get_logger().warn("time_to_reachは正の値である必要があります。")
            return None

        # KDL helperが利用可能かチェック
        if self.kdl_helper is None:
            self.get_logger().warn("KDL helper not available. Cannot calculate joint velocities.")
            return None

        # --- 1. 軌道全体で一定となる目標手先速度を計算 ---
        # 軌道の開始と終了の姿勢を取得
        start_pos = np.array(waypoints[0][0:3])
        start_quat = np.array(waypoints[0][3:])
        end_pos = np.array(waypoints[-1][0:3])
        end_quat = np.array(waypoints[-1][3:])

        # 目標並進速度ベクトル v = (P_end - P_start) / T_total
        v_target = (end_pos - start_pos) / time_to_reach

        # 目標角速度ベクトル ω = (AngleAxis_total) / T_total
        rotations = R.from_quat([start_quat, end_quat])
        relative_rotation = rotations[1] * rotations[0].inv()
        angle_axis = relative_rotation.as_rotvec()
        omega_target = angle_axis / time_to_reach

        # 6次元の目標手先速度ベクトル（この値はループ内で不変）
        cartesian_target_velocity = np.concatenate([v_target, omega_target])
        self.get_logger().info(f"Constant Target Cartesian Velocity: v={v_target.round(3)}, ω={omega_target.round(3)}")

        joint_velocities_list = []
        condition_numbers = []

        # --- 2. 各時刻の関節角度から関節速度を計算 ---
        for i, joint_angles in enumerate(joint_trajectory):
            try:
                # 現在の関節角度におけるヤコビアンを取得
                jacobian = self.kdl_helper.jacobian(joint_angles)
                cond_num = np.linalg.cond(jacobian)
                condition_numbers.append(cond_num)

                # 擬似逆行列を計算
                jacobian_inv = np.linalg.pinv(jacobian)
                
                #【重要】ループの外で計算した一定の目標手先速度を常に使用する
                joint_velocities = jacobian_inv @ cartesian_target_velocity
                joint_velocities = np.array(joint_velocities).flatten()

            except np.linalg.LinAlgError:
                self.get_logger().warn(f"警告: インデックス{i}のヤコビアン計算でエラー。速度をゼロにします。")
                joint_velocities = np.zeros(len(joint_angles))
            
            joint_velocities_list.append(joint_velocities.tolist())

        # --- 3. 統計情報の出力 ---
        if joint_velocities_list:
            all_velocities = np.array(joint_velocities_list)
            max_velocity = np.max(np.abs(all_velocities))
            self.get_logger().info(f"Calculated Joint Velocities - Max(abs): {max_velocity:.4f} rad/s")

        if condition_numbers:
            cond_array = np.array(condition_numbers)
            self.get_logger().info(f"Jacobian Condition Number Stats - "
                                  f"Max: {np.max(cond_array):.2f}, "
                                  f"Min: {np.min(cond_array):.2f}, "
                                  f"Avg: {np.mean(cond_array):.2f}")
            if np.max(cond_array) > 100:
                self.get_logger().warn("High condition number detected, indicating proximity to a singularity.")
            
        return joint_velocities_list

    def _plot_joint_velocities_and_positions(self, velocity_list: List[List[float]], position_list: List[List[float]], time_to_reach: float) -> None:
        """
        関節速度と位置をプロットする。
        
        Args:
            velocity_list (list): 各ポイントの関節速度のリスト
            position_list (list): 各ポイントの関節位置のリスト
            time_to_reach (float): 総実行時間
        """
        if not velocity_list or not position_list:
            self.get_logger().warn("No velocities or positions to plot.")
            return
            
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            
            # 時間軸を作成
            time_points_vel = np.linspace(0, time_to_reach, len(velocity_list))
            time_points_pos = np.linspace(0, time_to_reach, len(position_list))
            
            # 関節数を取得
            num_joints = len(velocity_list[0])
            
            # データを配列に変換
            velocities = np.array(velocity_list)
            positions = np.array(position_list)
            
            # プロット作成
            plt.figure(figsize=(15, 12))
            
            # 速度プロット
            for joint_idx in range(num_joints):
                plt.subplot(2, num_joints, joint_idx + 1)
                plt.plot(time_points_vel, velocities[:, joint_idx], 'b-', alpha=0.7, label='Line')
                plt.scatter(time_points_vel, velocities[:, joint_idx], c='red', s=8, alpha=0.6, label='Points')
                plt.title(f'Joint {joint_idx + 1} Velocity')
                plt.xlabel('Time (s)')
                plt.ylabel('Velocity (rad/s)')
                plt.grid(True)
                if joint_idx == 0:
                    plt.legend()
            
            # 位置プロット
            for joint_idx in range(num_joints):
                plt.subplot(2, num_joints, num_joints + joint_idx + 1)
                plt.plot(time_points_pos, positions[:, joint_idx], 'g-', alpha=0.7, label='Line')
                plt.scatter(time_points_pos, positions[:, joint_idx], c='orange', s=8, alpha=0.6, label='Points')
                plt.title(f'Joint {joint_idx + 1} Position')
                plt.xlabel('Time (s)')
                plt.ylabel('Position (rad)')
                plt.grid(True)
                if joint_idx == 0:
                    plt.legend()
                
            plt.tight_layout()
            plt.savefig('/tmp/joint_velocities_and_positions_plot.png', dpi=150, bbox_inches='tight')
            plt.show()
            self.get_logger().info("Joint velocity and position plot saved to /tmp/joint_velocities_and_positions_plot.png")
            
        except ImportError:
            self.get_logger().warn("matplotlib not available. Skipping velocity and position plot.")
        except Exception as e:
            self.get_logger().error(f"Error plotting joint velocities and positions: {e}")


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
            "wrist_3_joint",
        ],
        tool_link="pestle_tip",
        base_link="base_link",
        robot_urdf_file_path=get_package_share_directory("grinding_robot_description") + "/urdf/ur/ur5e_with_pestle.urdf",
        ik_solver=IKType.TRACK_IK,
    )

    while rclpy.ok():
        print("\nMenu:")
        print("0. Exit")
        print("1. Solve IK")
        print("2. Test JTC with one target pose")
        print("3. Test JTC with waypoints")
        print("10. Get joint names")
        print("11. Get current joint positions")
        choice = input("Enter your choice: ")

        if choice == "0":
            break
        elif choice == "1":
            target_pose = [-0.1, 0.4, 0.3, 1.0, 0.0, 0.0, 0.0]
            joint_positions = arm_controller.solve_ik(target_pose)
            print(f"IK solution: {joint_positions}")
        elif choice == "2":
            target_pose = [-0.1, 0.4, 0.3, 1.0, 0.0, 0.0, 0.0]
            arm_controller.set_goal_pose(
                target_pose, 
                time_to_reach=5, 
                send_immediately=True,
                # target_ee_link="another_tool_tip" # 必要に応じてEEリンクを指定
            )
        elif choice == "3":
            waypoints = [
                [-0.1, 0.4, 0.3, 1.0, 0.0, 0.0, 0.0],
                [-0.1, 0.4, 0.4, 1.0, 0.0, 0.0, 0.0],
                [-0.1, 0.3, 0.5, 1.0, 0.0, 0.0, 0.0],
            ]
            arm_controller.set_waypoints(
                waypoints,
                time_to_reach=5,
                send_immediately=True,
                max_joint_change_per_step=np.pi / 4,
            )

        elif choice == "10":
            print(f"Joint names: {arm_controller.get_joint_names()}")
        elif choice == "11":
            current_joint_positions = arm_controller.get_current_joint_positions()
            print(f"Current joint positions: {current_joint_positions}")
        else:
            print("Invalid choice. Please try again.")

        rclpy.spin_once(arm_controller)

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
