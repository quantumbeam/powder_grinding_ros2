#!/home/ubuntu/user/grinding_ws/venv/bin/python3
import sys
sys.path.append('/home/ubuntu/user/grinding_ws/venv/lib/python3.12/site-packages')

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

    def _init_ik_solver(self, base_link: str, ee_link: str, urdf_path: str) -> None:
        """
        Trac-IK ソルバーの初期化
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
            raise Exception(f"Unsupported IK solver: {self.ik_solver.name}")

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
                    if joint_positions is not None:
                        return joint_positions
                    else:
                        self.get_logger().warn("No IK solution found.")
                        return None
                except Exception as e:
                    self.get_logger().error(f"Could not solve IK: {e}")
                    return None
            else:
                raise Exception(f"Unsupported IK solver: {self.ik_solver.name}")
    
    def set_waypoints(self, waypoints: List[List[float]],
                      time_to_reach: int,
                      send_immediately: bool = False,
                      wait: bool = True) -> None:
        """
        複数のウェイポイントに対して軌道を生成し送信する
        """
        joint_trajectory = []
        q_init = self.get_current_joint_positions()
        for i, waypoint in enumerate(tqdm(waypoints, desc="Calculating IK for waypoints"), start=1):
            joint_positions = self.solve_ik(waypoint, q_init=q_init)
            q_init = joint_positions
            if joint_positions is not None:
                joint_trajectory.append(joint_positions)
            else:
                self.get_logger().warn(f"No joint positions found for waypoint [{i}]: {waypoint}")
                return  # 中断
        self.set_joint_trajectory(joint_trajectory, time_to_reach, send_immediately, wait)

    def set_goal_pose(self, goal_pose: List[float],
                      time_to_reach: int,
                      send_immediately: bool = False,
                      wait: bool = True) -> None:
        """
        単一のゴールポーズに対して軌道を生成し送信する
        """
        joint_positions = self.solve_ik(goal_pose)
        if joint_positions is not None:
            self.set_joint_trajectory([joint_positions], time_to_reach, send_immediately, wait)
        else:
            self.get_logger().warn("No joint positions found for goal pose.")

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
            if joint_positions:
                print(f"IK solution: {joint_positions}")
            else:
                print("No IK solution found.")
        elif choice == '2':
            target_pose = [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
            arm_controller.set_goal_pose(target_pose, time_to_reach=5, send_immediately=True)
        elif choice == '3':
            waypoints = [
                [0.5, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.6, 1.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.7, 1.0, 0.0, 0.0, 0.0]
            ]
            arm_controller.set_waypoints(waypoints, time_to_reach=5, send_immediately=True)
        elif choice == '4':
            print("Testing grinding motion ...")
            try:
                from grinding_motion_routines.grinding_motion_generator import MotionGenerator
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
            grinding_pos_end = [-8, 0.0001]
            grinding_rz_beginning = 36
            grinding_rz_end = 36
            number_of_rotations = 10
            angle_scale = 0.3
            yaw_bias = None
            yaw_twist_per_rotation = 0
            number_of_waypoints_per_circle = 50
            center_position = [0, 0]
            sec_per_rotation = 0.5

            motion_generator = MotionGenerator(mortar_top_position, mortar_inner_size)
            try:
                waypoints = motion_generator.create_circular_waypoints(
                    begining_position=grinding_pos_beginning,
                    end_position=grinding_pos_end,
                    begining_radious_z=grinding_rz_beginning,
                    end_radious_z=grinding_rz_end,
                    number_of_rotations=number_of_rotations,
                    angle_scale=angle_scale,
                    yaw_bias=yaw_bias,
                    yaw_twist_per_rotation=yaw_twist_per_rotation,
                    number_of_waypoints_per_circle=number_of_waypoints_per_circle,
                    center_position=center_position
                )
            except ValueError as e:
                print(f"Error generating circular waypoints: {e}")
                continue
            print("Generated grinding motion waypoints")
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