#!/usr/bin/env python3



import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from grinding_motion_routines.arm_position_controller import ArmPositionController  # Modified for ROS2
from sensor_msgs.msg import JointState  # Import JointState
import time
import numpy as np
from tqdm import tqdm

class JointTrajectoryControllerHelper(ArmPositionController, Node):  # Inherit from rclpy.node.Node
    """Motion Executor including IK and JointTrajectoryController(JTC) ."""

    def __init__(
        self,
        robot_urdf_pkg,
        robot_urdf_file_name,
        joint_trajectory_controller_name,
        tcp_link,
        ns=None,
        joint_names_prefix=None,
        ft_topic=None,
        gripper=False,
        ik_solver="trac_ik",
        solve_type="Distance",
    ):
        if joint_names_prefix is None:
            joint_names_prefix = ""
        
        # ROS2 Node Initialization
        rclpy.init() # Initialize rclpy if not already initialized
        Node.__init__(self, "joint_trajectory_controller_executor")  # Initialize the Node

        super().__init__(
            robot_urdf_pkg,
            robot_urdf_file_name,
            joint_trajectory_controller_name,
            gripper=gripper,
            namespace=ns,
            joint_names_prefix=joint_names_prefix,
            ee_link=tcp_link,
            ft_topic=ft_topic,
            ik_solver=ik_solver,
            solve_type=solve_type,
        )
        self.joint_names_prefix = joint_names_prefix
        self.init_end_effector_link = tcp_link
        self.solve_type = solve_type
        
        # ROS2 Action Client for FollowJointTrajectory
        self._action_client = ActionClient(self, FollowJointTrajectory, joint_trajectory_controller_name + '/follow_joint_trajectory') # Correct topic name
        
        # ROS2:  Subscribe to joint states for current joint positions
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self._joint_state_callback, 10)
        self.current_joint_positions = []


    def _joint_state_callback(self, msg):
        """
        Callback function to update current joint positions.
        """
        self.current_joint_positions = list(msg.position)

    def joint_angles(self):
        """
        Get the current joint angles, handling potential threading issues and delays in ROS2.
        """
        # In ROS2, the callback might not have populated the data yet.  Wait (with timeout).
        timeout_seconds = 1.0  # Wait at most 1 second.
        start_time = time.time()
        while not self.current_joint_positions and time.time() - start_time < timeout_seconds:
            rclpy.spin_once(self, timeout_sec=0.1) # Process callbacks
            time.sleep(0.01)

        if not self.current_joint_positions:
            self.get_logger().error("Failed to get joint positions from /joint_states topic!")
            return []  # Or raise an exception if appropriate

        return self.current_joint_positions


    def execute_to_goal_pose(
        self,
        goal_pose,
        ee_link="",
        time_to_reach=5.0,
        max_attempts=10,
        wait=True,
    ):
        """Supported pose is only x y z aw ax ay az"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        best_ik= None
        best_dif= float("inf")
        start_joint = self.joint_angles()
        for i in range(max_attempts):
            joint_goal = self._solve_ik(goal_pose)
            if joint_goal is None or np.any(joint_goal == "ik_not_found"):
                self.get_logger().error("IK not found, Please check the pose")
                continue
            dif=abs(np.sum(np.array(start_joint[0:-1])-np.array(joint_goal[0:-1])))
            if dif < best_dif:
                best_ik=joint_goal
                best_dif=dif
        self.set_joint_positions(best_ik, t=time_to_reach, wait=wait)
        return best_ik


    def generate_joint_trajectory(
        self,
        waypoints,
        ee_link="",
        joint_difference_limit=0.03,
        max_attempts=1000,
        max_attempts_for_first_waypoint=100,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        joint_trajectory = []
        start_joint = self.joint_angles()
        total_waypoints = len(waypoints)  # 全体のwaypoints数を取得

        for i, pose in tqdm(
            enumerate(waypoints),
            total=total_waypoints,
            desc="Planning motion for waypoints",
        ):
            if i == 0:
                best_joint_difference = float("inf")
                for i in tqdm(
                    range(max_attempts_for_first_waypoint),
                    desc="Finding best IK solution for 1st waypoint",
                ):
                    ik_joint = self._solve_ik(pose, q_guess=start_joint)
                    if ik_joint is None or np.any(ik_joint == "ik_not_found"):
                        self.get_logger().error("IK not found, Please check the pose")
                        continue
                    joint_difference = np.sum(np.abs(np.array(start_joint[0:-1]) - np.array(ik_joint[0:-1])))
                    if joint_difference < best_joint_difference:
                        best_ik_joint = ik_joint
                        best_joint_difference = joint_difference
                start_joint = best_ik_joint
                joint_trajectory.append(list(best_ik_joint))
            else:
                # 2番目以降のwaypoint
                retry_count = 0  # 各ポーズごとの再試行カウントをリセット
                joint_difference_list = []
                while retry_count < max_attempts:
                    ik_joint = self._solve_ik(pose, q_guess=start_joint)
                    if ik_joint is None or np.any(ik_joint == "ik_not_found"):
                        self.get_logger().error("IK not found, Please check the pose")
                        return None
                    joint_difference = np.sum(np.abs(np.array(start_joint[0:-1]) - np.array(ik_joint[0:-1])))
                    if joint_difference > joint_difference_limit:
                        retry_count += 1  # 再試行カウントを増やす
                        joint_difference_list.append(joint_difference)
                        if retry_count >= max_attempts:
                            self.get_logger().error(
                                f"Waypoint {i + 1}/{total_waypoints} failed after {max_attempts} trials, "
                                f"Joint difference was too large (min diff:{min(joint_difference_list)})"
                            )
                            return None
                        continue
                    else:
                        start_joint = ik_joint
                        joint_trajectory.append(list(ik_joint))
                        break

        return joint_trajectory if joint_trajectory else None

    def execute_by_joint_trajectory(self, joint_trajectory, time_to_reach=5.0):
        self.set_joint_trajectory(joint_trajectory, t=time_to_reach)


    def execute_by_waypoints(
        self,
        waypoints,
        joint_difference_limit,
        ee_link="",
        time_to_reach=5.0,
        max_attempts=1000,
        max_attempts_for_first_waypoint=100
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""

        joint_trajectory = self.generate_joint_trajectory(
            waypoints,
            ee_link=ee_link,
            joint_difference_limit=joint_difference_limit,
            max_attempts=max_attempts,
            max_attempts_for_first_waypoint=max_attempts_for_first_waypoint
        )
        self.set_joint_trajectory(joint_trajectory, t=time_to_reach)


    def execute_to_joint_goal(self, joint_goal, time_to_reach=5.0, wait=True):
        self.set_joint_positions(joint_goal, t=time_to_reach, wait=wait)


    def _change_ee_link(self, new_ee_link):
        old_ee_link = self.ee_link
        self.get_logger().info(
            "============ Cange End effector link: %s to %s"
            % (old_ee_link, new_ee_link)
        )
        self.ee_link = (
            new_ee_link
            if self.joint_names_prefix is None
            else self.joint_names_prefix + new_ee_link
        )
        self._init_ik_solver(self.base_link, self.ee_link, self.solve_type)
