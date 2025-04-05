import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from trac_ik_python.trac_ik import IK as TRACK_IK_SOLVER
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from enum import Enum, auto
from rclpy.parameter import Parameter
import time

class IKType(Enum):
    TRACK_IK = auto()
    PICK_IK = auto()
    QU_IK = auto()  

class ArmPositionController(Node):
    """
    Arm controller class with position control
    """

    def __init__(self,  
                 controller_name="joint_trajectory_controller",
                 joints_name=[""],
                 tool_link="tool0",
                 base_link="base_link",
                 ik_solver=IKType.TRACK_IK):
        """
        Initialize the ArmPositionController node.

        Args:
            controller_name (str): The name of the controller to publish trajectories to.
            joints_name (list): The list of joint names.
            tool_link (str): The name of the tool link.
            base_link (str): The name of the base link.
            ik_solver (IKType): The type of IK solver to use.
        """
        super().__init__('arm_position_controller')

        self.controller_name = controller_name
        self.joints_name = joints_name

        if not self.joints_name:
            raise Exception('"joints" parameter is not set!')

        action_topic = f"/{self.controller_name}/follow_joint_trajectory"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.goals = []
        self.i = 0
        
        if ik_solver == IKType.TRACK_IK:
            self.get_logger().info("TRACK IK selected")
        elif ik_solver == IKType.PICK_IK:
            self.get_logger().info("Pick IK selected")
        elif ik_solver == IKType.QU_IK:  # または FULL_IK
            self.get_logger().info("QuIK selected")
        else:
            raise Exception("unsupported ik_solver", ik_solver)
        self.ik_solver = ik_solver 

        self._init_ik_solver(base_link, tool_link, solve_type="Speed")
        self._robot_urdf_package = "ur_description" #add
        self._robot_urdf_file_name = "ur5e" #add

    def _load_urdf_string(self, package, filename):
        """
        Loads a URDF file as a string from a given ROS 2 package.

        Args:
            package (str): The name of the ROS 2 package.
            filename (str): The name of the URDF file (without the .urdf extension).

        Returns:
            str: The URDF file content as a string, or None if an error occurred.
        """
        try:
            package_dir = get_package_share_directory(package)
        except PackageNotFoundError:
            self.get_logger().error(f"Package '{package}' not found.")
            return None
        
        urdf_file = f"{package_dir}/urdf/{filename}.urdf"

        try:
            with open(urdf_file, 'r') as f:
                urdf = f.read()
            return urdf
        except FileNotFoundError:
            self.get_logger().error(f"URDF file '{urdf_file}' not found.")
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading URDF file: {e}")
            return None

    def _init_ik_solver(self, base_link, ee_link, solve_type):
        """
        Initializes the IK solver (Trac-IK).

        Args:
            base_link (str):  Name of the base link.
            ee_link (str):    Name of the end-effector link.
            solve_type (str): Type of solver to use ("Speed", "Distance", "Manipulation1", "Manipulation2")
        """
        self.base_link = base_link
        self.ee_link = ee_link

        if self.ik_solver == IKType.TRACK_IK:
            try:
                robot_description_param = self.get_parameter("robot_description")

                if robot_description_param.type_ == Parameter.Type.NOT_SET:
                    urdf_string = self._load_urdf_string(
                        self._robot_urdf_package, self._robot_urdf_file_name
                    )
                    if urdf_string is None:
                        self.get_logger().error("Failed to load URDF string.")
                        return

                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link=base_link,
                        tip_link=ee_link,
                        solve_type=solve_type,
                        timeout=0.002,
                        epsilon=1e-5,
                        urdf_string=urdf_string,
                    )
                else:
                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link=base_link,
                        tip_link=ee_link,
                        solve_type=solve_type,
                        timeout=0.002,
                        epsilon=1e-5,
                        urdf_string=robot_description_param.value
                    )
            except Exception as e:
                self.get_logger().error(f"Could not instantiate TRAC_IK: {e}")
        else:
            raise Exception("unsupported ik_solver: ", self.ik_solver.name)
    
    def solve_ik(self, pose, q_init=None):
        """
        Solve the inverse kinematics for a given pose.

        Args:
            pose (list): The target pose as [x, y, z, qx, qy, qz, qw].
            q_init (list): The initial joint positions.

        Returns:
            list: The joint positions that achieve the target pose.
        """
        if len(pose) != 7:
            self.get_logger().error("Pose must have 7 elements (x, y, z, qx, qy, qz, qw).")
            return None
        if q_init is None:
            q_init = [0.0] * self.trac_ik.number_of_joints

        solution = self.trac_ik.get_ik(q_init, *pose)
        if solution is None:
            self.get_logger().warn("IK solution not found.")
        return solution
    
    def set_waypoints(self, waypoints, time_to_reach, send_immediately=False, wait=True):
        """
        Set the waypoints to be published and the time to reach each waypoint.

        Args:
            waypoints (list): A list of lists, where each inner list represents a waypoint.
            time_to_reach (int): The time in seconds to reach each waypoint.
            send_immediately (bool): If True, send the waypoints immediately.
            wait (bool): If True, wait for the result after sending the goal.
        """
        joint_trajectory = []
        for waypoint in waypoints:
            joint_positions = self.solve_ik(waypoint)
            if joint_positions is not None:
                joint_trajectory.append(joint_positions)
            else:
                self.get_logger().warn("No joint positions found for waypoint.")
                return

        self.set_joint_trajectory(joint_trajectory, time_to_reach, send_immediately, wait)
    
    def set_joint_trajectory(self, joint_trajectory, time_to_reach, send_immediately=False, wait=True):
        """
        Set the joint_trajectory to be published and the time to reach each waypoint.

        Args:
            joint_trajectory (list): A list of lists, where each inner list represents a JointTrajectoryPoint.
            time_to_reach (int): The time in seconds to reach each waypoint.
            send_immediately (bool): If True, send the joint_trajectory immediately.
            wait (bool): If True, wait for the result after sending the goal.
        """
        if len(joint_trajectory[0]) != len(self.joints_name):
            self.get_logger().error("Joint trajectory has different number of joints than the joints name list.")
            return
        
        self.goals = []
        for goal_joints_position in joint_trajectory:
            point = JointTrajectoryPoint()
            point.positions = goal_joints_position
            point.time_from_start = Duration(sec=time_to_reach)
            self.goals.append(point)
        self.i = 0

        if send_immediately:
            self.send_goal(wait)

    def send_goal(self, wait=True):
        """
        Send the goal to the action server.

        Args:
            wait (bool): If True, wait for the result after sending the goal.
        """
        if self.goals:
            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joints_name
            goal_msg.trajectory.points = [self.goals[self.i]]

            self.action_client.wait_for_server()
            self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

            if wait:
                rclpy.spin_until_future_complete(self, self._send_goal_future)
                goal_handle = self._send_goal_future.result()
                if goal_handle.accepted:
                    self._get_result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, self._get_result_future)
                    result = self._get_result_future.result()
                    self.get_logger().info(f"Result: {result}")
                else:
                    self.get_logger().info('Goal rejected.')
        else:
            self.get_logger().warn("No goals set to send.")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        #rclpy.shutdown() #remove

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback}")

    def test_jtc_one_target(self, target_pose, time_to_reach):
        """Test JTC with one target pose."""
        joint_positions = self.solve_ik(target_pose)
        if joint_positions is not None:
            self.set_joint_trajectory([joint_positions], time_to_reach, send_immediately=True, wait=True)
        else:
            self.get_logger().warn("No joint positions found for target pose.")

    def test_jtc_waypoints(self, waypoints, time_to_reach):
        """Test JTC with waypoints."""
        self.set_waypoints(waypoints, time_to_reach, send_immediately=True, wait=True)

def main(args=None):
    """
    Main function to initialize and spin the ArmPositionController node.
    """
    rclpy.init(args=args)
    arm_position_controller = ArmPositionController(
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
        print("1. Solve IK")
        print("2. Test JTC with one target pose")
        print("3. Test JTC with waypoints")
        print("4. Exit")

        choice = input("Enter your choice: ")

        if choice == '1':
            # Example pose: [x, y, z, qx, qy, qz, qw]
            target_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
            joint_positions = arm_position_controller.solve_ik(target_pose)
            if joint_positions is not None:
                print(f"IK solution: {joint_positions}")
            else:
                print("No IK solution found.")
        elif choice == '2':
            # Example pose: [x, y, z, qx, qy, qz, qw]
            target_pose = [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
            time_to_reach = 5
            arm_position_controller.test_jtc_one_target(target_pose, time_to_reach)
        elif choice == '3':
            # Example waypoints
            waypoints = [
                [0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0],
                [0.6, 0.1, 0.6, 0.0, 0.0, 0.0, 1.0],
                [0.7, 0.2, 0.7, 0.0, 0.0, 0.0, 1.0]
            ]
            time_to_reach = 5
            arm_position_controller.test_jtc_waypoints(waypoints, time_to_reach)
        elif choice == '4':
            break
        else:
            print("Invalid choice. Please try again.")
        
        rclpy.spin_once(arm_position_controller, timeout_sec=0.1)
    
    arm_position_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
