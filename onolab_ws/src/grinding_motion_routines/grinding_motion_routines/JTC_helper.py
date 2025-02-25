import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class JTCHelper(Node):
    """
    A helper class to publish joint trajectories to a specified controller using actions.

    Attributes:
        controller_name (str): The name of the controller to publish trajectories to.
        joints (list): The list of joint names.
        goals (list): The list of JointTrajectoryPoint goals to publish.
        i (int): The index of the current goal to publish.
    """

    def __init__(self, node_name="jtc_helper", controller_name="joint_trajectory_controller", joints=[""]):
        """
        Initialize the JTCHelper node.

        Args:
            node_name (str): The name of the node.
            controller_name (str): The name of the controller to publish trajectories to.
            joints (list): The list of joint names.
        """
        super().__init__(node_name)

        self.controller_name = controller_name
        self.joints = joints

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        action_topic = "/" + self.controller_name + "/" + "follow_joint_trajectory"
        self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
        self.goals = []
        self.i = 0

    def set_waypoints(self, waypoints, time_to_reach, send_immediately=False, wait=True):
        """
        Set the waypoints to be published and the time to reach each waypoint.

        Args:
            waypoints (list): A list of lists, where each inner list represents a JointTrajectoryPoint.
            time_to_reach (int): The time in seconds to reach each waypoint.
            send_immediately (bool): If True, send the waypoints immediately.
            wait (bool): If True, wait for the result after sending the goal.
        """
        self.goals = []
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
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
        if len(self.goals) > 0:
            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self.joints
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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback}")

def main(args=None):
    """
    Main function to initialize and spin the JTCHelper node.
    """
    rclpy.init(args=args)
    jtc_helper = JTCHelper(
        controller_name="scaled_joint_trajectory_controller",
        joints=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
    )

    # Example waypoints
    p1 = [[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]]
    p2 = [[2.5, 2.0, 1.5, 1.0, 0.5, 0.0]]
    time_to_reach = 5
    jtc_helper.set_waypoints(p1, time_to_reach, send_immediately=True, wait=True)
    jtc_helper.set_waypoints(p2, time_to_reach, send_immediately=True, wait=True)
    rclpy.spin(jtc_helper)

if __name__ == "__main__":
    main()