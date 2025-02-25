import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JTCHelper(Node):
    """
    A helper class to publish joint trajectories to a specified controller.

    Attributes:
        controller_name (str): The name of the controller to publish trajectories to.
        wait_sec_between_publish (int): The number of seconds to wait between publishing trajectories.
        joints (list): The list of joint names.
        goals (list): The list of JointTrajectoryPoint goals to publish.
        i (int): The index of the current goal to publish.
    """

    def __init__(self, node_name="jtc_helper", controller_name="joint_trajectory_controller", wait_sec_between_publish=6, joints=[""]):
        """
        Initialize the JTCHelper node.

        Args:
            node_name (str): The name of the node.
            controller_name (str): The name of the controller to publish trajectories to.
            wait_sec_between_publish (int): The number of seconds to wait between publishing trajectories.
            joints (list): The list of joint names.
        """
        super().__init__(node_name)

        self.controller_name = controller_name
        self.wait_sec_between_publish = wait_sec_between_publish
        self.joints = joints

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        publish_topic = "/" + self.controller_name + "/" + "joint_trajectory"
        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.timer = self.create_timer(self.wait_sec_between_publish, self.timer_callback)
        self.goals = []
        self.i = 0

    def set_waypoints(self, waypoints, time_to_reach, publish_immediately=False):
        """
        Set the waypoints to be published and the time to reach each waypoint.

        Args:
            waypoints (list): A list of lists, where each inner list represents a JointTrajectoryPoint.
            time_to_reach (int): The time in seconds to reach each waypoint.
            publish_immediately (bool): If True, publish the waypoints immediately.
        """
        self.goals = []
        for waypoint in waypoints:
            point = JointTrajectoryPoint()
            point.positions = waypoint
            point.time_from_start = Duration(sec=time_to_reach)
            self.goals.append(point)
        self.i = 0

        if publish_immediately:
            self.publish_waypoints()

    def publish_waypoints(self):
        """
        Publish the waypoints immediately.
        """
        if len(self.goals) > 0:
            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")
            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(self.goals[self.i])
            self.publisher_.publish(traj)
            self.i += 1
            self.i %= len(self.goals)
        else:
            self.get_logger().warn("No goals set to publish.")

    def timer_callback(self):
        """
        Timer callback to publish the next goal in the list.
        """
        self.publish_waypoints()

def main(args=None):
    """
    Main function to initialize and spin the JTCHelper node.
    """
    rclpy.init(args=args)
    jtc_helper = JTCHelper(
        controller_name="scaled_joint_trajectory_controller",
        wait_sec_between_publish=6,
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
    waypoints = [
        [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],
        [2.5, 2.0, 1.5, 1.0, 0.5, 0.0]
    ]
    time_to_reach = 1
    jtc_helper.set_waypoints(waypoints, time_to_reach, publish_immediately=True)

    try:
        rclpy.spin(jtc_helper)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("Keyboard interrupt received. Shutting down node.")
    except Exception as e:
        print(f"Unhandled exception: {e}")

if __name__ == "__main__":
    main()