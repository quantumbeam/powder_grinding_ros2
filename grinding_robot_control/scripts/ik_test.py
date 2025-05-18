import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Add pytracik to the path if it's not in the standard locations
sys.path.append('/home/ubuntu/user/grinding_ws/src/powder_grinding/pytracik')
from pytracik.trac_ik import TracIK

class UR5eIKTestNode(Node):
    def __init__(self):
        super().__init__('ur5e_ik_test_node')
        self.get_logger().info("UR5e IK Test Node Started")

        # URDF path - Now using ur5e_with_pestle.urdf.xacro
        # Get the package share directory
        pkg_share_dir = get_package_share_directory('grinding_robot_description')
        urdf_path = os.path.join(pkg_share_dir, "urdf/ur5e_with_pestle.urdf")
        
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found at: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        # IK solver
        self.ur5e_iksolver = TracIK(base_link_name="world",  # Changed base link
                                   tip_link_name="pestle_tip",  # Changed tip link
                                   urdf_path=urdf_path)

        # Test parameters (adjust these for the UR5e)
        self.seed_jnt = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])  # Example seed for UR5e
        self.tgt_pos = np.array([0.3, 0.0, 0.3])  # Example target position
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]])  # Example target rotation (identity)

        # Publisher for JointState (for visualization)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_state)

        # Perform IK and FK
        self.run_ik_fk_test()

    def run_ik_fk_test(self):
        self.get_logger().info("Running IK and FK test...")
        try:
            # IK
            result = self.ur5e_iksolver.ik(self.tgt_pos, self.tgt_rotmat, seed_jnt_values=self.seed_jnt)
            if result is not None:
                self.get_logger().info(f"The IK solution is: {result}")
                # FK
                pos_fk, rot_fk = self.ur5e_iksolver.fk(result)
                self.get_logger().info(f"The FK result is: pos={pos_fk}, rot={rot_fk}")
                self.ik_result = result
            else:
                self.get_logger().error("IK solution not found.")
                self.ik_result = None
        except Exception as e:
            self.get_logger().error(f"Error during IK/FK calculation: {e}")
            self.ik_result = None

    def publish_joint_state(self):
        if self.ik_result is not None:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]
            joint_state_msg.position = self.ik_result.tolist()
            self.joint_state_publisher.publish(joint_state_msg)
            self.get_logger().info("Published joint state")
        else:
            self.get_logger().warn("IK result is None, not publishing joint state.")

def main(args=None):
    rclpy.init(args=args)
    ur5e_ik_test_node = UR5eIKTestNode()
    rclpy.spin(ur5e_ik_test_node)
    ur5e_ik_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
