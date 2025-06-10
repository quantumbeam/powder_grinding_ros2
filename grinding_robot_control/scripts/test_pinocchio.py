#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # Kept from original, though not used in this context
from ament_index_python.packages import get_package_share_directory

# Import Pinocchio specific modules
import pinocchio as pin
# from pinocchio.utils import * # Commented out as it's not explicitly used

# Print environment paths (for debugging)
print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
print("SYS.PATH:", sys.path)

class UR5ePinocchioIKTestNode(Node):
    """
    ROS 2 node to test Inverse Kinematics (IK) and Forward Kinematics (FK)
    for a UR5e robotic arm using the Pinocchio library.
    This implementation manually builds the IK solver to accommodate API constraints
    from a ROS-bundled Pinocchio installation.
    """

    # Class Constants
    PACKAGE_NAME = "grinding_robot_description"
    URDF_RELATIVE_PATH = "urdf/ur/ur5e_with_pestle.urdf"
    TIP_FRAME_NAME = "pestle_tip"  # End-effector frame name

    def __init__(self):
        super().__init__("ur5e_pinocchio_ik_test_node")
        self.get_logger().info("UR5e Pinocchio IK Test Node Started")
        self.get_logger().info(f"Pinocchio version: {pin.__version__}")
        self.get_logger().info(f"Pinocchio path: {pin.__file__}") # Log Pinocchio's path

        self.model: Optional[pin.Model] = None
        self.data: Optional[pin.Data] = None
        self.tip_frame_id: int = -1

        self.seed_jnt: Optional[np.ndarray] = None
        self.tgt_pos: Optional[np.ndarray] = None
        self.tgt_rotmat: Optional[np.ndarray] = None
        self.ik_result: Optional[np.ndarray] = None

        self.model_load_time_ms: float = 0.0
        self.ik_calc_time_ms: float = 0.0
        self.fk_calc_time_ms: float = 0.0

        self._initialize_robot_model()
        self._initialize_parameters()

        # Execute IK and FK test
        self.run_ik_fk_test()

    def _initialize_robot_model(self) -> None:
        """
        Loads the robot model from URDF using Pinocchio.
        """
        pkg_share_dir = get_package_share_directory(self.PACKAGE_NAME)
        urdf_path = os.path.join(pkg_share_dir, self.URDF_RELATIVE_PATH)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found at: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        try:
            self.get_logger().info(
                f"Attempting to load robot model from URDF: {urdf_path}"
            )
            start_time = time.perf_counter()
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            end_time = time.perf_counter()
            self.model_load_time_ms = (end_time - start_time) * 1000.0
            self.get_logger().info("Successfully loaded Pinocchio model.")

            if self.TIP_FRAME_NAME not in [frame.name for frame in self.model.frames]:
                self.get_logger().error(
                    f"Tip frame '{self.TIP_FRAME_NAME}' not found in URDF model."
                )
                raise ValueError(f"Tip frame '{self.TIP_FRAME_NAME}' not found.")
            self.tip_frame_id = self.model.getFrameId(self.TIP_FRAME_NAME)

            self.get_logger().info(
                f"Robot model has {self.model.nq} degrees of freedom (DOF)."
            )
            self.get_logger().info(
                f"Tip frame '{self.TIP_FRAME_NAME}' has ID: {self.tip_frame_id}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to load robot model with Pinocchio: {e}")
            raise

    def _initialize_parameters(self) -> None:
        """
        Initializes parameters for IK/FK testing (seed joint values, target position, target rotation).
        """
        if self.model is None:
            self.get_logger().error(
                "Robot model not loaded. Cannot initialize parameters."
            )
            raise RuntimeError("Robot model not loaded.")

        self.seed_jnt = pin.neutral(self.model)

        # Set specific values for UR5e's 6 revolute joints.
        ur5e_joint_values = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

        if self.model.nq >= len(ur5e_joint_values):
            self.seed_jnt[0:len(ur5e_joint_values)] = ur5e_joint_values
        else:
            self.get_logger().warn(
                f"Model DOF ({self.model.nq}) is less than expected UR5e joints ({len(ur5e_joint_values)})."
                f" Using a truncated seed."
            )
            self.seed_jnt[:self.model.nq] = ur5e_joint_values[:self.model.nq]

        self.tgt_pos = np.array([0.3, 0.0, 0.3])
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.get_logger().info("Initialized IK/FK test parameters.")

    def run_ik_fk_test(self) -> None:
        """
        Executes an Inverse Kinematics (IK) calculation using a manual Levenberg-Marquardt solver,
        and then a Forward Kinematics (FK) calculation based on the IK result.
        Logs all calculated times at the end.
        """
        if self.model is None or self.data is None or self.tip_frame_id == -1:
            self.get_logger().error(
                "Robot model or tip frame not initialized. Cannot run IK/FK test."
            )
            return
        if self.seed_jnt is None or self.tgt_pos is None or self.tgt_rotmat is None:
            self.get_logger().error(
                "IK/FK test parameters are not initialized. Cannot run IK/FK test."
            )
            return

        self.get_logger().info("Running IK and FK test...")
        try:
            # --- IK Calculation (Manual Levenberg-Marquardt approach) ---
            target_pose = pin.SE3(self.tgt_rotmat, self.tgt_pos)

            # IK parameters
            max_iterations = 1000
            tolerance = 1e-6 # Positional/orientational error tolerance in meters/radians
            damping = 1e-4   # Damping factor for Levenberg-Marquardt

            q_current = self.seed_jnt.copy()
            self.ik_result = None

            self.get_logger().info(
                f"Target pose (translation): {target_pose.translation.T}"
            )
            self.get_logger().info(f"Target pose (rotation):\n{target_pose.rotation}")
            self.get_logger().info(f"Seed joint values: {self.seed_jnt.T}")

            start_time_ik = time.perf_counter()

            for i in range(max_iterations):
                # 1. Compute Forward Kinematics
                pin.forwardKinematics(self.model, self.data, q_current)
                pin.updateFramePlacements(self.model, self.data) # Update frame placements

                current_pose = self.data.oMf[self.tip_frame_id]

                # 2. Compute Error (Pose difference in the tangent space)
                error_se3 = pin.log6(current_pose.inverse() * target_pose)
                # Calculate the norm of the full error vector (linear and angular)
                error_norm = np.linalg.norm(error_se3.vector)

                # Check for convergence
                if error_norm < tolerance:
                    self.ik_result = q_current.copy()
                    self.get_logger().info(f"IK converged after {i+1} iterations. Final error: {error_norm:.6f}")
                    break

                # 3. Compute Jacobian
                # Jacobian wrt the tip frame in the LOCAL_WORLD_ALIGNED frame
                jacobian = pin.computeFrameJacobian(
                    self.model, self.data, q_current, self.tip_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )

                # The computeFrameJacobian function already returns a 6xNV matrix,
                # where NV is the number of generalized velocities.
                # So, no need to trim rows like jacobian[:6, :].

                # 4. Compute Pseudo-Inverse with Damping (Levenberg-Marquardt step)
                # delta_q = (J^T J + lambda*I)^-1 J^T * error_se3.vector
                J_T = jacobian.T
                lambda_I = damping * np.identity(jacobian.shape[1]) # Identity matrix for joint velocities
                
                # Solve the linear system: (J_T @ J + lambda_I) * delta_q = J_T @ error_se3.vector
                delta_q = np.linalg.solve(J_T @ jacobian + lambda_I, J_T @ error_se3.vector)

                # 5. Update Joint Configuration
                # Safely integrate the joint increments into the current configuration space
                q_current = pin.integrate(self.model, q_current, delta_q)

            else: # Loop finished without breaking (max iterations reached, but not converged)
                self.get_logger().warn(f"IK did not converge after {max_iterations} iterations. Final error: {error_norm:.6f}")
                self.ik_result = q_current.copy() # Store final configuration even if not converged

            end_time_ik = time.perf_counter()
            self.ik_calc_time_ms = (end_time_ik - start_time_ik) * 1000.0

            if self.ik_result is not None:
                self.get_logger().info(f"IK solution found (Manual LM): {self.ik_result.T}")
                # Recalculate pose to confirm how close the final IK solution is to the target
                pin.forwardKinematics(self.model, self.data, self.ik_result)
                pin.updateFramePlacements(self.model, self.data)
                final_pose = self.data.oMf[self.tip_frame_id]
                final_error_se3 = pin.log6(final_pose.inverse() * target_pose)
                final_error_norm = np.linalg.norm(final_error_se3.vector)
                self.get_logger().info(f"Final IK error: {final_error_norm:.6f}")
            else:
                self.get_logger().warn("IK solution not found (Manual LM did not converge sufficiently).")

            # --- FK Calculation ---
            if self.ik_result is not None:
                start_time_fk = time.perf_counter()
                pin.forwardKinematics(self.model, self.data, self.ik_result)
                pin.updateFramePlacements(self.model, self.data)
                end_time_fk = time.perf_counter()
                self.fk_calc_time_ms = (end_time_fk - start_time_fk) * 1000.0

                fk_pose = self.data.oMf[self.tip_frame_id]
                pos_fk = fk_pose.translation
                rot_fk = fk_pose.rotation
                self.get_logger().info(
                    f"FK result - Position: {pos_fk.T}, Rotation: \n{rot_fk}"
                )
            else:
                self.get_logger().error("IK solution not available for FK calculation.")

        except Exception as e:
            self.get_logger().error(
                f"Error during IK/FK calculation with Pinocchio: {e}"
            )
            self.ik_result = None # Reset result on error
        finally:
            # --- Performance Summary ---
            self.get_logger().info("--- Performance Summary ---")
            self.get_logger().info(
                f"Pinocchio Model Loading Time: {self.model_load_time_ms:.3f} ms"
            )
            self.get_logger().info(
                f"IK Calculation Time: {self.ik_calc_time_ms:.3f} ms"
            )
            self.get_logger().info(
                f"FK Calculation Time: {self.fk_calc_time_ms:.3f} ms"
            )
            self.get_logger().info("---------------------------")


def main(args: Optional[list] = None) -> None:
    """
    Initializes the ROS 2 node and runs the UR5ePinocchioIKTestNode.
    """
    rclpy.init(args=args)
    ur5e_pinocchio_ik_test_node = UR5ePinocchioIKTestNode()
    ur5e_pinocchio_ik_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()