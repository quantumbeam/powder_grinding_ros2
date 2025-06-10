#!/usr/bin/env python3

import sys
import os
import time  # Import the time module
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

# Import Pinocchio specific modules
import pinocchio as pin
from pinocchio.utils import *


class UR5ePinocchioIKTestNode(Node):
    """
    ROS 2 node to test Inverse Kinematics (IK) and Forward Kinematics (FK)
    for a UR5e robotic arm using the Pinocchio library.
    """

    # Class Constants
    PACKAGE_NAME = "grinding_robot_description"
    URDF_RELATIVE_PATH = "urdf/ur/ur5e_with_pestle.urdf"
    # In Pinocchio, base and tip links are specified by their frame IDs in the model
    TIP_FRAME_NAME = "pestle_tip"  # End-effector frame name

    def __init__(self):
        super().__init__("ur5e_pinocchio_ik_test_node")
        self.get_logger().info("UR5e Pinocchio IK Test Node Started")

        self.model: Optional[pin.Model] = None
        self.data: Optional[pin.Data] = None
        self.tip_frame_id: int = -1  # End-effector frame ID

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
            # Load URDF using Pinocchio to build the model and data
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            end_time = time.perf_counter()
            self.model_load_time_ms = (end_time - start_time) * 1000.0
            self.get_logger().info(f"Successfully loaded Pinocchio model.")

            # Log all frame names in the model
            self.get_logger().info("--- All frames in the loaded Pinocchio model ---")
            for frame_id, frame in enumerate(self.model.frames):
                self.get_logger().info(
                    f"  ID: {frame_id}, Name: '{frame.name}'"
                )  # Added quotes for clear string comparison
            self.get_logger().info("---------------------------------------------")

            # Get the end-effector frame ID
            # FIX: Check against the list of frame names, not joint names (model.names)
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

        # Example seed for UR5e (in radians)
        self.seed_jnt = np.array([-1.57, 0.0, -1.57, 0.0, 0.0, 0.0])

        # Target position (x, y, z)
        self.tgt_pos = np.array([0.0, 0.5, 0.25])
        # Target rotation matrix (identity matrix - no change in orientation)
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.get_logger().info("Initialized IK/FK test parameters.")

    def run_ik_fk_test(self) -> None:
        """
        Executes an Inverse Kinematics (IK) calculation, and then a Forward Kinematics (FK)
        calculation based on the IK result. Logs all calculated times at the end.
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
            # --- IK Calculation (using Pinocchio's Levenberg-Marquardt solver) ---
            # Create target pose as a pin.SE3 object
            target_pose = pin.SE3(self.tgt_rotmat, self.tgt_pos)

            # Initialize IK solver
            # The LM solver minimizes the error function
            # Here, we minimize the error between the target pose and the current frame pose
            q = self.seed_jnt.copy()  # Start from the seed configuration

            # IK solver parameters
            EPS = 1e-6  # Stopping condition error threshold
            MAX_ITERS = 1000  # Maximum number of iterations
            DT = 1e-1  # Time step (influences gradient-based updates)
            damping = 1e-4  # Damping term (for LM solver)

            self.get_logger().info(
                f"Target pose (translation): {target_pose.translation.T}"
            )
            self.get_logger().info(f"Target pose (rotation):\n{target_pose.rotation}")
            self.get_logger().info(f"Seed joint values: {q.T}")

            start_time_ik = time.perf_counter()
            for i in range(MAX_ITERS):
                pin.forwardKinematics(self.model, self.data, q)
                pin.updateFramePlacements(self.model, self.data)

                # Current frame pose
                current_pose = self.data.oMf[self.tip_frame_id]

                # Error (inverse transform of current_pose wrt target_pose)
                # pin.log(M_error) converts SE(3) error to Lie algebra (twist)
                error = pin.log6(
                    current_pose.inverse() * target_pose
                ).vector  # Twist of error

                if np.linalg.norm(error) < EPS:
                    self.get_logger().info(f"IK converged after {i} iterations.")
                    break

                # Compute Jacobian
                # getFrameJacobian returns the Jacobian in the world frame
                J = pin.computeFrameJacobian(
                    self.model,
                    self.data,
                    q,
                    self.tip_frame_id,
                    pin.ReferenceFrame.LOCAL,  # Use Jacobian in the local frame
                )
                # 6xN matrix (N is number of joints)

                # Pseudo-inverse (damped least squares method)
                # J^T J + damping * I
                hessian = J.T @ J + damping * np.eye(J.shape[1])
                grad = J.T @ error
                dq = np.linalg.solve(hessian, grad) * DT # Remove negative sign

                # Update joint angles
                q = pin.integrate(self.model, q, dq)
            else:
                self.get_logger().warn(
                    f"IK did NOT converge after {MAX_ITERS} iterations. Final error: {np.linalg.norm(error):.6f}"
                )

            end_time_ik = time.perf_counter()
            self.ik_calc_time_ms = (end_time_ik - start_time_ik) * 1000.0

            # Store result only if converged
            if np.linalg.norm(error) < EPS:
                self.ik_result = q.copy()
                self.get_logger().info(f"IK solution found: {self.ik_result.T}")
            else:
                self.ik_result = None
                self.get_logger().warn(
                    "IK solution not found (did not converge to desired accuracy)."
                )

            # --- FK Calculation ---
            if self.ik_result is not None:
                start_time_fk = time.perf_counter()
                pin.forwardKinematics(self.model, self.data, self.ik_result)
                pin.updateFramePlacements(
                    self.model, self.data
                )  # Update frame placements
                end_time_fk = time.perf_counter()
                self.fk_calc_time_ms = (end_time_fk - start_time_fk) * 1000.0

                # Get the current pose of the end-effector
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
            self.ik_result = None
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
