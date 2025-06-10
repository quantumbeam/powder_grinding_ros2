#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory

from pytracik.trac_ik import TracIK


class UR5eIKTestNode(Node):
    """
    ROS 2 node to test Inverse Kinematics (IK) and Forward Kinematics (FK)
    for a UR5e robotic arm using the TracIK solver.
    """

    # Class Constants
    PACKAGE_NAME = "grinding_robot_description"
    URDF_RELATIVE_PATH = "urdf/ur/ur5e_with_pestle.urdf"
    BASE_LINK_NAME = "world"
    TIP_LINK_NAME = "pestle_tip"

    def __init__(self):
        super().__init__("ur5e_ik_test_node")
        self.get_logger().info("UR5e IK Test Node Started")

        self.ur5e_iksolver: Optional[TracIK] = None
        self.seed_jnt: Optional[np.ndarray] = None
        self.tgt_pos: Optional[np.ndarray] = None
        self.tgt_rotmat: Optional[np.ndarray] = None
        self.ik_result: Optional[np.ndarray] = None

        self.solver_init_time_ms: float = 0.0
        self.ik_calc_time_ms: float = 0.0
        self.fk_calc_time_ms: float = 0.0

        self._initialize_ik_solver()
        self._initialize_parameters()

        # Execute IK and FK test
        self.run_ik_fk_test()  # テストを実行

        # テスト完了後にノードを終了する
        self.get_logger().info("IK/FK test completed. Shutting down node.")
        # rclpy.spin() を呼び出さずに直接終了処理を行う
        # rclpy.spin_once() は不要な場合が多いため、ここでは直接終了へ

    def _initialize_ik_solver(self) -> None:
        """
        Initializes the TracIK solver based on the URDF file.
        """
        pkg_share_dir = get_package_share_directory(self.PACKAGE_NAME)
        urdf_path = os.path.join(pkg_share_dir, self.URDF_RELATIVE_PATH)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDF file not found at: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        try:
            self.get_logger().info(
                f"Attempting to initialize TracIK solver with URDF: {urdf_path}"
            )
            start_time = time.perf_counter()
            self.ur5e_iksolver = TracIK(
                base_link_name=self.BASE_LINK_NAME,
                tip_link_name=self.TIP_LINK_NAME,
                urdf_path=urdf_path,
            )
            end_time = time.perf_counter()
            self.solver_init_time_ms = (end_time - start_time) * 1000.0
            self.get_logger().info(f"Successfully initialized TracIK solver.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize TracIK solver: {e}")
            raise

    def _initialize_parameters(self) -> None:
        """
        Initializes parameters for IK/FK testing (seed joint values, target position, target rotation).
        """
        # Example seed for UR5e (in radians)
        self.seed_jnt = np.array([-1.57, 0.0, -1.57, 0.0, 0.0, 0.0])
        # Example target position (x, y, z)
        self.tgt_pos = np.array([0.0, 0.5, 0.25])
        # Example target rotation (identity matrix - no change in orientation)
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.get_logger().info("Initialized IK/FK test parameters.")

    def run_ik_fk_test(self) -> None:
        """
        Executes an Inverse Kinematics (IK) calculation, and then a Forward Kinematics (FK)
        calculation based on the IK result. Logs all calculated times at the end.
        """
        if self.ur5e_iksolver is None:
            self.get_logger().error(
                "TracIK solver is not initialized. Cannot run IK/FK test."
            )
            return
        if self.seed_jnt is None or self.tgt_pos is None or self.tgt_rotmat is None:
            self.get_logger().error(
                "IK/FK test parameters are not initialized. Cannot run IK/FK test."
            )
            return

        self.get_logger().info("Running IK and FK test...")
        try:
            # IK Calculation
            self.get_logger().info(
                f"Target position: {self.tgt_pos}, Target rotation: \n{self.tgt_rotmat}"
            )
            self.get_logger().info(f"Seed joint values: {self.seed_jnt}")

            start_time_ik = time.perf_counter()
            result_ik = self.ur5e_iksolver.ik(
                self.tgt_pos, self.tgt_rotmat, seed_jnt_values=self.seed_jnt
            )
            end_time_ik = time.perf_counter()
            self.ik_calc_time_ms = (end_time_ik - start_time_ik) * 1000.0

            if result_ik is not None:
                self.get_logger().info(f"IK solution found: {result_ik}")
                self.ik_result = result_ik

                # FK Calculation
                start_time_fk = time.perf_counter()
                pos_fk, rot_fk = self.ur5e_iksolver.fk(result_ik)
                end_time_fk = time.perf_counter()
                self.fk_calc_time_ms = (end_time_fk - start_time_fk) * 1000.0
                self.get_logger().info(
                    f"FK result - Position: {pos_fk}, Rotation: \n{rot_fk}"
                )
            else:
                self.get_logger().warn(
                    "IK solution not found. The target pose might be unreachable."
                )
                self.ik_result = None

        except Exception as e:
            self.get_logger().error(f"Error during IK/FK calculation: {e}")
            self.ik_result = None
        finally:
            # --- Performance Summary ---
            self.get_logger().info("--- Performance Summary ---")
            self.get_logger().info(
                f"TracIK Solver Initialization: {self.solver_init_time_ms:.3f} ms"
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
    Initializes the ROS 2 node and runs the UR5eIKTestNode.
    """
    rclpy.init(args=args)
    ur5e_ik_test_node = UR5eIKTestNode()

    ur5e_ik_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
