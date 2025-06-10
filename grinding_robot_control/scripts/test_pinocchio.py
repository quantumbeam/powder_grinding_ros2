#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # 今回は未使用ですが、元のコードにあったため残します
from ament_index_python.packages import get_package_share_directory

# Pinocchio関連モジュールのインポート
import pinocchio as pin
# from pinocchio.utils import * # utilsモジュールは明示的に使われていないためコメントアウト

# 環境パスの表示（デバッグ用）
print("PYTHONPATH:", os.environ.get('PYTHONPATH'))
print("SYS.PATH:", sys.path)

class UR5ePinocchioIKTestNode(Node):
    """
    ROS 2 node to test Inverse Kinematics (IK) and Forward Kinematics (FK)
    for a UR5e robotic arm using the Pinocchio library.
    この実装では、PinocchioのROSバンドルAPIの制約に対応するため、
    手動でIKソルバーを構築し、さらにその収束のブレを評価する機能を追加しています。
    """

    # クラス定数
    PACKAGE_NAME = "grinding_robot_description"
    URDF_RELATIVE_PATH = "urdf/ur/ur5e_with_pestle.urdf"
    TIP_FRAME_NAME = "pestle_tip"  # エンドエフェクタのフレーム名

    def __init__(self):
        super().__init__("ur5e_pinocchio_ik_test_node")
        self.get_logger().info("UR5e Pinocchio IK Test Node Started")
        self.get_logger().info(f"Pinocchio version: {pin.__version__}")
        self.get_logger().info(f"Pinocchio path: {pin.__file__}") # Pinocchioのパスをログに出力

        self.model: Optional[pin.Model] = None
        self.data: Optional[pin.Data] = None
        self.tip_frame_id: int = -1

        self.seed_jnt: Optional[np.ndarray] = None
        self.tgt_pos: Optional[np.ndarray] = None
        self.tgt_rotmat: Optional[np.ndarray] = None
        self.ik_result: Optional[np.ndarray] = None # FK計算のために最後の成功した結果を保持

        self.model_load_time_ms: float = 0.0
        self.ik_calc_time_ms: float = 0.0 # 平均IK計算時間
        self.fk_calc_time_ms: float = 0.0

        self._initialize_robot_model()
        self._initialize_parameters()

        # IK/FKテストと収束ブレ評価を実行
        self.run_ik_fk_test()

    def _initialize_robot_model(self) -> None:
        """
        Pinocchioを使ってURDFからロボットモデルをロードします。
        """
        pkg_share_dir = get_package_share_directory(self.PACKAGE_NAME)
        urdf_path = os.path.join(pkg_share_dir, self.URDF_RELATIVE_PATH)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDFファイルが見つかりません: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        try:
            self.get_logger().info(
                f"URDFからロボットモデルをロードしようとしています: {urdf_path}"
            )
            start_time = time.perf_counter()
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            end_time = time.perf_counter()
            self.model_load_time_ms = (end_time - start_time) * 1000.0
            self.get_logger().info("Pinocchioモデルのロードに成功しました。")

            if self.TIP_FRAME_NAME not in [frame.name for frame in self.model.frames]:
                self.get_logger().error(
                    f"先端フレーム '{self.TIP_FRAME_NAME}' がURDFモデルに見つかりません。"
                )
                raise ValueError(f"Tip frame '{self.TIP_FRAME_NAME}' not found.")
            self.tip_frame_id = self.model.getFrameId(self.TIP_FRAME_NAME)

            self.get_logger().info(
                f"ロボットモデルは {self.model.nq} 自由度 (DOF) を持っています。"
            )
            self.get_logger().info(
                f"先端フレーム '{self.TIP_FRAME_NAME}' のIDは: {self.tip_frame_id}"
            )

        except Exception as e:
            self.get_logger().error(f"Pinocchioでロボットモデルのロードに失敗しました: {e}")
            raise

    def _initialize_parameters(self) -> None:
        """
        IK/FKテストのためのパラメータ（シード関節値、目標位置、目標回転）を初期化します。
        """
        if self.model is None:
            self.get_logger().error(
                "ロボットモデルがロードされていません。パラメータを初期化できません。"
            )
            raise RuntimeError("Robot model not loaded.")

        self.seed_jnt = pin.neutral(self.model)

        # UR5eの6つの回転関節に具体的な値を設定します。
        ur5e_joint_values = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])

        if self.model.nq >= len(ur5e_joint_values):
            self.seed_jnt[0:len(ur5e_joint_values)] = ur5e_joint_values
        else:
            self.get_logger().warn(
                f"モデルのDOF ({self.model.nq}) が期待されるUR5eの関節数 ({len(ur5e_joint_values)}) より少ないです。"
                f"シードを切り詰めて使用します。"
            )
            self.seed_jnt[:self.model.nq] = ur5e_joint_values[:self.model.nq]

        self.tgt_pos = np.array([0.3, 0.0, 0.3])
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.get_logger().info("IK/FKテストパラメータを初期化しました。")

    def run_ik_fk_test(self) -> None:
        """
        手動Levenberg-Marquardtソルバーを使用して逆運動学（IK）計算を複数回実行し、
        収束のブレを評価します。その後、最後の成功したIK結果に基づいて順運動学（FK）計算を実行します。
        最後にすべての計算時間をログに出力します。
        """
        if self.model is None or self.data is None or self.tip_frame_id == -1:
            self.get_logger().error(
                "ロボットモデルまたは先端フレームが初期化されていません。IK/FKテストを実行できません。"
            )
            return
        if self.seed_jnt is None or self.tgt_pos is None or self.tgt_rotmat is None:
            self.get_logger().error(
                "IK/FKテストパラメータが初期化されていません。IK/FKテストを実行できません。"
            )
            return

        self.get_logger().info("収束のブレ評価のためのIK/FKテストを実行中...")
        
        try:
            # --- IK計算 (手動Levenberg-Marquardtアプローチ) ---
            target_pose = pin.SE3(self.tgt_rotmat, self.tgt_pos)

            # IKパラメータ
            max_iterations = 1000
            tolerance = 1e-6 # 位置/姿勢誤差の許容誤差（メートル/ラジアン）
            damping = 1e-4   # Levenberg-Marquardtのためのダンピング係数
            
            num_ik_runs = 100 # 収束ブレ評価のためにIKを実行する回数

            all_converged_solutions: List[np.ndarray] = [] # 成功裏に収束した関節設定を格納
            all_final_errors: List[float] = []        # 収束した解の最終エラーを格納
            all_iterations_taken: List[int] = []    # 収束にかかった反復回数を格納

            self.get_logger().info(f"ブレ分析のためにIKを {num_ik_runs} 回実行します。")
            self.get_logger().info(f"目標姿勢（並進）: {target_pose.translation.T}")
            self.get_logger().info(f"目標姿勢（回転）:\n{target_pose.rotation}")
            self.get_logger().info(f"初期シード関節値: {self.seed_jnt.T}")

            total_ik_time_sum = 0.0
            last_successful_ik_result = None # FK計算のために最後の成功したIK結果を一時的に保持

            for run_idx in range(num_ik_runs):
                q_current = self.seed_jnt.copy() # 毎回同じシードから開始
                
                start_time_ik_single_run = time.perf_counter()
                converged_in_run = False
                current_error_norm = float('inf') # 初期エラーを無限大に設定
                iterations_this_run = 0

                for i in range(max_iterations):
                    # 1. 順運動学の計算
                    pin.forwardKinematics(self.model, self.data, q_current)
                    pin.updateFramePlacements(self.model, self.data)

                    current_pose = self.data.oMf[self.tip_frame_id]

                    # 2. エラーの計算 (接空間での姿勢差分)
                    error_se3 = pin.log6(current_pose.inverse() * target_pose)
                    error_norm_current_step = np.linalg.norm(error_se3.vector) # 現在のステップでのエラーノルム

                    # 収束判定
                    if error_norm_current_step < tolerance:
                        converged_in_run = True
                        iterations_this_run = i + 1
                        current_error_norm = error_norm_current_step # 収束時の最終エラーを記録
                        break

                    # 3. ヤコビアンの計算
                    jacobian = pin.computeFrameJacobian(
                        self.model, self.data, q_current, self.tip_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                    )

                    # 4. ダンピング付き擬似逆行列の計算 (Levenberg-Marquardtステップ)
                    J_T = jacobian.T
                    lambda_I = damping * np.identity(jacobian.shape[1])
                    
                    delta_q = np.linalg.solve(J_T @ jacobian + lambda_I, J_T @ error_se3.vector)

                    # 5. 関節設定の更新
                    q_current = pin.integrate(self.model, q_current, delta_q)
                
                end_time_ik_single_run = time.perf_counter()
                total_ik_time_sum += (end_time_ik_single_run - start_time_ik_single_run) * 1000.0

                if converged_in_run:
                    all_converged_solutions.append(q_current.copy())
                    all_final_errors.append(current_error_norm)
                    all_iterations_taken.append(iterations_this_run)
                    last_successful_ik_result = q_current.copy() # 最後の成功した結果を更新
                else:
                    self.get_logger().warn(f"実行 {run_idx+1}: IKは収束しませんでした。最終エラー: {current_error_norm:.6f}")

            # --- 統計評価 ---
            self.get_logger().info("\n--- IK収束のブレ分析 ---")
            
            if not all_converged_solutions:
                self.get_logger().error("いずれの実行でもIK解が収束しませんでした。ブレ分析を実行できません。")
                self.ik_calc_time_ms = total_ik_time_sum / num_ik_runs if num_ik_runs > 0 else 0.0
            else:
                num_successful_runs = len(all_converged_solutions)
                self.get_logger().info(f"成功したIK実行の総数: {num_ik_runs} 回中 {num_successful_runs} 回")

                # NumPy配列に変換して統計計算を容易にする
                converged_solutions_array = np.array(all_converged_solutions)

                # 関節解の統計
                mean_joint_solution = np.mean(converged_solutions_array, axis=0)
                std_joint_solution = np.std(converged_solutions_array, axis=0)
                min_joint_solution = np.min(converged_solutions_array, axis=0)
                max_joint_solution = np.max(converged_solutions_array, axis=0)

                self.get_logger().info(f"平均収束関節解: {mean_joint_solution.T}")
                self.get_logger().info(f"収束関節解の標準偏差: {std_joint_solution.T}")
                self.get_logger().info(f"最小収束関節解: {min_joint_solution.T}")
                self.get_logger().info(f"最大収束関節解: {max_joint_solution.T}")

                # 最終エラーの統計
                mean_final_error = np.mean(all_final_errors)
                std_final_error = np.std(all_final_errors)
                min_final_error = np.min(all_final_errors)
                max_final_error = np.max(all_final_errors)

                self.get_logger().info(f"平均最終エラー: {mean_final_error:.6f}")
                self.get_logger().info(f"最終エラーの標準偏差: {std_final_error:.6f}")
                self.get_logger().info(f"最小最終エラー: {min_final_error:.6f}")
                self.get_logger().info(f"最大最終エラー: {max_final_error:.6f}")

                # 収束にかかった反復回数の統計
                mean_iterations = np.mean(all_iterations_taken)
                std_iterations = np.std(all_iterations_taken)
                min_iterations = np.min(all_iterations_taken)
                max_iterations = np.max(all_iterations_taken)

                self.get_logger().info(f"収束までの平均反復回数: {mean_iterations:.1f}")
                self.get_logger().info(f"収束までの反復回数の標準偏差: {std_iterations:.1f}")
                self.get_logger().info(f"収束までの最小反復回数: {min_iterations}")
                self.get_logger().info(f"収束までの最大反復回数: {max_iterations}")

                self.ik_calc_time_ms = total_ik_time_sum / num_ik_runs # 1回の実行あたりの平均時間
                self.ik_result = last_successful_ik_result # FK計算のためにセット


            # --- FK計算 (最後の成功したIK結果に対して実行、もしあれば) ---
            if self.ik_result is not None:
                self.get_logger().info("\n--- 最後の成功したIK解に対するFK計算 ---")
                start_time_fk = time.perf_counter()
                pin.forwardKinematics(self.model, self.data, self.ik_result)
                pin.updateFramePlacements(self.model, self.data)
                end_time_fk = time.perf_counter()
                self.fk_calc_time_ms = (end_time_fk - start_time_fk) * 1000.0

                fk_pose = self.data.oMf[self.tip_frame_id]
                pos_fk = fk_pose.translation
                rot_fk = fk_pose.rotation
                self.get_logger().info(
                    f"FK結果 - 位置: {pos_fk.T}、回転: \n{rot_fk}"
                )
                # この特定の解のエラーを再確認
                final_error_se3 = pin.log6(fk_pose.inverse() * target_pose)
                final_error_norm = np.linalg.norm(final_error_se3.vector)
                self.get_logger().info(f"最終FK確認エラー（最後の成功したIK用）: {final_error_norm:.6f}")
            else:
                self.get_logger().error("\nFK計算のための成功したIK解が見つかりませんでした。")
                self.fk_calc_time_ms = 0.0 # FKは実行されなかった

        except Exception as e:
            self.get_logger().error(
                f"PinocchioでのIK/FK計算中にエラーが発生しました: {e}"
            )
            self.ik_result = None # エラー発生時は結果をリセット
        finally:
            # --- 全体的なパフォーマンス概要 ---
            self.get_logger().info("\n--- 全体的なパフォーマンス概要 ---")
            self.get_logger().info(
                f"Pinocchioモデルロード時間: {self.model_load_time_ms:.3f} ms"
            )
            self.get_logger().info(
                f"IK計算時間（1実行あたり平均）: {self.ik_calc_time_ms:.3f} ms"
            )
            self.get_logger().info(
                f"FK計算時間（最後の成功したIK用）: {self.fk_calc_time_ms:.3f} ms"
            )
            self.get_logger().info("-----------------------------------")


def main(args: Optional[list] = None) -> None:
    """
    ROS 2ノードを初期化し、UR5ePinocchioIKTestNodeを実行します。
    """
    rclpy.init(args=args)
    ur5e_pinocchio_ik_test_node = UR5ePinocchioIKTestNode()
    ur5e_pinocchio_ik_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()