#!/usr/bin/env python3

import sys
import os
import time
import numpy as np
from typing import Optional, Tuple, List # Listを追加

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState # 今回は未使用ですが、元のコードにあったため残します
from ament_index_python.packages import get_package_share_directory

from pytracik.trac_ik import TracIK


class UR5eIKTestNode(Node):
    """
    ROS 2 node to test Inverse Kinematics (IK) and Forward Kinematics (FK)
    for a UR5e robotic arm using the TracIK solver.
    この実装では、IKソルバーの収束のブレを評価する機能を追加しています。
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
        self.ik_result: Optional[np.ndarray] = None # FK計算のために最後の成功した結果を保持

        self.solver_init_time_ms: float = 0.0
        self.ik_calc_time_ms: float = 0.0 # 平均IK計算時間
        self.fk_calc_time_ms: float = 0.0

        self._initialize_ik_solver()
        self._initialize_parameters()

        # IK/FKテストと収束ブレ評価を実行
        self.run_ik_fk_test()

        # テスト完了後にノードを終了する
        self.get_logger().info("IK/FK test completed. Shutting down node.")

    def _initialize_ik_solver(self) -> None:
        """
        URDFファイルに基づいてTracIKソルバーを初期化します。
        """
        pkg_share_dir = get_package_share_directory(self.PACKAGE_NAME)
        urdf_path = os.path.join(pkg_share_dir, self.URDF_RELATIVE_PATH)

        if not os.path.exists(urdf_path):
            self.get_logger().error(f"URDFファイルが見つかりません: {urdf_path}")
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        try:
            self.get_logger().info(
                f"URDF: {urdf_path} でTracIKソルバーを初期化しようとしています。"
            )
            start_time = time.perf_counter()
            self.ur5e_iksolver = TracIK(
                base_link_name=self.BASE_LINK_NAME,
                tip_link_name=self.TIP_LINK_NAME,
                urdf_path=urdf_path,
                # TracIKの内部パラメータ（オプション）
                # timeout=0.005, # デフォルトのタイムアウト（秒）
                # epsilon=1e-5, # デフォルトの許容誤差
            )
            end_time = time.perf_counter()
            self.solver_init_time_ms = (end_time - start_time) * 1000.0
            self.get_logger().info(f"TracIKソルバーの初期化に成功しました。")
        except Exception as e:
            self.get_logger().error(f"TracIKソルバーの初期化に失敗しました: {e}")
            raise

    def _initialize_parameters(self) -> None:
        """
        IK/FKテストのためのパラメータ（シード関節値、目標位置、目標回転）を初期化します。
        """
        # UR5eのシード関節値の例 (ラジアン単位)
        # TracIKのデフォルトのシードと同じ姿勢を試す: [-1.57, 0.0, -1.57, 0.0, 0.0, 0.0]
        # ただし、TracIKはモデルの関節の数に合わせて自動的に配列を調整するため、
        # ここではモデルの自由度を確認する必要はない。
        self.seed_jnt = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0]) # Pinocchioと同じ姿勢で試す
        
        # 目標位置の例 (x, y, z)
        # Pinocchioと同じ目標位置で試す
        self.tgt_pos = np.array([0.3, 0.0, 0.3])
        # 目標回転の例 (単位行列 - 向きの変化なし)
        self.tgt_rotmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        self.get_logger().info("IK/FKテストパラメータを初期化しました。")

    def run_ik_fk_test(self) -> None:
        """
        TracIKソルバーを使用して逆運動学（IK）計算を複数回実行し、
        収束のブレを評価します。その後、最後の成功したIK結果に基づいて順運動学（FK）計算を実行します。
        最後にすべての計算時間をログに出力します。
        """
        if self.ur5e_iksolver is None:
            self.get_logger().error(
                "TracIKソルバーが初期化されていません。IK/FKテストを実行できません。"
            )
            return
        if self.seed_jnt is None or self.tgt_pos is None or self.tgt_rotmat is None:
            self.get_logger().error(
                "IK/FKテストパラメータが初期化されていません。IK/FKテストを実行できません。"
            )
            return

        self.get_logger().info("収束のブレ評価のためのIK/FKテストを実行中...")
        
        try:
            # IK計算を複数回実行するための設定
            num_ik_runs = 100 # 収束ブレ評価のためにIKを実行する回数

            all_converged_solutions: List[np.ndarray] = [] # 成功裏に収束した関節設定を格納
            all_final_errors: List[float] = []        # 収束した解の最終位置エラーを格納

            self.get_logger().info(f"ブレ分析のためにIKを {num_ik_runs} 回実行します。")
            self.get_logger().info(f"目標位置: {self.tgt_pos}、目標回転: \n{self.tgt_rotmat}")
            self.get_logger().info(f"初期シード関節値: {self.seed_jnt}")

            total_ik_time_sum = 0.0
            last_successful_ik_result = None # FK計算のために最後の成功したIK結果を一時的に保持

            for run_idx in range(num_ik_runs):
                start_time_ik_single_run = time.perf_counter()
                
                # TracIKのik関数を呼び出す
                # TracIKは内部で収束判定を行うため、エラーや反復回数は直接取得できないことが多い
                result_ik = self.ur5e_iksolver.ik(
                    self.tgt_pos, self.tgt_rotmat, seed_jnt_values=self.seed_jnt
                )
                
                end_time_ik_single_run = time.perf_counter()
                total_ik_time_sum += (end_time_ik_single_run - start_time_ik_single_run) * 1000.0

                if result_ik is not None:
                    # TracIKのFKを使って、見つかったIK解の最終位置エラーを計算
                    pos_fk_check, rot_fk_check = self.ur5e_iksolver.fk(result_ik)
                    
                    # 位置誤差のノルムを最終エラーとする（TracIKは通常、位置と回転の両方を許容誤差に含める）
                    final_pos_error = np.linalg.norm(pos_fk_check - self.tgt_pos)
                    # 回転誤差の評価も可能だが、簡潔にするため位置誤差に焦点を当てる
                    # 回転誤差: trac_ik_lib.PyKDL.Rotation.GetRPY(rot_fk_check.Inverse() * KDL.Rotation.RPY(*target_rpy))
                    
                    all_converged_solutions.append(result_ik.copy())
                    all_final_errors.append(final_pos_error)
                    last_successful_ik_result = result_ik.copy() # 最後の成功した結果を更新
                    self.get_logger().info(f"実行 {run_idx+1}: IK解が見つかりました。最終位置エラー: {final_pos_error:.6f}")
                else:
                    self.get_logger().warn(f"実行 {run_idx+1}: IK解が見つかりませんでした。目標姿勢は到達不能かもしれません。")

            # --- 統計評価 ---
            self.get_logger().info("\n--- IK収束のブレ分析 ---")
            
            if not all_converged_solutions:
                self.get_logger().error("いずれの実行でもIK解が見つかりませんでした。ブレ分析を実行できません。")
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

                # 最終位置エラーの統計
                mean_final_error = np.mean(all_final_errors)
                std_final_error = np.std(all_final_errors)
                min_final_error = np.min(all_final_errors)
                max_final_error = np.max(all_final_errors)

                self.get_logger().info(f"平均最終位置エラー: {mean_final_error:.6f}")
                self.get_logger().info(f"最終位置エラーの標準偏差: {std_final_error:.6f}")
                self.get_logger().info(f"最小最終位置エラー: {min_final_error:.6f}")
                self.get_logger().info(f"最大最終位置エラー: {max_final_error:.6f}")

                # TracIKは反復回数を直接提供しないため、ここではログに出力しない
                # self.get_logger().info("TracIKは収束までの反復回数を直接提供しません。")

                self.ik_calc_time_ms = total_ik_time_sum / num_ik_runs # 1回の実行あたりの平均時間
                self.ik_result = last_successful_ik_result # FK計算のためにセット


            # --- FK計算 (最後の成功したIK結果に対して実行、もしあれば) ---
            if self.ik_result is not None:
                self.get_logger().info("\n--- 最後の成功したIK解に対するFK計算 ---")
                start_time_fk = time.perf_counter()
                pos_fk, rot_fk = self.ur5e_iksolver.fk(self.ik_result)
                end_time_fk = time.perf_counter()
                self.fk_calc_time_ms = (end_time_fk - start_time_fk) * 1000.0
                self.get_logger().info(
                    f"FK結果 - 位置: {pos_fk}、回転: \n{rot_fk}"
                )
                # この特定の解のエラーを再確認
                final_pos_error_check = np.linalg.norm(pos_fk - self.tgt_pos)
                self.get_logger().info(f"最終FK確認エラー（最後の成功したIK用）: {final_pos_error_check:.6f}")
            else:
                self.get_logger().error("\nFK計算のための成功したIK解が見つかりませんでした。")
                self.fk_calc_time_ms = 0.0 # FKは実行されなかった

        except Exception as e:
            self.get_logger().error(f"IK/FK計算中にエラーが発生しました: {e}")
            self.ik_result = None # エラー発生時は結果をリセット
        finally:
            # --- 全体的なパフォーマンス概要 ---
            self.get_logger().info("\n--- 全体的なパフォーマンス概要 ---")
            self.get_logger().info(
                f"TracIKソルバー初期化時間: {self.solver_init_time_ms:.3f} ms"
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
    ROS 2ノードを初期化し、UR5eIKTestNodeを実行します。
    """
    rclpy.init(args=args)
    ur5e_ik_test_node = UR5eIKTestNode()

    # rclpy.spin() や rclpy.spin_once() を呼び出す代わりに、
    # __init__ で run_ik_fk_test() を呼び出し、完了後にノードを終了する
    # これは、ノードが単一のタスクを実行して終了する設計に適している
    ur5e_ik_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()