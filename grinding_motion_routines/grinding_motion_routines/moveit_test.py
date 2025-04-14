#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def main():
    rclpy.init()

    # パス取得
    description_pkg = get_package_share_directory("grinding_robot_description")
    config_pkg = get_package_share_directory("grinding_moveit_config")

    # MoveIt 設定の構築
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="grinding_moveit_config")
        .robot_description_semantic(
            file_path=os.path.join(config_pkg, "srdf", "ur.srdf.xacro"),
            mappings={"name": "ur5e"}
        )
        .to_moveit_configs()
    )
    
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="grinding", package_name="grinding_moveit_config")
    #     .robot_description(
    #         file_path=os.path.join(description_pkg, "urdf", "grinding.urdf.xacro"),
    #         mappings={"name": "grinding"}
    #     )
    #     .robot_description_semantic(
    #         file_path=os.path.join(config_pkg, "srdf", "grinding.srdf.xacro"),
    #         mappings={"name": "grinding"}
    #     )
    #     .trajectory_execution(
    #         file_path=os.path.join(config_pkg, "config", "moveit_controllers.yaml")
    #     )
    #     # .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
    #     .to_moveit_configs()
    # )

    robot = MoveItPy(node_name="moveit_py", config_dict=moveit_config.to_dict())

    print("MoveItPy successfully initialized!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()



# import rclpy
# from moveit.planning import MoveItPy
# from config import MOVEIT_CONFIG
# import os
# from ament_index_python.packages import get_package_share_directory

# def main():
#     rclpy.init()

#     # MoveIt 設定を辞書形式で取得
#     config_dict = MOVEIT_CONFIG.to_dict()

#     # プランニング設定ファイルのパス
#     planner_yaml_path = os.path.join(
#         get_package_share_directory("grinding_moveit_config"),
#         "config",
#         "moveit_planners.yaml"
#     )

#     # MoveItPy の初期化（YAML をパラメータファイルとして読み込む）
#     robot = MoveItPy(
#         node_name="moveit_py",
#         config_dict=config_dict,
#         launch_params_filepaths=[planner_yaml_path]
#     )

#     # 必要な処理があればここに（例：planning_component を取得するなど）

#     # 終了処理
#     robot.shutdown()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()



# import rclpy
# import yaml
# from moveit.planning import MoveItPy
# from moveit_configs_utils import MoveItConfigsBuilder
# import os
# from ament_index_python.packages import get_package_share_directory

# def load_yaml(package_name, relative_path):
#     yaml_path = os.path.join(get_package_share_directory(package_name), relative_path)
#     with open(yaml_path, 'r') as f:
#         return yaml.safe_load(f)

# # MoveIt設定構築
# MOVEIT_CONFIG = (
#     MoveItConfigsBuilder("ur", package_name="grinding_moveit_config")
#     .robot_description(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_robot_description"),
#             "urdf",
#             "ur5e_with_pestle.urdf.xacro"
#         ),
#         mappings={"name": "ur"}
#     )
#     .robot_description_semantic(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "srdf",
#             "ur.srdf.xacro"
#         ),
#         mappings={"name": "ur"}
#     )
#     .trajectory_execution(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "config",
#             "moveit_controllers.yaml"
#         )
#     )
#     .to_moveit_configs()
# )

# def main():
#     rclpy.init()

#     config_dict = MOVEIT_CONFIG.to_dict()

#     # YAMLファイルを明示的に追加
#     from ament_index_python.packages import get_package_share_directory
#     import os, yaml
#     planner_yaml_path = os.path.join(
#         get_package_share_directory("grinding_moveit_config"),
#         "config",
#         "moveit_planners.yaml"
#     )
#     with open(planner_yaml_path, "r") as f:
#         planner_yaml = yaml.safe_load(f)

#     # 明示的に config_dict に反映
#     config_dict["planning_pipelines"] = planner_yaml["planning_pipelines"]
#     config_dict["default_planning_pipeline"] = planner_yaml["default_planning_pipeline"]


#     # MoveItPy に渡すノードパラメータを定義
#     node_parameters = {
#         "planning_pipelines": planner_yaml["planning_pipelines"],
#         "default_planning_pipeline": planner_yaml["default_planning_pipeline"]
#     }

#     robot = MoveItPy(node_name="moveit_py", config_dict=config_dict, node_parameters=node_parameters)

#     print("MoveItPy initialized successfully!")
#     robot.shutdown()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
