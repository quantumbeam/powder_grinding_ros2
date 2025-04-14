from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

MOVEIT_CONFIG = (
    MoveItConfigsBuilder(robot_name="ur", package_name="grinding_moveit_config") # OK
    .robot_description(file_path = os.path.join(
        get_package_share_directory("grinding_robot_description"),
        "urdf", "ur5e_with_pestle.urdf.xacro"),
        mappings={"name":"ur"}
    )
    .robot_description_semantic(file_path=os.path.join(
        get_package_share_directory("grinding_moveit_config"),
        "srdf", "ur.srdf.xacro"),
        mappings={"name": "ur"}
    )
    .trajectory_execution(file_path= os.path.join(
        get_package_share_directory("grinding_moveit_config"),
        "config","moveit_controllers.yaml")
    )
    .planning_pipelines()
    .to_moveit_configs()
)



# import os
# import yaml
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder

# # MoveIt構成ビルド
# moveit_config_builder = (
#     MoveItConfigsBuilder("grinding_robot", package_name="grinding_moveit_config")
#     .robot_description(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_robot_description"),
#             "urdf",
#             "ur5e_with_pestle.urdf.xacro"
#         ),
#         mappings={
#             "name": "grinding_robot",
#             "tf_prefix": "",
#             "parent": "world",
#             "joint_limits_parameters_file": "config/joint_limits.yaml",
#             "kinematics_parameters_file": "config/kinematics.yaml",
#             "physical_parameters_file": "",
#             "visual_parameters_file": "",
#             "safety_limits": "false",
#             "safety_pos_margin": "0.15",
#             "safety_k_position": "20.0"
#         },
#     )
#     .robot_description_semantic(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "srdf",
#             "ur.srdf.xacro"
#         ),
#         mappings={"name": "grinding_robot"}
#     )
#     .planning_scene_monitor(
#         publish_robot_description=True,
#         publish_robot_description_semantic=True,
#     )
#     .trajectory_execution(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "config",
#             "moveit_controllers.yaml"
#         )
#     )
#     .joint_limits(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "config",
#             "joint_limits.yaml"
#         )
#     )
#     .robot_description_kinematics(
#         file_path=os.path.join(
#             get_package_share_directory("grinding_moveit_config"),
#             "config",
#             "kinematics.yaml"
#         )
#     )
#     .planning_pipelines(
#         pipelines=["ompl"],
#         default_planning_pipeline="ompl"
#     )
# )

# planning_yaml_path = os.path.join(
#     get_package_share_directory("grinding_moveit_config"),
#     "config",
#     "planning.yaml"
# )

# with open(planning_yaml_path, "r") as f:
#     planning_yaml = yaml.safe_load(f)

# print("=== planning.yaml の中身 ===")
# print(planning_yaml)

# # planning.yaml の中身を MOVEIT_CONFIG_DICT に展開
# planning_pipeline_config = planning_yaml.get("planning_pipelines", {}).get("ompl", {})

# MOVEIT_CONFIG_DICT["planning_pipelines"] = ["ompl"]
# MOVEIT_CONFIG_DICT["default_planning_pipeline"] = "ompl"
# MOVEIT_CONFIG_DICT["ompl"] = planning_pipeline_config

# MOVEIT_CONFIG = MOVEIT_CONFIG_DICT


# from moveit_configs_utils import MoveItConfigsBuilder
# import os
# from ament_index_python.packages import get_package_share_directory

# MOVEIT_CONFIG = (
#     MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
#     .robot_description(
#         file_path=os.path.join(
#             get_package_share_directory("ur_description"),
#             "urdf",
#             "ur5e.urdf.xacro"
#         ),
#         mappings = {
#         },

#     )
#     .robot_description_semantic(
#         file_path=os.path.join(
#             get_package_share_directory("ur_moveit_config"),
#             "srdf",
#             "ur.srdf.xacro"
#         ),
#         mappings = {
#             "name": "ur5e",  
#         },

#     )
#     .planning_scene_monitor(
#         publish_robot_description=True,
#         publish_robot_description_semantic=True,
#     )
#     .trajectory_execution(
#         file_path=os.path.join(
#             get_package_share_directory("ur_moveit_config"),
#             "config",
#             "moveit_controllers.yaml"
#         )
#     )
#     .joint_limits(
#         file_path=os.path.join(
#             get_package_share_directory("ur_moveit_config"),
#             "config",
#             "joint_limits.yaml"
#         )
#     )
#     .robot_description_kinematics(
#         file_path=os.path.join(
#             get_package_share_directory("ur_moveit_config"),
#             "config",
#             "kinematics.yaml"
#         )
#     )
#     .
# (
#         pipelines=["ompl"],
#         default_planning_pipeline="ompl",
#     )
#     .to_moveit_configs()
# )

