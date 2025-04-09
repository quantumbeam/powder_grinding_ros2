import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIG = (
    MoveItConfigsBuilder("grinding_robot", package_name="grinding_moveit_config")
    .robot_description(
        file_path=os.path.join(
            get_package_share_directory("grinding_robot_description"),
            "urdf",
            "ur5e_with_pestle.urdf.xacro"
        ),
        mappings={
            "robot_ip": "192.168.56.101",
            "tf_prefix": "",
            "parent": "world",
            "joint_limits_parameters_file": "",
            "kinematics_parameters_file": "",
            "physical_parameters_file": "",
            "visual_parameters_file": "",
            "safety_limits": "",
            "safety_pos_margin": "",
            "safety_k_position": ""
        },
    )
    .robot_description_semantic(
        file_path=os.path.join(
            get_package_share_directory("grinding_moveit_config"),
            "config",
            "ur.srdf.xacro" 
        )
    )
    .planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True,
    )
    .trajectory_execution(
        file_path=os.path.join(
            get_package_share_directory("grinding_moveit_config"),
            "config",
            "moveit_controllers.yaml"
        )
    )
    .joint_limits(
        file_path=os.path.join(
            get_package_share_directory("grinding_moveit_config"),
            "config",
            "joint_limits.yaml"
        )
    )
    .robot_description_kinematics(
        file_path=os.path.join(
            get_package_share_directory("grinding_moveit_config"),
            "config",
            "kinematics.yaml"
        )
    )
    .planning_pipelines(
        pipelines=["ompl"],
        default_planning_pipeline="ompl"
    )
    .to_moveit_configs()
)