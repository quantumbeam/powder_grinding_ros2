import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

MOVEIT_CONFIG = (
      MoveItConfigsBuilder("grinding_robot", package_name="grinding_moveit_config")
       .robot_description(
           file_path=os.path.join(get_package_share_directory("grinding_robot_description"),
                             "urdf", "ur5e_with_pestle.urdf.xacro"),
           mappings={
               "name":"$(arg name)"
               "tf_prefix"="$(arg tf_prefix)"
               "parent"="world"
               "joint_limits_parameters_file"="$(arg joint_limit_params)"
               "kinematics_parameters_file"="$(arg kinematics_params)"
               "physical_parameters_file"="$(arg physical_params)"
               "visual_parameters_file"="$(arg visual_params)"
               "safety_limits"="$(arg safety_limits)"
               "safety_pos_margin"="$(arg safety_pos_margin)"
               "safety_k_position"="$(arg safety_k_position)"
                 },
        )
       .robot_description_semantic(
            file_path="../grinding_scene_description/launch/load_planning_scene.launch.xml",
            mappings={ "model": "cobotta", "namespace": "",},
       )
       .planning_scene_monitor(
           publish_robot_description=True,
           publish_robot_description_semantic=True,
       )
       .trajectory_execution(file_path="robots/cobotta/config/moveit_controllers.yaml")
       .joint_limits(file_path="robots/cobotta/config/joint_limits.yaml")
       .robot_description_kinematics(file_path="config/kinematics.yaml")
       .planning_pipelines(
           pipelines=["ompl"],
           default_planning_pipeline="ompl",
       )
       .to_moveit_configs()
)