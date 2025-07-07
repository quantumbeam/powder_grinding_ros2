import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # planning_scene_configの完全パスを外部から指定可能にする
    planning_scene_config_arg = DeclareLaunchArgument(
        "planning_scene_config",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("grinding_scene_description"),
                "config",
                "planning_scene_config.yaml",
            ]
        ),
        description="Full path to planning scene configuration file",
    )

    def launch_setup(context, *args, **kwargs):
        planning_scene_config = LaunchConfiguration("planning_scene_config").perform(
            context
        )

        load_scene_container = ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="load_scene_container",
            namespace="",
            composable_node_descriptions=[
                ComposableNode(
                    package="grinding_scene_description",
                    plugin="grinding_scene_description::PlanningSceneLoader",
                    name="scene_load_node",
                    parameters=[planning_scene_config],
                )
            ],
            output="screen",
        )
        load_scene_node = Node(
            package="grinding_scene_description",
            executable="load_planning_scene_node",
            name="planning_scene_loader",
            parameters=[planning_scene_config],
            output="screen",
        )

        return [
            # load_scene_container,
            load_scene_node
        ]

    return LaunchDescription(
        [planning_scene_config_arg, OpaqueFunction(function=launch_setup)]
    )
