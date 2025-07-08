import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    motion_config=PathJoinSubstitution([
            get_package_share_directory("grinding_motion_routines"),
            "config",
            "motion_config.yaml"
        ])
    planning_scene_config=PathJoinSubstitution([
            get_package_share_directory("grinding_scene_description"),
            "config",
            "planning_scene_config.yaml"
        ])
    

    # Motion primitive test node
    grinding_demo_node = Node(
        package='grinding_motion_routines',
        executable='grinding_motion_primitive.py',
        name='grinding_demo_node', 
        output='screen',
        prefix='xterm -hold -e',
        parameters=[motion_config,planning_scene_config],
    )

    return LaunchDescription([
        grinding_demo_node
    ])
