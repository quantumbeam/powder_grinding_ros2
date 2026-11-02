from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motion_config_arg = DeclareLaunchArgument(
        "motion_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("grinding_motion_routines"), "config", "cobotta_motion_config.yaml"]
        ),
        description="Full path to the motion configuration file",
    )

    # NOTE: "name" is not overridden here, because a node name remap would rename
    # all nodes in the process (the demo node and its JTC helper node).
    cobotta_grinding_demo_node = Node(
        package="grinding_motion_routines",
        executable="cobotta_grinding_demo.py",
        output="screen",
        parameters=[LaunchConfiguration("motion_config")],
    )

    return LaunchDescription([motion_config_arg, cobotta_grinding_demo_node])
