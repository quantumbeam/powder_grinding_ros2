from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

pkg_name = "grinding_descriptions"


def generate_launch_description():
    planning_scene_config = PathJoinSubstitution(
        [
            get_package_share_directory(pkg_name),
            "config",
            "planning_scene_config.yaml",
        ]
    )

    load_planing_scene_node = Node(
        package=pkg_name,
        executable="load_planing_scene",
        name="load_planing_scene",
        output="screen",
        parameters=[planning_scene_config],
    )

    return LaunchDescription([load_planing_scene_node])
