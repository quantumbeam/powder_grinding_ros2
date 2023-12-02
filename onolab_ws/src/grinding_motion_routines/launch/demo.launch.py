from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name = "grinding_motion_routines"


def generate_launch_description():
    ld = LaunchDescription()

    motion_config = PathJoinSubstitution(
        [
            get_package_share_directory(pkg_name),
            "config",
            "motion_config.yaml",
        ]
    )
    planning_scene_config = PathJoinSubstitution(
        [
            get_package_share_directory(pkg_name),
            "config",
            "planning_scene_config_sound_sensor.yaml",
        ]
    )

    node1 = Node(
        package=pkg_name,
        executable="grinding_demo",
        name="create_waypoints_node",
        output="screen",
        emulate_tty=True,
        parameters=[motion_config, planning_scene_config],
    )

    ld.add_action(node1)
    # 複数ノードを追加する場合は，configN,nodeNを作ってld.add_action(nodeN)?

    return ld
