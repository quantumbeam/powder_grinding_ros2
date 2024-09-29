
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, AndSubstitution, NotSubstitution
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type', default='ur5e')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.58.42')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='false')
    use_moveit = LaunchConfiguration('use_moveit', default='true')
    launch_rviz_with_moveit = LaunchConfiguration('launch_rviz_with_moveit', default='true')

    ur_robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_robot_driver'), '/launch','/ur_control.launch.py']),
        launch_arguments={'ur_type': ur_type,
                          'robot_ip': robot_ip,
                          'use_fake_hardware': use_fake_hardware,
                          'launch_rviz': launch_rviz}.items()
    )



    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ur_moveit_config'), '/launch','/ur_moveit.launch.py']),
        launch_arguments={'ur_type': ur_type,
                        'launch_rviz':launch_rviz_with_moveit}.items(),
        condition=IfCondition(use_moveit)
    )

    return LaunchDescription([
        ur_robot_driver_launch,
        ur_moveit_launch
    ])
