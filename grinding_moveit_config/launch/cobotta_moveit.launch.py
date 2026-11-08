# cobotta_moveit.launch.py
#
# MoveIt (move_group + RViz MotionPlanning) for COBOTTA with pestle and spatula.
# Start it after grinding_robot_bringup/cobotta_with_pestle_bringup.launch.py
# (ros2_control, arm_controller and robot_state_publisher are started there),
# or let the bringup include it with launch_moveit:=true.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from grinding_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    description_file = LaunchConfiguration('description_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    load_planning_scene = LaunchConfiguration('load_planning_scene')
    planning_scene_config = LaunchConfiguration('planning_scene_config')
    rviz_config_file = LaunchConfiguration('moveit_rviz_config_file')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')

    # The ros2_control part of the URDF is not used by MoveIt, so the hardware arguments do not matter.
    robot_description = {
        'robot_description': ParameterValue(
            Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', description_file]),
            value_type=str,
        )
    }

    srdf_path = os.path.join(get_package_share_directory('grinding_moveit_config'), 'srdf', 'cobotta.srdf')
    with open(srdf_path) as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('grinding_moveit_config'), 'config', 'cobotta_kinematics.yaml']
    )
    robot_description_planning = {
        'robot_description_planning': load_yaml('grinding_moveit_config', 'config/cobotta_joint_limits.yaml')
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(
        load_yaml('grinding_moveit_config', 'config/ompl_planning.yaml')
    )

    moveit_controllers = {
        'moveit_simple_controller_manager': load_yaml('grinding_moveit_config', 'config/cobotta_controllers.yaml'),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            {'publish_robot_description_semantic': True},
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': use_sim_time},
            warehouse_ros_config,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    # Publish the table and the mortar to /planning_scene after move_group is up.
    planning_scene_loader = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='grinding_scene_description',
                executable='load_planning_scene_node',
                name='planning_scene_loader',
                parameters=[planning_scene_config],
                output='screen',
            )
        ],
        condition=IfCondition(load_planning_scene),
    )

    return [move_group_node, rviz_node, planning_scene_loader]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'description_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('grinding_robot_description'), 'urdf', 'cobotta', 'cobotta_with_pestle.urdf.xacro'
            ]),
            description='URDF/XACRO description file (absolute path) with the robot.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time.'),
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz with MotionPlanning?'),
        DeclareLaunchArgument(
            'load_planning_scene', default_value='true', description='Add the table and the mortar to the planning scene?'
        ),
        DeclareLaunchArgument(
            'planning_scene_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('grinding_scene_description'), 'config', 'cobotta_planning_scene_config.yaml'
            ]),
            description='Planning scene configuration file (absolute path).',
        ),
        DeclareLaunchArgument(
            'moveit_rviz_config_file',
            default_value=PathJoinSubstitution([FindPackageShare('grinding_moveit_config'), 'rviz', 'cobotta_moveit.rviz']),
            description='RViz config file (absolute path).',
        ),
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored.',
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
