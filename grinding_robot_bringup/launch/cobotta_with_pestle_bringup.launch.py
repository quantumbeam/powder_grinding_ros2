# cobotta_with_pestle_bringup.launch.py
#
# ROS2 port of the ROS1 `grinding_robot_bringup/cobotta_bringup.launch`.
# Starts ros2_control (DENSO b-CAP slave driver or mock hardware), the controllers,
# robot_state_publisher and RViz for COBOTTA with pestle and spatula.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    # --- Declare Launch Arguments ---
    declared_arguments.append(DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.11',
        description='IP address of the COBOTTA controller.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware interface for testing without a physical robot.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'robot_name',
        default_value='CVR038A1-NV6-NN',
        description='Robot type name. Must be consistent with @TYPE_NAME of the real robot.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'send_format',
        default_value='0',
        description='b-CAP slave send format (0: no I/O, recommended for COBOTTA).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'recv_format',
        default_value='2',
        description='b-CAP slave receive format (2: no I/O, recommended for COBOTTA).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'bcap_slave_control_cycle_msec',
        default_value='8.0',
        description='DENSO robot control cycle [ms].'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'denso_config_file',
        # config_novar.xml does not register the global variables (I*, F*, P*, J*, ...) and tasks
        # on the controller, which makes the connection much faster. The variables required for
        # the slave mode (@ERROR_CODE, @SERVO_ON, @TYPE_NAME) are added by the driver itself.
        # Use config.xml if you need to access the controller variables via ROS.
        default_value=PathJoinSubstitution([FindPackageShare('denso_robot_core'), 'config', 'config_novar.xml']),
        description='denso_robot_core variable configuration file (absolute path).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Print parsed DENSO driver arguments.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'initial_joint_controller',
        default_value='arm_controller',
        description='Initial robot controller.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'description_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_robot_description'), 'urdf', 'cobotta', 'cobotta_with_pestle.urdf.xacro'
        ]),
        description='URDF/XACRO description file (absolute path) with the robot.'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_robot_bringup'), 'config', 'cobotta_controllers.yaml'
        ]),
        description='ros2_control controllers configuration file (absolute path).'
    ))
    declared_arguments.append(DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('grinding_robot_description'), 'rviz', 'view_robot.rviz'
        ]),
        description='RViz config file (absolute path) to use when launching rviz.'
    ))

    # --- Initialize Arguments ---
    robot_ip = LaunchConfiguration('robot_ip')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    robot_name = LaunchConfiguration('robot_name')
    send_format = LaunchConfiguration('send_format')
    recv_format = LaunchConfiguration('recv_format')
    bcap_slave_control_cycle_msec = LaunchConfiguration('bcap_slave_control_cycle_msec')
    verbose = LaunchConfiguration('verbose')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')
    launch_rviz = LaunchConfiguration('launch_rviz')
    description_file = LaunchConfiguration('description_file')
    controllers_file = LaunchConfiguration('controllers_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', description_file,
        ' ', 'use_mock_hardware:=', use_mock_hardware,
        ' ', 'ip_address:=', robot_ip,
        ' ', 'robot_name:=', robot_name,
        ' ', 'send_format:=', send_format,
        ' ', 'recv_format:=', recv_format,
        ' ', 'verbose:=', verbose,
    ])
    robot_description = {
        'robot_description': ParameterValue(value=robot_description_content, value_type=str)
    }

    # Parameters read by denso_robot_core (ignored with mock hardware)
    denso_params = {
        'denso_bcap_slave_control_cycle_msec': bcap_slave_control_cycle_msec,
        'denso_config_file': LaunchConfiguration('denso_config_file'),
    }

    # --- Nodes ---
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file, denso_params],
        output='screen',
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[initial_joint_controller, '--controller-manager', '/controller_manager'],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
