from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='wrench_filter_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='grinding_force_torque',
                    plugin='grinding_force_torque::WrenchFilter',
                    name='wrench_filter',
                    parameters=[
                        {'input_topic': '/force_torque_sensor_broadcaster/wrench'},
                        {'output_topic': '/wrench_filtered'},
                        {'data_window': 100},
                        {'initial_zero': True},
                        {'disable_filtering': False},
                        {'filter_b': [0.0001, 0.0003, 0.0003, 0.0001]},
                        {'filter_a': [1.0, -2.847, 2.706, -0.858]}
                    ]
                )
            ],
            output='screen',
        )
    ])