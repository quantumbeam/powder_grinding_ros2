from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='grinding_ft_filter',
            executable='ft_filter.py',  # ← 拡張子なし！
            name='ft_filter',
            output='screen',
            parameters=[
                {'input_topic': '/force_torque_sensor_broadcaster/wrench'},
                {'output_topic': '/wrench_filtered'},
                {'sampling_frequency': 100.0},
                {'cutoff_frequency': 2.5},
                {'filter_order': 3},
                {'data_window': 100},
                {'initial_zero': True},
                {'disable_filtering': False}
            ]
        )
    ])
