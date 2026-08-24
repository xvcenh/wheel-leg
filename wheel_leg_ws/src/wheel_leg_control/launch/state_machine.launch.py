import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_leg_control',
            executable='state_machine_node',
            name='state_machine',
            output='screen',
            parameters=[{
                'base_speed': 0.15,
                'max_speed': 0.3,
                'kp_lateral': 3.0,
                'kd_lateral': 0.5,
                'max_home_speed': 0.2,
                'kp_distance': 0.8,
                'kp_yaw': 1.5,
                'target_tolerance': 0.05,
                'yaw_tolerance': 0.05,
                'stop_delay': 5.0,
            }],
        ),
    ])
