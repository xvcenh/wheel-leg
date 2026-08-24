import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyS1',
            description='UART serial port for STM32 communication'),
        DeclareLaunchArgument(
            'baud_rate', default_value='115200',
            description='UART baud rate'),

        Node(
            package='wheel_leg_driver',
            executable='wheel_leg_driver_node',
            name='wheel_leg_driver',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'wheel_radius': 0.0603,
                'track_width': 0.30,
            }],
        ),
    ])
