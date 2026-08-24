from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'uart_port', default_value='/dev/ttyS1',
            description='UART port for STM32 communication'),
        DeclareLaunchArgument(
            'uart_baudrate', default_value='115200',
            description='UART baud rate'),
        DeclareLaunchArgument(
            'control_freq', default_value='100.0',
            description='RL control frequency (Hz)'),
        DeclareLaunchArgument(
            'max_torque', default_value='10.0',
            description='Maximum torque (Nm) for safety limit'),

        Node(
            package='rl_policy',
            executable='rl_policy_node',
            name='rl_policy_node',
            output='screen',
            parameters=[{
                'uart_port': LaunchConfiguration('uart_port'),
                'uart_baudrate': 115200,
                'control_freq': 100.0,
                'max_torque': 10.0,
            }],
        ),
    ])
