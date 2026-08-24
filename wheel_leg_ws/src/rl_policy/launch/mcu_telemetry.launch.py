from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'transport', default_value='can',
            description='Transport: can (CAN3 FD main channel) or uart'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='CAN interface name (SocketCAN)'),
        DeclareLaunchArgument(
            'uart_port', default_value='/dev/ttyS1',
            description='UART port for STM32 communication (transport=uart)'),
        DeclareLaunchArgument(
            'publish_freq', default_value='100.0',
            description='Telemetry publish frequency (Hz)'),

        Node(
            package='rl_policy',
            executable='mcu_telemetry_node',
            name='mcu_telemetry_node',
            output='screen',
            parameters=[{
                'transport': LaunchConfiguration('transport'),
                'can_interface': LaunchConfiguration('can_interface'),
                'uart_port': LaunchConfiguration('uart_port'),
                'uart_baudrate': 921600,
                'publish_freq': LaunchConfiguration('publish_freq'),
            }],
        ),
    ])
