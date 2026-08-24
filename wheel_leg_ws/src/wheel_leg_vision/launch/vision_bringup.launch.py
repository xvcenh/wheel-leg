import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('color', default_value='red',
            description='Tape color: red | yellow | yellow_black'),

        # Line detector node
        Node(
            package='wheel_leg_vision',
            executable='line_detector_node',
            name='line_detector',
            output='screen',
            parameters=[{
                'color': LaunchConfiguration('color'),
                'hsv_lower': [0, 100, 100],
                'hsv_upper': [10, 255, 255],
                'roi_top': 0.4,
                'roi_bottom': 1.0,
                'min_contour_area': 200,
                'publish_debug_img': True,
            }],
        ),

        # Box detector node
        Node(
            package='wheel_leg_vision',
            executable='box_detector_node',
            name='box_detector',
            output='screen',
            parameters=[{
                'color': LaunchConfiguration('color'),
                'box_hsv_lower': [0, 100, 100],
                'box_hsv_upper': [10, 255, 255],
                'min_box_area': 500,
                'ema_alpha': 0.7,
                'publish_debug_img': True,
            }],
        ),
    ])
