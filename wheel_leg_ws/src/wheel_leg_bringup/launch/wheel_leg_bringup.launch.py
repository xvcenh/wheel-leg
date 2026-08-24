"""
Master bringup launch file for the wheel-legged robot system.

Launches:
  1. mipi_cam        - GS130W stereo camera (dual channel)
  2. joy_node        - Bluetooth gamepad input
  3. mcu_telemetry_node - CAN3 (CAN FD) bridge to STM32: IMU/joint/voltage/temp telemetry + cmd
  4. wheel_leg_vision - Line detector + box detector
  5. wheel_leg_control - State machine controller

Usage:
  ros2 launch wheel_leg_bringup wheel_leg_bringup.launch.py
  ros2 launch wheel_leg_bringup wheel_leg_bringup.launch.py color:=yellow
  ros2 launch wheel_leg_bringup wheel_leg_bringup.launch.py can_interface:=can0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ============================================================
    # Launch Arguments
    # ============================================================
    color_arg = DeclareLaunchArgument(
        'color', default_value='red',
        description='Tape color: red | yellow | yellow_black')

    can_if_arg = DeclareLaunchArgument(
        'can_interface', default_value='can0',
        description='CAN interface name (SocketCAN, FD mode)')

    image_width_arg = DeclareLaunchArgument(
        'image_width', default_value='1280',
        description='MIPI camera image width')

    image_height_arg = DeclareLaunchArgument(
        'image_height', default_value='1088',
        description='MIPI camera image height')

    # ============================================================
    # 1. GS130W Stereo Camera (mipi_cam dual channel)
    # ============================================================
    mipi_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('mipi_cam'),
                'launch',
                'mipi_cam_dual_channel.launch.py'
            )
        ]),
        launch_arguments={
            'mipi_image_width': LaunchConfiguration('image_width'),
            'mipi_image_height': LaunchConfiguration('image_height'),
        }.items(),
    )

    # ============================================================
    # 2. Joy Node (Bluetooth gamepad)
    # ============================================================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'autorepeat_rate': 0.0,
            'deadzone': 0.05,
        }],
    )

    # ============================================================
    # 3. MCU Telemetry Node (CAN3 FD <-> STM32, 主通信通道)
    # ============================================================
    mcu_telemetry_node = Node(
        package='rl_policy',
        executable='mcu_telemetry_node',
        name='mcu_telemetry_node',
        output='screen',
        parameters=[{
            'transport': 'can',
            'can_interface': LaunchConfiguration('can_interface'),
            'publish_freq': 100.0,
        }],
    )

    # ============================================================
    # 4. Vision Nodes (delayed to ensure camera is ready)
    # ============================================================
    line_detector_node = Node(
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
    )

    box_detector_node = Node(
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
    )

    # Delay vision nodes by 2s to let camera stabilize
    vision_group = TimerAction(
        period=2.0,
        actions=[line_detector_node, box_detector_node],
    )

    # ============================================================
    # 5. State Machine Controller
    # ============================================================
    state_machine_node = Node(
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
    )

    # ============================================================
    # Assemble Launch Description
    # ============================================================
    return LaunchDescription([
        color_arg,
        can_if_arg,
        image_width_arg,
        image_height_arg,
        mipi_cam_launch,
        joy_node,
        mcu_telemetry_node,
        vision_group,
        state_machine_node,
    ])
