#!/usr/bin/env python3
"""
Launch file for complete vision-guided navigation system.

This launches all three nodes: OCR, VLM, and Control.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for navigation system."""
    
    # Get package directories
    ocr_pkg = get_package_share_directory('vision_ocr')
    vlm_pkg = get_package_share_directory('vision_vlm')
    control_pkg = get_package_share_directory('vision_control')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # OCR Node
    ocr_node = Node(
        package='vision_ocr',
        executable='ocr_node',
        name='ocr_node',
        output='screen',
        parameters=[
            os.path.join(ocr_pkg, 'config', 'ocr_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # VLM Node
    vlm_node = Node(
        package='vision_vlm',
        executable='vlm_node',
        name='vlm_node',
        output='screen',
        parameters=[
            os.path.join(vlm_pkg, 'config', 'vlm_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Control Node
    control_node = Node(
        package='vision_control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[
            os.path.join(control_pkg, 'config', 'control_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        ocr_node,
        vlm_node,
        control_node,
    ])
