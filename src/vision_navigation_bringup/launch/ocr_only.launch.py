#!/usr/bin/env python3
"""
Launch file for OCR node only.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for OCR node."""
    
    ocr_pkg = get_package_share_directory('vision_ocr')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
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
    
    return LaunchDescription([
        use_sim_time_arg,
        ocr_node,
    ])
