#!/usr/bin/env python3
"""
Launch file for VLM node only.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for VLM node."""
    
    vlm_pkg = get_package_share_directory('vision_vlm')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
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
    
    return LaunchDescription([
        use_sim_time_arg,
        vlm_node,
    ])
