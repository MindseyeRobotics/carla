#!/usr/bin/env python3

"""
ROS2 launch file for CARLA-PX4 integration bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for CARLA-PX4 bridge."""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'carla_host',
            default_value='localhost',
            description='CARLA server host address'
        ),
        
        DeclareLaunchArgument(
            'carla_port',
            default_value='2000',
            description='CARLA server port'
        ),
        
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='drone',
            description='Vehicle ID for ROS2 topic namespacing'
        ),
        
        LogInfo(msg=['Starting CARLA-PX4 Bridge...']),
        
        # CARLA-PX4 Bridge Node
        Node(
            package='carla_px4_bridge',
            executable='px4_bridge_node',
            name='carla_px4_bridge',
            output='screen',
            parameters=[{
                'carla_host': LaunchConfiguration('carla_host'),
                'carla_port': LaunchConfiguration('carla_port'),
                'vehicle_id': LaunchConfiguration('vehicle_id'),
            }],
        ),
    ])
