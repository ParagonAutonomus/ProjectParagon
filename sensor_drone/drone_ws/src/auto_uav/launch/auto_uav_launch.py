#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the mission_controller node
        Node(
            package='auto_uav',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ),
        
        # Launch the navigator node
        Node(
            package='auto_uav',
            executable='navigator',
            name='navigator',
            output='screen'
        ),
    ])