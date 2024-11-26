from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_uav',
            executable='mission_controller',
            namespace='auto_uav',
            name='mission_controller',
            output='screen'
        ),
        Node(
            package='auto_uav',
            executable='navigator',
            namespace='auto_uav',
            name='navigator',
            output='screen'
        ),
        Node(
            package='auto_uav',
            executable='lidar_bridge',
            namespace='auto_uav',
            name='lidar_bridge',
            output='screen'
        ),
        Node(
            package='auto_uav',
            executable='thermal_camera_bridge',
            namespace='auto_uav',
            name='thermal_camera_bridge',
            output='screen'
        ),
    ])