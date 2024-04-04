from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Micro ROS Agent
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ESP32'],
            output='screen'
        ),

        # Robot RQT Steering GUI (assuming the executable name, replace if different)
        Node(
            package='robot_rqt_steering',
            executable='rqt_steering_gui',
            output='screen'
        ),

        # Hoverboard Bridge Node
        Node(
            package='hoverboard_bridge',
            executable='bridge_node',
            output='screen'
        )
    ])
