from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autovech',
            executable='throttle_executable',
            output='screen'),
        Node(
            package='autovech',
            executable='brakes_executable',
            output='screen'),
        Node(
            package='autovech',
            executable='gamepad_executable',
            output='screen'),
    ])