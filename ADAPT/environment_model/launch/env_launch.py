from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adapt_envmod',
            executable='envmod',
            name='env'
        ),
        Node(
            package='adapt_envmod',
            executable='visualization',
            name='map'
        )   
    ])