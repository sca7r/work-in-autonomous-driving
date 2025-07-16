from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adapt_inf_trans',
            executable='inf_trans',
            name='inf_trans'
        ),
         Node(
            package='adapt_inf_spotupd',
            executable='spot_upd',
            name='inf_trans'
        ),
         Node(
            package='adapt_spot_filter',
            executable='filter_node',
            name='filter_node'
        )

    ])
