from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='adapt_transmitter',  #your package name
             
            executable='transmitter_node',   #the entry point
            name= 'transmiiter_data'           # Node name
            
        ),
    ])
