from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Find the share directory of the car_description package
    car_description_share = FindPackageShare('car_description').find('car_description')

    publish_model_launch_file = car_description_share + '/launch/publish_model.launch.py'

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(publish_model_launch_file)
        ),

        Node(
            package='adapt_vi',     
            executable='gvi_node',  
            name='vi'
        ),
        Node(
            package='adapt_spotsl',
            executable='spotsl_node',
            name='spotsl'
        ),
        Node(
            package='adapt_loc',
            executable='localization',
            name='localization'
        ),
        Node(
            package='adapt_roucomp',     
            executable='route',  
            name='routemodule5'
        ),
        Node(
            package='adapt_trajp',
            executable='traj',
            name='trajectory_planner'
        ),
        Node(
            package='adapt_envmod',
            executable='env_mod',
            name='envmod'
        ),
        Node(
            package='adapt_envmod',
            executable='map',
            name='mapping'
        ),
        Node(
            package='adapt_latlongcon',
            executable='pp',
            name='path_tracking'
        ),
        Node(
            package='adapt_behplan',
            executable='behave',
            name='beh'
        ),
        Node(
            package='adapt_mobint',
            executable='minode',
            name='adaptmi'
        ),
        Node(
            package='adapt_transceiver',
            executable='transceiver_node',
            name='transceiver'
        ),
        Node(
            package='adapt_transceiver',
            executable='cpm',
            name='CPM'
        )
    ])
