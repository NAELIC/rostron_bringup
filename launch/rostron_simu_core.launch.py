import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('rostron_bringup')

    ns = LaunchConfiguration('namespace')
    config = LaunchConfiguration('config_rostron')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='yellow',
            description='Namespace teams'),
        DeclareLaunchArgument(
            'config_rostron',
            default_value=os.path.join(bringup_dir, 'params', 'rostron_sim.yaml'),
            description='Configuration files for rostron'),

        # Vision - Receive vision data
        Node(
            package='rostron_receiver',
            namespace=ns,
            executable='vision',
            parameters=[config],
            output='screen'
        ),

        # Filters + Localisation - Minimal filters
        # Todo(@Etienne) - rename this on agreggate_loc
        Node(
            package='rostron_filters_py',
            namespace=ns,
            executable='minimal_filter',
            name='filter',
            parameters=[config],
            output='screen'
        ),

        # Send to simulation (Orders, etc...)
        Node(
            package='rostron_sender',
            namespace=ns,
            executable='sim_sender',
            parameters=[config],
            output='screen'
        )
    ])
