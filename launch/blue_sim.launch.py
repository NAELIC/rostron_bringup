import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

namespace='blue'

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rostron_bringup'),
        'config',
        'blue_sim.yaml'
    )

    vision = Node(
        package='rostron_receiver',
        namespace=namespace,
        executable='vision',
        parameters=[config],
        output='screen'
    )

    sender_simu = Node(
        package='rostron_sender',
        namespace=namespace,
        executable='sim_sender',
        parameters=[config],
        output='screen'
    )

    minimal_filter = Node(
        package='rostron_filters_py',
        namespace=namespace,
        executable='minimal_filter',
        parameters=[config],
        output='screen'
    )

    ld.add_action(vision)
    ld.add_action(sender_simu)
    ld.add_action(minimal_filter)

    return ld
