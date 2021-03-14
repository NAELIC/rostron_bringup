import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('rostron_bringup'),
        'config',
        'yellow_sim.yaml'
    )

    vision = Node(
        package='rostron_receiver',
        namespace='yellow',
        executable='vision',
        parameters=[config],
        output='screen'
    )

    sender_simu = Node(
        package='rostron_sender',
        namespace='yellow',
        executable='sim_sender',
        parameters=[config],
        output='screen'
    )

    ld.add_action(vision)
    ld.add_action(sender_simu)
    # return launch.LaunchDescription([
    #     launch.actions.DeclareLaunchArgument(
    #         'node_prefix',
    #         default_value=[
    #             launch.substitutions.EnvironmentVariable('USER'), '_'],
    #         description='Prefix for node names'),
    #     launch_ros.actions.Node(
    #         package='demo_nodes_cpp', executable='talker', output='screen',
    #         name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    # ])

    return ld
