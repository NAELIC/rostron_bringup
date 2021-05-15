import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('rostron_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ns = LaunchConfiguration('team')
    config_rostron = LaunchConfiguration('config_rostron')

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'team',
            default_value='yellow',
            description='Namespace teams'),
        DeclareLaunchArgument(
            'config_rostron',
            default_value=os.path.join(
                bringup_dir, 'params', 'rostron_sim.yaml'),
            description='Configuration files for rostron'),
        DeclareLaunchArgument(
            'rviz',
            default_value='False',
            description='Display RVIZ')
    ])

    group_main = [
        PushRosNamespace(
            namespace=ns),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'simu_core.launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'config_rostron': config_rostron
            }.items()
        ),
    ]

    ld.add_action(GroupAction(group_main))

    return ld
