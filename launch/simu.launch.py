import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('rostron_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ns = LaunchConfiguration('team')
    config_rostron = LaunchConfiguration('config_rostron')
    use_rviz = LaunchConfiguration('rviz')

    robots = [0]
    # robots = [0, 1, 2, 3, 4, 5]

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
                os.path.join(launch_dir, 'rostron_simu_core.launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'config_rostron': config_rostron
            }.items()
        ),
    ]

    for robot in robots:
        ns_robot = ['robot_', str(robot)]
        group = GroupAction([
            PushRosNamespace(
                namespace=ns_robot),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    # See textsubstitution
                    'namespace': [ns, '/robot_', str(robot)],
                    # 'rviz_config': rviz_config_file
                }.items()),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rostron_nav.launch.py')
                ),
                launch_arguments={
                    'namespace': [ns, '/robot_', str(robot)],
                    'team': ns,
                    'robot_id': str(robot)
                }.items())
        ])

        group_main.append(group)

    ld.add_action(GroupAction(group_main))

    return ld
