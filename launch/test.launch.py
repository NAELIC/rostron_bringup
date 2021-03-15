import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config_dir = os.path.join(
        get_package_share_directory('rostron_bringup'),
        'config',
        'nav2_default_view.rviz')

    map_file = os.path.join(get_package_share_directory(
        'rostron_bringup'), 'config', 'map.yaml')
    nav2_yaml = os.path.join(get_package_share_directory(
        'rostron_bringup'), 'config', 'nav2_params.yaml')
    default_bt_xml_path = os.path.join(get_package_share_directory(
        'rostron_bringup'), 'config', 'navigate_w_replanning_and_recovery.xml')

    return LaunchDescription([
        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_yaml]),
        # Planner
        Node(
            package='nav2_planner',
            node_executable='planner_server',
            node_name='planner_server',
            output='screen',
            parameters=[nav2_yaml]),

        # Recoveries
        Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            node_name='recoveries_server',
            output='screen'),

        # To see
        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'bt_xml_filename': default_bt_xml_path}]),


        Node(
            package='nav2_map_server',
            node_executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}
                        ]),

        Node(
            package='rviz2',
            node_executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'map_server']}])
    ])
