import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('rostron_bringup')
    # TODO : @Etienne(Make parameters)
    map_file = os.path.join(bringup_dir, 'maps', 'division_b.yaml')

    ns = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('conf')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    robot_id = LaunchConfiguration('robot_id')
    team = LaunchConfiguration('team')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'map_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        # 'default_nav_to_pose_bt_xml': default_bt_xml_filename,
        # 'default_nav_through_poses_bt_xml': default_bt_xml_filename,
        'robot_id': robot_id,
        'team': team
    }

    conf = RewrittenYaml(
        source_file=params_file,
        root_key=ns,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='yellow/robot_1',
            description='Namespace robot in teams'),
        DeclareLaunchArgument(
            'conf',
            default_value=os.path.join(
                bringup_dir, 'params', 'nav.yaml'),
            description='Namespace robot in teams'),
        DeclareLaunchArgument(
            'default_bt_xml_filename',
            default_value=os.path.join(
                bringup_dir,
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),
        DeclareLaunchArgument(
            'robot_id',
            default_value='0',
            description='ID Robot'),
        DeclareLaunchArgument(
            'team',
            default_value='yellow',
            description='ID Robot'),
        # Navigation - Start

        # Fake TF Transform odom <-> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            remappings=remappings
        ),

        # Navigation
        Node(
            package='rostron_nav',
            executable='localisation',
            name='localisation',
            output='screen',
            remappings=remappings,
            parameters=[{'robot_id': robot_id},
                        {'team': team}
                        ]
        ),


        Node(
            package='rostron_nav',
            executable='nav2order',
            remappings=remappings,
            parameters=[{'robot_id': robot_id},
                        {'team': team}
                        ]
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            remappings=remappings,
            parameters=[conf]),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            remappings=remappings,
            parameters=[conf]),

        # Recoveries
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            remappings=remappings,
            output='screen'),

        # To see
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            remappings=remappings,
            parameters=[conf]
        ),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            remappings=remappings,
            parameters=[{'use_sim_time': False},
                        {'yaml_filename': map_file}
                        ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            remappings=remappings,
            parameters=[{'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])
