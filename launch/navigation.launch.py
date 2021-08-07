from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    robots = [0, 1, 2, 3, 4, 5]
    ld = LaunchDescription()

    for r in robots:
        ld.add_action(Node(
            package='rostron_navigation',
            executable='navigation',
            parameters=[
                    {"id": r}
            ],
            output='screen'
        ))

    return ld
