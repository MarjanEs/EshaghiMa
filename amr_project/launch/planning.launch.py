import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    package_share = (
        get_package_share_directory(
            'amr_project'
        )
    )

    config_file = os.path.join(
        package_share,
        'config',
        'planner.yaml'
    )

    planner_node = Node(
        package='amr_project',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        planner_node
    ])
