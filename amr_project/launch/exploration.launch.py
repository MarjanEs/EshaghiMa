from launch import (
    LaunchDescription
)

from launch_ros.actions import (
    Node
)


def generate_launch_description():

    exploration_node = Node(
        package='amr_project',
        executable='exploration_node',
        name='exploration_node',
        output='screen'
    )

    return LaunchDescription([
        exploration_node
    ])