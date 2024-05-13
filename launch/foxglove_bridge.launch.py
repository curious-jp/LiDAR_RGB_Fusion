from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        name="foxglove_bridge",
    )
    ld.add_action(node)
    return ld
