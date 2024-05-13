import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    pkg_prefix = get_package_share_directory("junkyard")
    rviz_file_path = os.path.join(pkg_prefix, "launch/rviz/test.rviz")
    node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        name="rviz2",
        arguments=["-d", rviz_file_path],
    )
    ld.add_action(node)
    return ld
