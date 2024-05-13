from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                description="Path to the V4L2 video device (e.g., /dev/video0)",
                default_value="/dev/video2",
            ),
        ]
    )
    node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        name="camera",
        parameters=[
            {"video_device": LaunchConfiguration("video_device")},
            {"image_size": [1280, 720]},
        ],
    )
    ld.add_action(node)
    return ld
