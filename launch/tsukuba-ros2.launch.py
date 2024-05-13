from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [  ###################################################
            # parameters
            DeclareLaunchArgument(
                "map_file",
                default_value=EnvironmentVariable("ROS_MAP") + ".yaml",
            ),
            DeclareLaunchArgument(
                "xacro_file",
                default_value=get_package_share_directory("sick_wagen")
                + "/xacro/"
                + EnvironmentVariable("ROS_XACRO")
                + ".xacro",
            ),
            ##################################################
            # nodes
            # joy controller
            Node(
                package="sick_wagen",
                executable="joy2call.py",
                name="joy2call",
                required=True,
                output="screen",
            ),
            # map server
            Node(
                package="map_server",
                executable="map_server",
                name="map_server",
                arguments=[LaunchConfiguration("map_file")],
                output="screen",
            ),
            # xacro
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                arguments=[LaunchConfiguration("xacro_file")],
                output="screen",
                remappings=[("joint_states", "/whill/states/jointStates")],
            ),

        ]
    )
