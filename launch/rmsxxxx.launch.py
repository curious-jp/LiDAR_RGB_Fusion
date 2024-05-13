import launch_ros.actions
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='rms',
            namespace='rms',
            output='screen',
            parameters=[
                {"scanner_type": "sick_rms_xxxx"},
                {"range_min": 0.0},
                {"range_max": 250.0},
                {"range_filter_handling": 0},
                {"hostname": "192.168.0.4"},
                {"cloud_topic": "cloud"},
                {"frame_id": "world"},
                {"port": "2112"},
                {"use_binary_protocol": False},
                {"timelimit": 5},
                {"load_application_default": False},
                {"tracking_mode": 0},
                {"transmit_raw_targets": False},
                {"transmit_objects": True},
                {"min_intensity": 0.0},
                {"use_generation_timestamp": True},
                {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
                {"add_transform_check_dynamic_updates": False},
                {"start_services": True},
                {"message_monitoring_enabled": False},
                {"read_timeout_millisec_default": 5000},
                {"read_timeout_millisec_startup": 120000},
                {"read_timeout_millisec_kill_node": 150000},
                {"client_authorization_pw": "F4724744"},
                {"ros_qos": -1}
            ],
        ),
    ])
