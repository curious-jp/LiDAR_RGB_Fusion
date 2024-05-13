from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sick_scan_xd",
                executable="sick_generic_caller",
                name="multiScan",
                namespace="multiScan",
                output="screen",
                parameters=[
                    {"scanner_type": "sick_multiscan"},
                    {"hostname": "192.168.0.1"},
                    {"udp_receiver_ip": "192.168.0.30"},
                    {"udp_sender": ""},
                    {"udp_port": 2116},
                    {"segment_count": 12},
                    {"publish_frame_id": "base_link"},
                    {"imu_frame_id": "imu_link"},
                    {"publish_laserscan_segment_topic": "laserscan_segment"},
                    {"publish_laserscan_fullframe_topic": "laserscan_fullframe"},
                    {"udp_input_fifolength": 20},
                    {"msgpack_output_fifolength": 20},
                    {"verbose_level": 1},
                    {"measure_timing": True},
                    {"export_csv": False},
                    {"export_udp_msg": False},
                    {"logfolder": ""},
                    {"udp_timeout_ms": 60000},
                    {"scandataformat": 2},
                    {"imu_enable": True},
                    {"imu_udp_port": 7503},
                    {"imu_latency_microsec": 0},
                    {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
                    {"add_transform_check_dynamic_updates": False},
                    {"sopas_tcp_port": "2111"},
                    {"start_sopas_service": True},
                    {"send_sopas_start_stop_cmd": True},
                    {"sopas_cola_binary": False},
                    {"sopas_timeout_ms": 5000},
                    {"client_authorization_pw": "F4724744"},
                    {"host_read_filtersettings": True},
                    {"host_FREchoFilter": 2},
                    {"host_set_FREchoFilter": True},
                    {"host_LFPangleRangeFilter": "0 -180.0 +179.0 -90.0 +90.0 1"},
                    {"host_set_LFPangleRangeFilter": False},
                    {"host_LFPlayerFilter": "0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"},
                    {"host_set_LFPlayerFilter": False},
                    {"msgpack_validator_enabled": False},
                    {"msgpack_validator_verbose": 1},
                    {"msgpack_validator_discard_msgpacks_out_of_bounds": True},
                    {"msgpack_validator_check_missing_scandata_interval": 12},
                    {"msgpack_validator_required_echos": "0"},
                    {"msgpack_validator_azimuth_start": -180.0},
                    {"msgpack_validator_azimuth_end": 180.0},
                    {"msgpack_validator_elevation_start": -90.0},
                    {"msgpack_validator_elevation_end": 90.0},
                    {"msgpack_validator_valid_segments": "0 1 2 3 4 5 6 7 8 9 10 11"},
                    {
                        "msgpack_validator_layer_filter": "1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1"
                    },
                    {"ros_qos": -1},
                    {"laserscan_layer_filter": "0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0"},
                    {"custom_pointclouds": "cloud_360"},
                    {
                        "cloud_360": "coordinateNotation=3 updateMethod=0 fields=x,y,z,i,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/multiScan/cloud_360 frameid=multiscan_link publish=1"
                    },
                ],
            )
        ]
    )
