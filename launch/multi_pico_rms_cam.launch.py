from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    LAPTOP_IP = "192.168.0.142"
    MULTISCAN_IP = "192.168.0.2"
    PICOSCAN_IP = "192.168.0.3"
    RMS_IP = "192.168.0.4"

    multiscan_node = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="multiScan",
        namespace="multiScan",
        output="screen",
        parameters=[
            {"scanner_type": "sick_multiscan"},
            {"hostname": MULTISCAN_IP},
            {"udp_receiver_ip": LAPTOP_IP},
            {"udp_sender": ""},
            {"udp_port": 2115},
            {"segment_count": 12},
            {"publish_frame_id": "world"},
            {"publish_laserscan_segment_topic": "laserscan_segment"},
            {"publish_laserscan_fullframe_topic": "laserscan_fullframe"},
            {"udp_input_fifolength": 20},
            {"msgpack_output_fifolength": 20},
            {"verbose_level": 0},
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
            {"ros_qos": -1},
            {"laserscan_layer_filter": "0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0"},
            {"custom_pointclouds": "cloud_360"},
            {
                "cloud_360": "coordinateNotation=3 updateMethod=0 fields=x,y,z,i,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/multiScan/cloud_360 frameid=world publish=1"
            },
        ],
    )

    picoscan_node = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="picoScan",
        namespace="picoScan",
        output="screen",
        parameters=[
            {"scanner_type": "sick_picoscan"},
            {"hostname": PICOSCAN_IP},
            {"udp_receiver_ip": LAPTOP_IP},
            {"port": "2118"},
            {"udp_port": 2117},
            {"all_segments_min_deg": -138.0},
            {"all_segments_max_deg": 138.0},
            {"publish_frame_id": "world"},
            {"publish_laserscan_segment_topic": "laserscan_segment"},
            {"publish_laserscan_fullframe_topic": "laserscan_fullframe"},
            {"udp_input_fifolength": 20},
            {"msgpack_output_fifolength": 20},
            {"verbose_level": 0},
            {"measure_timing": True},
            {"export_csv": False},
            {"export_udp_msg": False},
            {"logfolder": ""},
            {"send_udp_start": False},
            {"send_udp_start_string": "magicalActivate"},
            {"udp_timeout_ms": 60000},
            {"scandataformat": 2},
            {"imu_enable": False},
            {"imu_udp_port": 7504},
            {"imu_latency_microsec": 0},
            {"add_transform_xyz_rpy": "0,0,0,0,0,0"},
            {"sopas_tcp_port": "2111"},
            {"start_sopas_service": True},
            {"send_sopas_start_stop_cmd": True},
            {"sopas_cola_binary": False},
            {"sopas_timeout_ms": 5000},
            {"client_authorization_pw": "F4724744"},
            {"host_read_filtersettings": True},
            {"host_FREchoFilter": 2},
            {"host_set_FREchoFilter": True},
            {"msgpack_validator_enabled": False},
            {"laserscan_layer_filter": "1"},
            {"custom_pointclouds": "cloud_360"},
            {
                "cloud_360": "coordinateNotation=3 updateMethod=0 fields=x,y,z,i,echo,reflector echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0,999,0 topic=/picoScan/cloud_360 frameid=world publish=1"
            },
        ],
    )

    rms_node = Node(
        package="sick_scan_xd",
        executable="sick_generic_caller",
        name="rms",
        namespace="rms",
        output="screen",
        parameters=[
            {"scanner_type": "sick_rms_xxxx"},
            {"range_min": 0.0},
            {"range_max": 250.0},
            {"range_filter_handling": 0},
            {"hostname": RMS_IP},
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
            {"ros_qos": -1},
        ],
    )

    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        output="screen",
        name="v4l2_camera_node",
        parameters=[{"video_device": "/dev/video0"}, {"image_size": [1280, 720]}],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "radar"],
    )

    ld.add_action(multiscan_node)
    ld.add_action(picoscan_node)
    ld.add_action(rms_node)
    ld.add_action(static_tf_node)
    ld.add_action(camera_node)
    return ld
