from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    with_sik_radio_arg = DeclareLaunchArgument(
        'with_sik_radio',
        default_value='true',
        description='Whether to launch 900MHz slow-scan emergency camera node'
    )

    node_camera_node = Node(
        package= "rover_camera",
        namespace = "/rover/camera",
        executable = "media_server",
        name = "media_server",
    )

    node_aruco = Node(
        package="rover_camera",
        namespace="/rover/camera",
        executable="aruco_detection",
        name="aruco_detection"
    )
    
    node_panorama = Node(
        package="rover_camera",
        namespace="/rover/camera",
        executable="panorama",
        name="panorama"
    )

    node_emergency = Node(
        package="rover_camera",
        namespace="/rover/camera",
        executable="emergency_camera",
        name="emergency_camera",
        parameters=[{
            "camera_url": "rtsp://192.168.144.30:554/1/h264major",
            "interval_ms": 1500,
            "target_width": 320,
            "target_height": 240,
            "jpeg_quality": 35
        }],
        condition=IfCondition(LaunchConfiguration('with_sik_radio'))
    )

    return LaunchDescription([
        with_sik_radio_arg,
        node_camera_node,
        node_aruco,
        node_panorama,
        node_emergency
    ])
