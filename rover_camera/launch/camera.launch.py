from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

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
        namespace="/rover/auxiliary",
        executable="panorama",
        name="panorama"
    )
    ld.add_action(node_camera_node)
    ld.add_action(node_aruco)
    ld.add_action(node_panorama)

    return ld
