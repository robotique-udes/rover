import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

web_dir = os.path.join(get_package_share_directory('rover_auxiliary'), 'web')

def generate_launch_description():
    ld = LaunchDescription()

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ])
    )
    ld.add_action(rosbridge)

    node_ddb_control = Node(
        package="rover_auxiliary",
        namespace="/rover/auxiliary",
        executable="ddb_control",
        name="ddb_control"
    )
    ld.add_action(node_ddb_control)

    node_camera_manager = Node(
        package="rover_auxiliary",
        namespace="/rover/camera_manager",
        executable="camera_manager",
        name="camera_manager"
    )
    ld.add_action(node_camera_manager)

    node_emergency_stop = Node(
        package="rover_auxiliary",
        namespace="/rover/remote_estop",
        executable="remote_estop",
        name="remote_estop"
    )
    ld.add_action(node_emergency_stop)

    web_server = ExecuteProcess(
    cmd=['python3', '-m', 'http.server', '5500', '--directory', web_dir],
    output='screen'
    )
    ld.add_action(web_server)

    return ld