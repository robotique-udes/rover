import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    urdf_file = "robot.urdf"
    pkg_desc = "rover_sim"

    robot_desc_path = os.path.join(get_package_share_directory(pkg_desc), "rover_urdf", urdf_file)

    with open(robot_desc_path, 'r') as file:
        robot_desc_path = file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publiser_node',
        emulate_tty=True,
        parameters=[{'use_time_sim': True, 'robot_description': robot_desc_path}],
        output="screen"
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
        ]
    )