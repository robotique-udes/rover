import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    gui_node = Node(package="rover_gui",
                    namespace="base/gui",
                    executable="main_gui",
                    name="main_gui")

    ld.add_action(gui_node)

    return LaunchDescription([gui_node])
