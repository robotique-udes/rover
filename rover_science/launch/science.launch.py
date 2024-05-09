import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        teleop_node = Node (package="rover_science",
                                    namespace="/rover/science",
                                    executable="rover_science",
                                    name="rover_science",
                                        )
        
        ld.add_action(teleop_node)

        return ld
