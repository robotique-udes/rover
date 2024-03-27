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

        joy_launch_file = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(
                                get_package_share_directory('rover_joy'),
                                'launch/joy.launch.py'))
        )

        teleop_node = Node (package="rover_drive_train",
                                    namespace="/rover/drive_drain",
                                    executable="teleop",
                                    name="teleop",
                                    parameters=[{"speed_factor_crawler": 0.01},
                                                {"speed_factor_normal": 0.25},
                                                {"speed_factor_turbo": 1.0},
                                                {"smallest_radius": 0.30}]
                                                )
        
        arbitration_node = Node(package="rover_drive_train",
                                        namespace="/rover/drive_dtrain",
                                        executable="arbitration",
                                        name="arbitration",
                                        )
        
        ld.add_action(joy_launch_file)
        ld.add_action(teleop_node)
        ld.add_action(arbitration_node)

        return ld
