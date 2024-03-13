import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        node_driver_teleop = Node (package="rover_driver_train",
                                    namespace="/teleop",
                                    executable="driver_train_msg",
                                    name="driver_train",
                                    parameters=[{"speed_factor_crawler": 0.01},
                                                {"speed_factor_normal": 0.25},
                                                {"speed_factor_turbo": 1.0},
                                                {"smalllest_radius": 0.30}],
                                    remappings=[("driver", "raw")])
        
        node_joy_formatted = LaunchDescription(
                                PythonLaunchDescriptionSource(
                                        os.path.join(
                                                get_package_share_directory("rover_joy_cpp"),
                                                "launch/topics/joy.launch.py"))
                                        )

        
        ld.add_action(node_driver_teleop)
        ld.add_action(node_joy_formatted)


        return ld
