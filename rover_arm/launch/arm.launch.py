from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        teleop_node = Node (package="rover_arm",
                                    namespace="/rover/arm",
                                    executable="teleop",
                                    name="teleop",
                                    parameters=[{"speed_factor_crawler": 0.01},
                                                {"speed_factor_normal": 0.25},
                                                {"speed_factor_turbo": 1.0},
                                                {"smallest_radius": 0.30}]
                                                )
        
        arbitration_node = Node(package="rover_arm",
                                        namespace="/rover/arm",
                                        executable="arbitration",
                                        name="arbitration",
                                        )
        
        ld.add_action(teleop_node)
        ld.add_action(arbitration_node)

        return ld
