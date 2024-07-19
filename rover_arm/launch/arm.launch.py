from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        teleop_node = Node (package="rover_arm",
                                    namespace="/rover/arm",
                                    executable="teleop",
                                    name="teleop"
                                    )
        
        ld.add_action(teleop_node)

        return ld
