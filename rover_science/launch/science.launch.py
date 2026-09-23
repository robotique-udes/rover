from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        science_node = Node(package="rover_science",
                                        namespace="/rover/science",
                                        executable="teleop",
                                        name="teleop",
                                        )
        
        ld.add_action(science_node)

        return ld