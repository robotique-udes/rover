from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
        ld: LaunchDescription = LaunchDescription()

        test_camera_node = Node (package="rover_odometry",
                                    namespace="/rover/odometry",
                                    executable="test_camera",
                                    name="test_camera"
                                    )
        
        ld.add_action(test_camera_node)

        return ld
