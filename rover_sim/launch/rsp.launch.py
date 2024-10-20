import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_path = os.path.join(get_package_share_directory('rover_sim'))
    xacro_file = os.path.join(pkg_path, 'description', 'rover.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])

    # Explicitly set the robot_description parameter as a string
    robot_description = ParameterValue(robot_description_config, value_type=str)

    params = {'robot_description': robot_description, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher
    ])
