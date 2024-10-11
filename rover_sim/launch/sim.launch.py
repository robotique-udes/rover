from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set the path to your URDF file and RViz config file
    urdf_file_name = 'new_rover.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('rover_sim'), 
        'model', 
        urdf_file_name
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('rover_sim'), 
        'config', 
        'rover.rviz'
    )

    # Load the URDF file to the robot_description parameter
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # LaunchDescription to launch RViz with the robot model
    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Joint State Publisher to publish the joint states
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Robot State Publisher to publish the URDF to the /robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_description_content
            }]
        ),

        # Launch RViz with the robot model and specified config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
