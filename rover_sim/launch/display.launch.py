from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch argument
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='',
        description='Model argument'
    )

    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('rover_sim'),
        'urdf',
        'rover.urdf'
    )

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # Set up the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_desc, value_type=str)
        }]
    )

    # Set up the joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # Set up RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('rover_sim'),
        'rviz',
        'urdf_config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )

    # Create and return the launch description
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])