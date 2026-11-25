from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('rover_rviz')

    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_gui = LaunchConfiguration('use_gui')

    robot_description = ParameterValue(Command(['cat ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'urdf_file',
            default_value=PathJoinSubstitution([pkg_share, 'urdf', 'arm_v2.urdf'])),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([pkg_share, 'rviz', 'arm.rviz'])),
        DeclareLaunchArgument('use_gui', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(use_gui)),

        Node(
            package='rover_rviz',
            executable='arm_to_joint_state',
            output='screen',
            condition=UnlessCondition(use_gui)),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]),
    ])