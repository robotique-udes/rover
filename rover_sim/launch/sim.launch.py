from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Explore possibilities other than using the following:
# export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/emile/ros2_ws/src (in bashrc)
# If used, make sure to source ws

# Also check for way to spawn rover at different pose in gz

def generate_launch_description():
    # Set the path to your URDF file
    urdf_file_name = 'new_rover.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('rover_sim'),
        'model',
        urdf_file_name
    )

    # Set the path to your RViz config file
    rviz_config_file = os.path.join(
        get_package_share_directory('rover_sim'),
        'config',
        'rover.rviz'
    )

    # Declare the LaunchDescription
    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Launch Ignition Gazebo with an empty world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--verbose', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the URDF robot model using Ignition service
        ExecuteProcess(
            cmd=[
                'ign', 'service', '-s', '/world/empty/create',
                '--reqtype', 'ignition.msgs.EntityFactory',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf_filename: "{urdf_path}", name: "urdf_model"'
            ],
            output='screen'
        ),

        # Joint State Publisher to publish the joint states (needed for RViz)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Robot State Publisher to publish the URDF to the /robot_description topic (needed for RViz)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': open(urdf_path).read()
            }]
        ),

        # Launch RViz with the robot model and specified config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])
