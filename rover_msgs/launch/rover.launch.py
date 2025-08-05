from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    with_sim_arg = DeclareLaunchArgument(
        'with_sim',
        default_value='false',
    )
    
    with_sim = LaunchConfiguration('with_sim')

    return LaunchDescription([
        with_sim_arg,

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([FindPackageShare('rover_can'), 'launch', 'can.launch.py'])
        #     ])
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_security'), 'launch', 'security_rover.launch.py'])
            ])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_drive_train'), 'launch', 'drive_train.launch.py'])
            ])
        ),
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_auxiliary'), 'launch', 'auxiliary_rover.launch.py'])
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_arm'), 'launch', 'arm.launch.py'])
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_sim'), 'launch', 'sim.launch.py'])
            ]),
            condition=IfCondition(with_sim)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('rover_camera'), 'launch', 'camera.launch.py'])
            ])
        )
    ])
