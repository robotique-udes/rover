from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    simulate_arm_arg = DeclareLaunchArgument(
        'simulate_arm',
        default_value='false',
    )
    
    simulate_arm = LaunchConfiguration('simulate_arm')

    return LaunchDescription([
        simulate_arm_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_can'), 'launch', 'can.launch.py'])])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_security'), 'launch', 'security_rover.launch.py'])])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_drive_train'), 'launch', 'drive_train.launch.py'])])),
    
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_auxiliary'), 'launch', 'auxiliary_rover.launch.py'])])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_arm'), 'launch', 'arm.launch.py'])])),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_sim'), 'launch', 'sim.launch.py'])]),
            launch_arguments={'simulate_arm': simulate_arm}.items()
        )
    ])
