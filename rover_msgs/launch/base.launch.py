from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    with_sik_radio_arg = DeclareLaunchArgument(
        'with_sik_radio',
        default_value='true',
        description='Whether to launch SiK 900MHz telemetry radio node'
    )
    serial_port_arg = DeclareLaunchArgument(
        'sik_serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for SiK radio'
    )

    return LaunchDescription([
        with_sik_radio_arg,
        serial_port_arg,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_joy'), 'launch', 'joy.launch.py'])])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_security'), 'launch', 'security_base.launch.py'])])),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rover_auxiliary'), 'launch', 'auxiliary_base.launch.py'])]),
            launch_arguments={
                'with_sik_radio': LaunchConfiguration('with_sik_radio'),
                'sik_serial_port': LaunchConfiguration('sik_serial_port')
            }.items())
    ])
