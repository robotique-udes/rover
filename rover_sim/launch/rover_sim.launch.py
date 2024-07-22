import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    simulation_arg = DeclareLaunchArgument(
        'simulate_arm',
        default_value='false',
        description='Flag to enable arm simulation'
    )
    
    simulate_arm = LaunchConfiguration('simulate_arm')
    
    urdf_file = "robot.urdf"
    pkg_desc = "rover_sim"
    robot_desc_path = os.path.join(get_package_share_directory(pkg_desc), "rover_urdf", urdf_file)

    with open(robot_desc_path, 'r') as file:
        robot_desc = file.read()

    rviz_config_dir = os.path.join(get_package_share_directory(pkg_desc), 'rover_rviz', 'rover.rviz')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        output="screen",
        condition=IfCondition(simulate_arm)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='/rover/arm/sim',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir],
        condition=IfCondition(simulate_arm)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='/rover/arm/sim',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(simulate_arm)
    )
    
    sim_node = Node(
        package="rover_sim",
        namespace="/rover/arm/sim",
        executable="simulation",
        name="simulation",
        condition=IfCondition(simulate_arm)
    )
    
    encoder_sim_node = Node(
        package="rover_sim",
        namespace="/rover/arm/sim",
        executable="encoder_simulation",
        name="encoder_simulation",
        condition=IfCondition(simulate_arm)
    )            

    return LaunchDescription([
        simulation_arg,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher,
        sim_node,
        encoder_sim_node
    ])
