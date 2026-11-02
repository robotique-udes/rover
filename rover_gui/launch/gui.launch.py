import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    supp = os.path.join(
        get_package_share_directory("rover_gui"), "launch", "valgrind.supp"
    )
    valgrind_prefix = (
        f"valgrind --leak-check=full --show-leak-kinds=definite "
        f"--track-origins=yes --num-callers=30 "
        f"--suppressions={supp} --log-file=/tmp/main_gui.log"
    )

    use_valgrind = LaunchConfiguration("use_valgrind")

    def make_node(prefix, condition):
        return Node(
            package="rover_gui",
            namespace="base/gui",
            executable="main_gui",
            name="main_gui",
            prefix=prefix,
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_valgrind", default_value="false"),
        make_node(valgrind_prefix, IfCondition(use_valgrind)),
        make_node("", UnlessCondition(use_valgrind)),
    ])