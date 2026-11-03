import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    gui_valgrind = make_node(valgrind_prefix, IfCondition(use_valgrind))
    gui_normal = make_node("", UnlessCondition(use_valgrind))

    # Same file that the QProcess used to start, now supervised by this launch
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_msgs"), "launch", "base.launch.py"
            )
        )
    )

    def shutdown_when_exits(gui):
        return RegisterEventHandler(
            OnProcessExit(
                target_action=gui,
                on_exit=[EmitEvent(event=Shutdown(reason="main_gui exited"))],
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_valgrind", default_value="false"),
        base,
        gui_valgrind,
        gui_normal,
        # Only the handler for the node that actually ran will ever fire
        shutdown_when_exits(gui_valgrind),
        shutdown_when_exits(gui_normal),
    ])