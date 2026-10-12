import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(get_package_share_directory("flexigrasp"), "config", "flexigrasp.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_cfg,
                              description="YAML with flexigrasp_node / flexigrasp_gui parameters"),
        DeclareLaunchArgument("gui", default_value="true", description="start the live viewer"),
        DeclareLaunchArgument("demo", default_value="false",
                              description="simulated pads, no hardware needed"),
        Node(package="flexigrasp", executable="flexigrasp_node", name="flexigrasp_node",
             output="screen",
             parameters=[LaunchConfiguration("params_file"),
                         {"demo": LaunchConfiguration("demo")}]),
        Node(package="flexigrasp", executable="flexigrasp_gui", name="flexigrasp_gui",
             output="screen", condition=IfCondition(LaunchConfiguration("gui")),
             parameters=[LaunchConfiguration("params_file")]),
    ])
