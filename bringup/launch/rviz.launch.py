import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = FindPackageShare("bringup").find("bringup")
    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_1.rviz")


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="ia_planning",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
