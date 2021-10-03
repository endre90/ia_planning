import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = FindPackageShare("bringup").find("bringup")
    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_1.rviz")

    scenario_parameters = {
        "scenario_path": "/ros/scenarios/scenario_1",
        "scenario": "scenario_1",
    }

    big_file_paths = {
        "big_file_paths": os.path.join(bringup_dir, "config", "paths.json")
    }

    sms_node = Node(
        package="sms",
        executable="sms",
        namespace="ia_planning",
        output="screen",
        parameters=[scenario_parameters],
    )

    static_visualization_node = Node(
        package="static_visualization",
        executable="static_visualization",
        namespace="ia_planning",
        output="screen",
        parameters=[scenario_parameters, big_file_paths],
    )

    cube_handler_node = Node(
        package="handlers",
        executable="cube_handler",
        namespace="ia_planning",
        output="screen",
    )

    nodes_to_start = [
        sms_node,
        static_visualization_node,
        cube_handler_node,
    ]

    return LaunchDescription(nodes_to_start)
