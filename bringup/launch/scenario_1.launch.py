import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = FindPackageShare("bringup").find("bringup")
    rviz_config_file = os.path.join(bringup_dir, "config", "scenario_1.rviz")

    scenario_parameters = {
        "scenario_path": "/home/endre/ia_ws/src/ia_planning/scenarios/scenario_1",
        "scenario": "scenario_1",
    }

    big_file_paths = {
        "big_file_paths": os.path.join(bringup_dir, "config", "paths.json")
    }

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="ia_planning",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

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

    include_tars = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "tars.launch.py")
        )
    )
    include_case = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "case.launch.py")
        )
    )

    nodes_to_start = [
        # include_tars,
        # include_case,
        sms_node,
        rviz_node,
        static_visualization_node,
    ]

    return LaunchDescription(nodes_to_start)
